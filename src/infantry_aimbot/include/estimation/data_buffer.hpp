#pragma once

// header-only

#include <vector>
#include <memory>
#include <chrono>
#include <shared_mutex>
#include <mutex>
#include <system_error>

#include <outcome.hpp>

using time_point = std::chrono::steady_clock::time_point;

namespace ia {
namespace estimation {

template <typename T>
using sp = std::shared_ptr<T>;
template <typename T>
using up = std::unique_ptr<T>;

template <typename T>
using result_sp = outcome_v2::result<sp<T>>;
template <typename T>
using result_up = outcome_v2::result<up<T>>;

using result_void = outcome_v2::result<void>;

/**
 * 模板化环形数据缓冲区：
 * - 按时间戳严格递增插入
 * - 支持最大容量（满时覆盖最旧）
 * - 支持超时淘汰（可禁用）
 * - 线程安全：读共享，写独占
 * - 存储为 shared_ptr<T> 与时间戳
 */
template <typename T>
class DataBuffer {
 public:
  using value_type = T;
  using ptr = std::shared_ptr<T>;

  DataBuffer() = default;
  ~DataBuffer() = default;

  // 初始化固定容量（>0）
  result_void Initialize(size_t size) {
    if (size == 0) {
      return outcome_v2::failure(std::make_error_code(std::errc::invalid_argument));
    }
    std::unique_lock lock(mtx_);
    buffer_.assign(size, {});  // 重置为 size 个空槽
    next_ = 0;
    count_ = 0;
    capacity_ = size;
    initialized_ = true;
    has_last_ts_ = false;
    last_ts_ = time_point{};
    // 重置超时状态
    use_timeout_ = (max_age_ > std::chrono::steady_clock::duration::zero());
    return outcome_v2::success();
  }

  // 按时间递增插入（严格递增），若启用超时则超时数据会被丢弃
  result_void AddData(ptr data, time_point timestamp) {
    if (!initialized_) {
      return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
    }
    if (!data) {
      return outcome_v2::failure(std::make_error_code(std::errc::invalid_argument));
    }

    std::unique_lock lock(mtx_);

    // 保证时间戳严格递增
    if (has_last_ts_ && !(timestamp > last_ts_)) {
      return outcome_v2::failure(std::make_error_code(std::errc::invalid_argument));
    }

    // 若启用超时：丢弃“已超时”的输入（相对当前时刻）
    if (use_timeout_) {
      const auto now = std::chrono::steady_clock::now();
      if ((now - timestamp) > max_age_) {
        return outcome_v2::failure(std::make_error_code(std::errc::timed_out));
      }
    }

    auto &slot = buffer_[next_];
    slot.data = std::move(data);
    slot.ts = timestamp;

    last_ts_ = timestamp;
    has_last_ts_ = true;

    next_ = (next_ + 1) % capacity_;
    if (count_ < capacity_) {
      ++count_;
    }

    // 插入后进行一次过期剔除（保持 count_ 与有效数据一致）
    if (use_timeout_) {
      PruneExpiredUnlocked(std::chrono::steady_clock::now());
    }

    return outcome_v2::success();
  }

  // 便捷重载：按值/右值插入
  result_void AddData(const T& data, time_point timestamp) {
    return AddData(std::make_shared<T>(data), timestamp);
  }
  result_void AddData(T&& data, time_point timestamp) {
    return AddData(std::make_shared<T>(std::move(data)), timestamp);
  }

  // 获取最新数据（若启用超时且最新已过期，则返回 no_message）
  result_sp<T> GetLatestData() const {
    if (!initialized_) {
      return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
    }
    std::shared_lock lock(mtx_);
    if (count_ == 0) {
      return outcome_v2::failure(std::make_error_code(std::errc::no_message));
    }

    const size_t idx = (next_ + capacity_ - 1) % capacity_;
    const auto &slot = buffer_[idx];
    if (slot.data) {
      if (use_timeout_) {
        const auto now = std::chrono::steady_clock::now();
        if ((now - slot.ts) > max_age_) {
          return outcome_v2::failure(std::make_error_code(std::errc::no_message));
        }
      }
      return outcome_v2::success(slot.data);
    }

    return outcome_v2::failure(std::make_error_code(std::errc::no_message));
  }

  // id代表数据的新旧，0为最新，1为次新，依此类推（若索引处数据已过期，则返回 no_message）
  result_sp<T> GetDataById(int id) const {
    if (!initialized_) {
      return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
    }
    if (id < 0) {
      return outcome_v2::failure(std::make_error_code(std::errc::invalid_argument));
    }
    std::shared_lock lock(mtx_);
    if (static_cast<size_t>(id) >= count_) {
      // 请求的“旧度”超过当前缓冲的有效数量
      return outcome_v2::failure(std::make_error_code(std::errc::no_message));
    }

    // 0=最新 -> (next_-1)，1=更旧 -> (next_-2) ...
    const size_t idx = (next_ + capacity_ - 1 - static_cast<size_t>(id)) % capacity_;
    const auto &slot = buffer_[idx];
    if (slot.data) {
      if (use_timeout_) {
        const auto now = std::chrono::steady_clock::now();
        if ((now - slot.ts) > max_age_) {
          return outcome_v2::failure(std::make_error_code(std::errc::no_message));
        }
      }
      return outcome_v2::success(slot.data);
    }
    // 不应出现：有 count_ 保障
    return outcome_v2::failure(std::make_error_code(std::errc::no_message));
  }

  result_sp<size_t> GetSize() const {
    if (!initialized_) {
      return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
    }
    std::shared_lock lock(mtx_);
    return outcome_v2::success(std::make_shared<size_t>(count_));
  }

  // 启用超时，设置最大保留时间；max_age<=0 等价于禁用
  result_void SetTimeout(std::chrono::steady_clock::duration max_age) {
    std::unique_lock lock(mtx_);
    if (max_age <= std::chrono::steady_clock::duration::zero()) {
      use_timeout_ = false;
      max_age_ = std::chrono::steady_clock::duration::zero();
      return outcome_v2::success();
    }
    use_timeout_ = true;
    max_age_ = max_age;

    // 即时清理当前已过期数据（从最旧开始剔除）
    if (initialized_ && count_ > 0) {
      PruneExpiredUnlocked(std::chrono::steady_clock::now());
    }
    return outcome_v2::success();
  }

  // 手动触发一次过期清理（可选）
  result_void PurgeExpired() {
    if (!initialized_) {
      return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
    }
    std::unique_lock lock(mtx_);
    if (!use_timeout_) {
      return outcome_v2::success();
    }
    PruneExpiredUnlocked(std::chrono::steady_clock::now());
    return outcome_v2::success();
  }

 private:
  struct Slot {
    ptr data;
    time_point ts{};
  };

  std::vector<Slot> buffer_;

  // 写入位置（下一次写入的下标）、当前有效数量、容量
  size_t next_{0};
  size_t count_{0};
  size_t capacity_{0};

  mutable std::shared_mutex mtx_;
  bool initialized_{false};

  // 用于确保时间戳严格递增
  bool has_last_ts_{false};
  time_point last_ts_{};

  // 超时控制
  bool use_timeout_{false};
  std::chrono::steady_clock::duration max_age_{};

  // 仅在持有独占锁时调用：从最旧开始剔除已过期元素，维护 count_
  void PruneExpiredUnlocked(time_point now) {
    if (!use_timeout_ || count_ == 0) return;

    while (count_ > 0) {
      const size_t oldest = (next_ + capacity_ - count_) % capacity_;
      auto &slot = buffer_[oldest];
      if (slot.data && (now - slot.ts) > max_age_) {
        slot.data.reset();
        --count_;
        continue;
      }
      break;
    }
  }
};

}  // namespace estimation
}  // namespace ia