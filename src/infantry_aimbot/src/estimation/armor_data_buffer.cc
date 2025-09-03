#include "estimation/armor_data_buffer.hpp"

#include <system_error>

namespace ia {
namespace estimation {

result_void DataBuffer::Initialize(size_t size) {
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

result_void DataBuffer::AddData(std::shared_ptr<std::vector<Armor>> data, time_point timestamp) {
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
  slot.first = std::move(data);
  slot.second = timestamp;

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

result_sp<std::vector<Armor>> DataBuffer::GetLatestData() const {
  if (!initialized_) {
    return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
  }
  std::shared_lock lock(mtx_);
  if (count_ == 0) {
    return outcome_v2::failure(std::make_error_code(std::errc::no_message));
  }

  const size_t idx = (next_ + capacity_ - 1) % capacity_;
  const auto &slot = buffer_[idx];
  if (slot.first) {
    if (use_timeout_) {
      const auto now = std::chrono::steady_clock::now();
      if ((now - slot.second) > max_age_) {
        return outcome_v2::failure(std::make_error_code(std::errc::no_message));
      }
    }
    return outcome_v2::success(slot.first);
  }

  return outcome_v2::failure(std::make_error_code(std::errc::no_message));
}

result_sp<std::vector<Armor>> DataBuffer::GetDataById(int id) const {
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
  if (slot.first) {
    if (use_timeout_) {
      const auto now = std::chrono::steady_clock::now();
      if ((now - slot.second) > max_age_) {
        return outcome_v2::failure(std::make_error_code(std::errc::no_message));
      }
    }
    return outcome_v2::success(slot.first);
  }
  // 不应出现：有 count_ 保障
  return outcome_v2::failure(std::make_error_code(std::errc::no_message));
}

result_sp<size_t> DataBuffer::GetSize() const {
  if (!initialized_) {
    return outcome_v2::failure(std::make_error_code(std::errc::operation_not_permitted));
  }
  std::shared_lock lock(mtx_);
  return outcome_v2::success(std::make_shared<size_t>(count_));
}

result_void DataBuffer::SetTimeout(std::chrono::steady_clock::duration max_age) {
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

result_void DataBuffer::PurgeExpired() {
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

void DataBuffer::PruneExpiredUnlocked(time_point now) {
  if (!use_timeout_ || count_ == 0) return;

  // 从最旧开始剔除，直到遇到未过期的为止
  while (count_ > 0) {
    const size_t oldest = (next_ + capacity_ - count_) % capacity_;
    auto &slot = buffer_[oldest];
    if (slot.first && (now - slot.second) > max_age_) {
      slot.first.reset();
      --count_;
      // 继续检查下一个最旧
      continue;
    }
    break;
  }
}

}  // namespace estimation
}  // namespace ia