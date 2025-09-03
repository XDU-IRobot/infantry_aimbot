#pragma once

#include <vector>
#include <memory>
#include <chrono>
#include <shared_mutex>

#include "typedefs.hpp"

using time_point = std::chrono::steady_clock::time_point;

namespace ia {
namespace estimation {
class DataBuffer {
 public:
  DataBuffer() = default;
  ~DataBuffer() = default;

  // 初始化固定容量
  result_void Initialize(size_t size);

  // 按时间递增插入（严格递增），若启用超时则超时数据会被丢弃
  result_void AddData(std::shared_ptr<std::vector<Armor>> data, time_point timestamp);

  // 获取最新数据（若启用超时且最新已过期，则返回 no_message）
  result_sp<std::vector<Armor>> GetLatestData() const;

  // id代表数据的新旧，0为最新，1为次新，依此类推（若索引处数据已过期，则返回 no_message）
  result_sp<std::vector<Armor>> GetDataById(int id) const;

  result_sp<size_t> GetSize() const;

  // 启用超时，设置最大保留时间；max_age<=0 等价于禁用
  result_void SetTimeout(std::chrono::steady_clock::duration max_age);

  // 手动触发一次过期清理（可选）
  result_void PurgeExpired();

 private:
  std::vector<std::pair<std::shared_ptr<std::vector<Armor>>, time_point>> buffer_;

  // 写入位置（下一次写入的下标）、当前有效数量、容量
  size_t next_{0};
  size_t count_{0};
  size_t capacity_{0};

  // 读写锁：写入独占，读取共享
  mutable std::shared_mutex mtx_;

  // 是否已初始化
  bool initialized_{false};

  // 用于确保时间戳严格递增
  bool has_last_ts_{false};
  time_point last_ts_{};

  // 超时控制
  bool use_timeout_{false};
  std::chrono::steady_clock::duration max_age_{};

  // 仅在持有独占锁时调用：从最旧开始剔除已过期元素，维护 count_
  void PruneExpiredUnlocked(time_point now);
};
}  // namespace estimation
}  // namespace ia