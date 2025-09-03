#pragma once

#include <vector>
#include <memory>

#include "typedefs.hpp"
#include "estimation/armor_data_buffer.hpp"

namespace ia {
namespace estimation {
class ArmorTracker {
 public:
  ArmorTracker();
  ~ArmorTracker();
  // 判断装甲板的组别，返回和装甲板vector等长的int vector，-1表示无法判断，0-1代表所在组别
  result_sp<std::vector<size_t>> MatchArmor(std::shared_ptr<std::vector<Armor>> armor_now, std::shared_ptr<std::vector<Armor>> armor_last);

 private:
};
}  // namespace estimation
}  // namespace ia