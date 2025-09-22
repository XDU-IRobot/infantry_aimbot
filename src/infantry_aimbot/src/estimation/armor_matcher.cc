#include "estimation/armor_matcher.hpp"
#include "estimation/matcher.hpp"
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace ia {
namespace estimation {

ArmorMatcher::ArmorMatcher() {}
ArmorMatcher::~ArmorMatcher() {}

static size_t AllocateMinUnused(std::unordered_set<size_t>& used) {
  size_t id = 1;
  while (used.count(id)) ++id;
  used.insert(id);
  return id;
}

// 按 num_id 分组进行匹配；数量不等时仅匹配较少的一侧
result_sp<std::vector<std::pair<size_t, size_t>>> MatchArmor(std::shared_ptr<std::vector<Armor>> armor_now,
                                                             std::shared_ptr<std::vector<Armor>> armor_last) {
  if (!armor_now || !armor_last) {
    return outcome_v2::failure(std::make_error_code(std::errc::invalid_argument));
  }

  const auto& now = *armor_now;
  const auto& last = *armor_last;

  // 按 num_id 分桶索引
  std::unordered_map<int, std::vector<size_t>> now_by_id, last_by_id;
  now_by_id.reserve(now.size());
  last_by_id.reserve(last.size());
  for (size_t i = 0; i < now.size(); ++i) now_by_id[now[i].num_id].push_back(i);
  for (size_t j = 0; j < last.size(); ++j) last_by_id[last[j].num_id].push_back(j);

  auto matches = std::make_shared<std::vector<std::pair<size_t, size_t>>>();
  matches->reserve(std::min(now.size(), last.size()));

  const double INF = 1e18;

  // 针对同时出现在两组中的 num_id 进行匹配
  for (const auto& kv : last_by_id) {
    const int id = kv.first;
    const auto& last_idx_vec = kv.second;

    auto it_now = now_by_id.find(id);
    if (it_now == now_by_id.end()) continue;

    const auto& now_idx_vec = it_now->second;
    const size_t m = last_idx_vec.size();
    const size_t n = now_idx_vec.size();
    if (m == 0 || n == 0) continue;

    // 使用通用匈牙利：惰性成本评估，采用平方距离（避免 sqrt）
    auto cost = [&](size_t i, size_t j) {
      const auto& al = last[last_idx_vec[i]];
      const auto& an = now[now_idx_vec[j]];
      const auto &pa = al.pose.position, &pb = an.pose.position;
      const double dx = pa.x - pb.x, dy = pa.y - pb.y, dz = pa.z - pb.z;
      const double d2 = dx * dx + dy * dy + dz * dz;
      return d2;  // 若要阈值裁剪，可在大于阈值时返回 INF
    };

    // 直接基于大小 m x n 的惰性成本进行匹配
    auto assign = ia::estimation::HungarianAssign(m, n, cost, INF);

    // 收集匹配对（仅真实行/列，且过滤掉无效大代价）
    std::vector<std::pair<size_t, size_t>> pairs;  // (now_index, last_index)
    pairs.reserve(std::min(m, n));
    for (size_t i = 0; i < m; ++i) {
      int j = assign[i];
      if (j < 0 || static_cast<size_t>(j) >= n) continue;
      if (cost(i, static_cast<size_t>(j)) >= INF * 0.5) continue;
      size_t li = last_idx_vec[i];
      size_t nj = now_idx_vec[static_cast<size_t>(j)];
      pairs.emplace_back(nj, li);
    }

    // 在该 num_id 组内为匹配对设置 group_id
    std::unordered_set<size_t> used_gid;
    used_gid.reserve(pairs.size());
    for (auto [nj, li] : pairs) {
      const auto& al = (*armor_last)[li];
      if (al.group_id != 0) used_gid.insert(al.group_id);
    }

    for (auto [nj, li] : pairs) {
      auto& al = (*armor_last)[li];
      auto& an = (*armor_now)[nj];

      if (al.group_id != 0) {
        an.group_id = al.group_id;
        used_gid.insert(al.group_id);
      } else {
        const size_t gid = AllocateMinUnused(used_gid);
        al.group_id = gid;
        an.group_id = gid;
      }

      matches->emplace_back(nj, li);
    }
  }

  return outcome_v2::success(matches);
}

}  // namespace estimation
}  // namespace ia