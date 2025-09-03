#include "estimation/armor_tracker.hpp"
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace ia {
namespace estimation {

ArmorTracker::ArmorTracker() {}
ArmorTracker::~ArmorTracker() {}

// 匈牙利算法（最小权完备匹配），输入为方阵
static std::vector<int> HungarianMinCost(const std::vector<std::vector<double>>& a) {
  const int n = static_cast<int>(a.size());
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1), v(n + 1);
  std::vector<int> p(n + 1), way(n + 1);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, INF);
    std::vector<char> used(n + 1, false);
    do {
      used[j0] = true;
      int i0 = p[j0], j1 = 0;
      double delta = INF;
      for (int j = 1; j <= n; ++j) if (!used[j]) {
        double cur = a[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else { minv[j] -= delta; }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> assignment(n, -1);  // 行 i -> 列 j
  for (int j = 1; j <= n; ++j)
    if (p[j] != 0) assignment[p[j] - 1] = j - 1;
  return assignment;
}

static inline double PoseDist(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
  const auto &pa = a.position, &pb = b.position;
  const double dx = pa.x - pb.x, dy = pa.y - pb.y, dz = pa.z - pb.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

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
  for (size_t i = 0; i < now.size(); ++i)  now_by_id[now[i].num_id].push_back(i);
  for (size_t j = 0; j < last.size(); ++j) last_by_id[last[j].num_id].push_back(j);

  auto matches = std::make_shared<std::vector<std::pair<size_t, size_t>>>();
  matches->reserve(std::min(now.size(), last.size()));

  const double BIG = 1e9;

  // 针对同时出现在两组中的 num_id 进行匹配
  for (const auto& [id, last_idx_vec] : last_by_id) {
    auto it_now = now_by_id.find(id);
    if (it_now == now_by_id.end()) continue;

    const auto& now_idx_vec = it_now->second;
    const int m = static_cast<int>(last_idx_vec.size());
    const int n = static_cast<int>(now_idx_vec.size());
    const int N = std::max(m, n);

    // 构建 N×N 代价矩阵
    std::vector<std::vector<double>> cost(N, std::vector<double>(N, BIG));
    for (int i = 0; i < m; ++i) {
      const auto& al = last[last_idx_vec[i]];
      for (int j = 0; j < n; ++j) {
        const auto& an = now[now_idx_vec[j]];
        cost[i][j] = PoseDist(al.pose, an.pose);
      }
    }

    // 匈牙利算法（最小化）
    auto assign = HungarianMinCost(cost);

    // 收集匹配对（仅真实行/列）
    std::vector<std::pair<size_t,size_t>> pairs; // (now_index, last_index)
    pairs.reserve(std::min(m, n));
    for (int i = 0; i < m; ++i) {
      int j = assign[i];
      if (j < 0 || j >= n) continue;
      if (cost[i][j] >= BIG * 0.5) continue;
      size_t li = last_idx_vec[static_cast<size_t>(i)];
      size_t nj = now_idx_vec[static_cast<size_t>(j)];
      pairs.emplace_back(nj, li);
    }

    // 在该 num_id 组内为匹配对设置 group_id
    // 先把本组内（匹配对中的）非零 group_id 记为“已使用”，用于后续分配 0 情况的最小未用
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
        // 直接沿用 armor_last 的 group_id
        an.group_id = al.group_id;
        // 标记为已使用，避免后续 0 情况复用
        used_gid.insert(al.group_id);
      } else {
        // 为该对分配本 num_id 组内最小未用正整数
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