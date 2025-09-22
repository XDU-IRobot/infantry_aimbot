#pragma once

#include <vector>
#include <limits>
#include <type_traits>
#include <algorithm>

namespace ia {
namespace estimation {

// 通用匈牙利最小权匹配（支持非方阵），惰性成本访问器：cost(i,j)
// 要求：返回有限 double；若想禁用边，返回 >= INF/2 的大值
template <typename CostFunc>
inline std::vector<int> HungarianAssign(size_t rows, size_t cols, CostFunc cost,
                                        double INF = 1e18) {
  // 若行>列，转置以降低复杂度，并在末尾反转结果
  if (rows > cols) {
    auto costT = [&](size_t j, size_t i) { return cost(i, j); };
    auto assignRight = HungarianAssign(cols, rows, costT, INF); // size = cols, map col->row
    std::vector<int> assignRow(rows, -1);
    for (size_t j = 0; j < cols; ++j) {
      int i = assignRight[j];
      if (i >= 0 && static_cast<size_t>(i) < rows) {
        assignRow[static_cast<size_t>(i)] = static_cast<int>(j);
      }
    }
    return assignRow; // size = rows, map row->col
  }

  // 现在 rows <= cols
  const size_t m = rows, n = cols;

  std::vector<double> u(m + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

  std::vector<double> minv(n + 1);
  std::vector<char> used(n + 1);

  for (size_t i = 1; i <= m; ++i) {
    p[0] = static_cast<int>(i);
    size_t j0 = 0;
    std::fill(minv.begin(), minv.end(), INF);
    std::fill(used.begin(), used.end(), 0);

    do {
      used[j0] = 1;
      int i0 = p[j0];
      size_t j1 = 0;
      double delta = INF;

      for (size_t j = 1; j <= n; ++j) if (!used[j]) {
        double cur = cost(static_cast<size_t>(i0 - 1), static_cast<size_t>(j - 1)) - u[static_cast<size_t>(i0)] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = static_cast<int>(j0); }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }

      for (size_t j = 0; j <= n; ++j) {
        if (used[j]) { u[static_cast<size_t>(p[j])] += delta; v[j] -= delta; }
        else { minv[j] -= delta; }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      size_t j1 = static_cast<size_t>(way[j0]);
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> assign(static_cast<size_t>(m), -1); // 行 i -> 列 j
  for (size_t j = 1; j <= n; ++j)
    if (p[j] != 0)
      assign[static_cast<size_t>(p[j] - 1)] = static_cast<int>(j - 1);
  return assign;
}

// 便捷包装：对任意容器进行匹配，返回 (left_index, right_index) 对
// costFn(left[i], right[j]) -> double；返回 >= INF/2 则视为不可连边
template <typename LeftRange, typename RightRange, typename CostFn>
inline std::vector<std::pair<size_t, size_t>>
MinCostPairs(const LeftRange& left, const RightRange& right, CostFn costFn, double INF = 1e18) {
  const size_t m = left.size();
  const size_t n = right.size();
  if (m == 0 || n == 0) return {};

  auto cost = [&](size_t i, size_t j) {
    double c = costFn(left[i], right[j]);
    return c;
  };

  auto assign = HungarianAssign(m, n, cost, INF);
  std::vector<std::pair<size_t, size_t>> pairs;
  pairs.reserve(std::min(m, n));
  for (size_t i = 0; i < m; ++i) {
    int j = assign[i];
    if (j < 0 || static_cast<size_t>(j) >= n) continue;
    if (cost(i, static_cast<size_t>(j)) >= INF * 0.5) continue;
    pairs.emplace_back(i, static_cast<size_t>(j));
  }
  return pairs;
}

}  // namespace estimation
}  // namespace ia