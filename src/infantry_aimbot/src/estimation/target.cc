#include "estimation/target.hpp"

#include <numeric>

#include "tools/math_tools.hpp"

namespace ia {
namespace estimation {

Target::Target(const Armor& armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
               const Eigen::VectorXd& P0_diag)
    : name(armor.name),
      armor_kind(armor.kind),
      jumped(false),
      last_id(0),
      armor_num_(armor_num),
      t_(t),
      is_converged_(false),
      switch_count_(0),
      update_count_(0) {
  priority = armor.priority;
  const Eigen::Vector3d& xyz = armor.xyz_in_world;
  const Eigen::Vector3d& ypr = armor.ypr_in_world;

  // 旋转中心坐标
  double center_x = xyz[0] + radius * std::cos(ypr[0]);
  double center_y = xyz[1] + radius * std::sin(ypr[0]);
  double center_z = xyz[2];

  // x vx y vy z vz a w r l h
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, radius, 0, 0}};
  Eigen::MatrixXd P0 = P0_diag.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::LimitRad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::Predict(std::chrono::steady_clock::time_point t) {
  double dt = tools::DeltaTime(t, t_);
  Predict(dt);
  t_ = t;
}

void Target::Predict(double dt) {
  // 状态转移矩阵 (匀速模型)
  // clang-format off
  Eigen::MatrixXd F{
      {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
      {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
      {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
      {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
      {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
      {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
      {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
      {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
      {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  double v1, v2;
  if (name == ArmorName::kOutpost) {
    v1 = 10;
    v2 = 0.1;
  } else {
    v1 = 100;
    v2 = 400;
  }
  double a = dt * dt * dt * dt / 4;
  double b = dt * dt * dt / 2;
  double c = dt * dt;
  // clang-format off
  Eigen::MatrixXd Q{
      {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
      {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
      {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
      {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
      {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
      {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
      {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
      {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
      {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
      {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
      {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::LimitRad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (Convergened() && name == ArmorName::kOutpost && std::abs(ekf_.x[7]) > 2) {
    ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;
  }

  ekf_.Predict(F, Q, f);
}

void Target::Update(const Armor& armor) {
  // 装甲板匹配
  int id = 0;
  double min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d>& xyza_list = ArmorXyzaList();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(xyza_i_list.begin(), xyza_i_list.end(),
            [](const std::pair<Eigen::Vector4d, int>& a, const std::pair<Eigen::Vector4d, int>& b) {
              Eigen::Vector3d ypd1 = tools::Xyz2Ypd(a.first.head(3));
              Eigen::Vector3d ypd2 = tools::Xyz2Ypd(b.first.head(3));
              return ypd1[2] < ypd2[2];
            });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < std::min(3, armor_num_); i++) {
    const auto& xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::Xyz2Ypd(xyza.head(3));
    double angle_error = std::abs(tools::LimitRad(armor.ypr_in_world[0] - xyza[3])) +
                         std::abs(tools::LimitRad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;
  if (id != last_id) switch_count_++;
  last_id = id;
  update_count_++;

  UpdateYpda(armor, id);
}

void Target::UpdateYpda(const Armor& armor, int id) {
  Eigen::MatrixXd H = HJacobian(ekf_.x, id);

  double center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  double delta_angle = tools::LimitRad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig{{4e-3, 4e-3, std::log(std::abs(delta_angle) + 1) + 1,
                         std::log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

  Eigen::MatrixXd R = R_dig.asDiagonal();

  auto h = [&](const Eigen::VectorXd& x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = HArmorXyz(x, id);
    Eigen::VectorXd ypd = tools::Xyz2Ypd(xyz);
    double angle = tools::LimitRad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  auto z_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::LimitRad(c[0]);
    c[1] = tools::LimitRad(c[1]);
    c[3] = tools::LimitRad(c[3]);
    return c;
  };

  const Eigen::Vector3d& ypd = armor.ypd_in_world;
  const Eigen::Vector3d& ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.Update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::EkfX() const { return ekf_.x; }

const tools::ExtendedKalmanFilter& Target::Ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::ArmorXyzaList() const {
  std::vector<Eigen::Vector4d> result;
  for (int i = 0; i < armor_num_; i++) {
    double angle = tools::LimitRad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = HArmorXyz(ekf_.x, i);
    result.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return result;
}

bool Target::Diverged() const {
  bool r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  bool l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;
  return !(r_ok && l_ok);
}

bool Target::Convergened() {
  if (name != ArmorName::kOutpost && update_count_ > 3 && !Diverged()) {
    is_converged_ = true;
  }
  if (name == ArmorName::kOutpost && update_count_ > 10 && !Diverged()) {
    is_converged_ = true;
  }
  return is_converged_;
}

Eigen::Vector3d Target::HArmorXyz(const Eigen::VectorXd& x, int id) const {
  double angle = tools::LimitRad(x[6] + id * 2 * CV_PI / armor_num_);
  bool use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  double r = use_l_h ? x[8] + x[9] : x[8];
  double armor_x = x[0] - r * std::cos(angle);
  double armor_y = x[2] - r * std::sin(angle);
  double armor_z = use_l_h ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::HJacobian(const Eigen::VectorXd& x, int id) const {
  double angle = tools::LimitRad(x[6] + id * 2 * CV_PI / armor_num_);
  bool use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  double r = use_l_h ? x[8] + x[9] : x[8];
  double dx_da = r * std::sin(angle);
  double dy_da = -r * std::cos(angle);
  double dx_dr = -std::cos(angle);
  double dy_dr = -std::sin(angle);
  double dx_dl = use_l_h ? -std::cos(angle) : 0.0;
  double dy_dl = use_l_h ? -std::sin(angle) : 0.0;
  double dz_dh = use_l_h ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
      {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
      {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
      {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
      {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = HArmorXyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::Xyz2YpdJacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
      {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
      {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
      {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
      {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

}  // namespace estimation
}  // namespace ia
