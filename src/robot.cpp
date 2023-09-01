#include <particle_filter/robot.hpp>

namespace pf {
Robot::Robot(Eigen::Vector3f& init_state, Eigen::Matrix<float, 4, 2>& corners): state(init_state), corners(corners) {
  noise = std::make_shared<Noise>();
}
Robot::~Robot() {}

void Robot::move(float alpha, float d) {
  // Add noise
  alpha += noise->sample_value(0.0f, angle_std);
  d += noise->sample_value(0.0f, distance_std);

  // Calc kinematics
  float R = L / tan(alpha);
  float beta = d / R;
  float xc = state(0) - R * sin(state(2));
  float yc = state(1) + R * cos(state(2));

  // Update state
  state(0) = xc + R * sin(state(2) + beta);
  state(1) = yc - R * cos(state(2) + beta);
  state(2) = fmod(state(2) + beta, 2.0f * M_PI);
}

void Robot::sense() {
  auto pos2tl = corners.row(0).transpose() - state.head<2>();
  auto pos2tr = corners.row(1).transpose() - state.head<2>();
  auto pos2br = corners.row(2).transpose() - state.head<2>();
  auto pos2bl = corners.row(3).transpose() - state.head<2>();
  z = Eigen::Vector4f(
      std::atan2(pos2tl.y(), pos2tl.x()) - state(2) + noise->sample_value(0.0f, measurement_std),
      std::atan2(pos2tr.y(), pos2tr.x()) - state(2) + noise->sample_value(0.0f, measurement_std),
      std::atan2(pos2br.y(), pos2br.x()) - state(2) + noise->sample_value(0.0f, measurement_std),
      std::atan2(pos2bl.y(), pos2bl.x()) - state(2) + noise->sample_value(0.0f, measurement_std));
}
}  // namespace pf