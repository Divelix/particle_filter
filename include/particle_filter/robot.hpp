#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>

#include "particle_filter/noise.hpp"

namespace pf {
class Robot {
 public:
  Robot(Eigen::Vector3f& init_state, Eigen::Matrix<float, 4, 2>& corners);
  ~Robot();
  void move(float alpha, float d);
  void sense();
  Eigen::Vector3f state;
  Eigen::Vector4f z;
  Eigen::Matrix<float, 4, 2> corners;

 private:
  std::shared_ptr<Noise> noise;
  const float L = 1.0f;  // distance between back and front wheels
  const float angle_std = 0.01f;
  const float distance_std = 0.01f;
  const float measurement_std = 0.01f;
};
}  // namespace pf