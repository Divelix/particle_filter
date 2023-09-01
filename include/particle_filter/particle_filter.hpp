#pragma once
// #include <algorithm>
#include <array>
#include <iostream>
// #include <memory>
#include <particle_filter/robot.hpp>

namespace pf {
class ParticleFilter {
 public:
  ParticleFilter(Robot* pRobot, uint32_t n);
  ~ParticleFilter();
  void move(float alpha, float d);
  Eigen::Vector3f estimate_value();
  std::vector<std::pair<float, float>> get_particles_poses();

 private:
  Robot* pRobot;
  std::vector<std::pair<Robot, float>> particles;  // Robot + weight

  float find_probability(float mu, float mu0, float sigma);
  void normalize_weights();
};
}  // namespace pf