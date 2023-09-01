#include <particle_filter/noise.hpp>

namespace pf {
Noise::Noise(): generator(rd()) {}

float Noise::sample_value(float mu, float std) {
  std::normal_distribution<float> distribution(mu, std);
  return distribution(generator);
}
}  // namespace pf