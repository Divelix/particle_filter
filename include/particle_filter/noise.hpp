#pragma once
#include <random>

namespace pf {
class Noise {
 public:
  Noise();
  float sample_value(float mu, float std);

 private:
  std::random_device rd;   // Obtain random seed from hardware
  std::mt19937 generator;  // Init the Mersenne Twister generator with the seed
};
}  // namespace pf