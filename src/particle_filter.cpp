#include <particle_filter/particle_filter.hpp>

namespace pf {
ParticleFilter::ParticleFilter(Robot* pRobot, uint32_t n): pRobot(pRobot) {
  particles.reserve(n);
  std::random_device rd;   // Seed for the random number engine
  std::mt19937 gen(rd());  // Mersenne Twister random number engine
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> ang_dist(-M_PI, M_PI);
  float init_weight = 1.0f / n;
  for (uint32_t i = 0; i < n; ++i) {
    Eigen::Vector3f rand_state(pos_dist(gen), pos_dist(gen), ang_dist(gen));
    auto robot = Robot(rand_state, pRobot->corners);
    particles.emplace_back(std::make_pair(robot, init_weight));
  }
}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::move(float alpha, float d) {
  // Motion step
  for (auto& particle: particles) {
    particle.first.move(alpha, d);
  }

  // Measure step
  for (uint32_t i = 0; i < particles.size(); ++i) {
    particles[i].first.sense();
    // std::cout << particles[i].first->z.transpose() << std::endl;
    Eigen::Vector4f probs;
    for (int j = 0; j < 4; ++j) {
      probs(j) = find_probability(particles[i].first.z(j), pRobot->z(j), 0.5f);
    }
    // std::cout << (particles[i].first->z - pRobot->z).transpose() << std::endl;
    particles[i].second = probs.prod();
  }
  normalize_weights();

  // Resample step
  float maxWeight = 0.0f;
  for (auto& particle: particles) {
    if (particle.second > maxWeight) maxWeight = particle.second;
  }
  std::random_device rd;                                                    // Seed for the random number engine
  std::mt19937 gen(rd());                                                   // Mersenne Twister random number engine
  std::uniform_int_distribution<int> uniform_int(0, particles.size() - 1);  // Uniform distribution between 0 and 6
  std::uniform_real_distribution<float> uniform_float(0, 2 * maxWeight);    // Uniform distribution between 0 and 6
  uint32_t index = uniform_int(gen);
  float betta;
  std::vector<std::pair<Robot, float>> newPartilces;
  newPartilces.reserve(particles.size());
  for (uint32_t i = 0; i < particles.size(); ++i) {
    betta += uniform_float(gen);
    while (betta > particles[index].second) {
      betta -= particles[index].second;
      index = (index + 1) % particles.size();
    }
    newPartilces.emplace_back(particles[index]);
  }
  particles = std::move(newPartilces);
  normalize_weights();
}

Eigen::Vector3f ParticleFilter::estimate_value() {
  Eigen::Vector3f estimate = Eigen::Vector3f::Zero();
  for (const auto& particle: particles) {
    estimate += particle.first.state * particle.second;
  }
  return estimate;
}

float ParticleFilter::find_probability(float mu, float mu0, float sigma) {
  float exponent = -0.5 * std::pow((mu - mu0) / sigma, 2);
  float coef = 1.0 / (sigma * std::sqrt(2 * M_PI));
  return coef * std::exp(exponent);
}
void ParticleFilter::normalize_weights() {
  float sum = 0.0f;
  for (auto& particle: particles) {
    sum += particle.second;
  }
  for (auto& particle: particles) {
    particle.second /= sum;
  }
}
std::vector<std::pair<float, float>> ParticleFilter::get_particles_poses() {
  auto poses = std::vector<std::pair<float, float>>();
  poses.reserve(particles.size());
  for (auto& particle: particles) {
    poses.emplace_back(std::make_pair(particle.first.state(0), particle.first.state(1)));
  }
  return poses;
}
}  // namespace pf