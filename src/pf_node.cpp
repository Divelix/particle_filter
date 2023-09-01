#include <iomanip>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "particle_filter/particle_filter.hpp"
#include "particle_filter/robot.hpp"
#include "particle_filter/ros_sender.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char* argv[]) {
  // Bearings positiions
  Eigen::Vector3f init_state(10.0f, 20.0f, 1.0f);
  Eigen::Matrix<float, 4, 2> corners;
  corners << -50.0f, 50.f,  // top left
      50.0f, 50.0f,         // top right
      50.0f, -50.0f,        // bottom right
      -50.0f, -50.0f;       // bottom left
  rclcpp::init(argc, argv);
  auto pNode = rclcpp::Node::make_shared("pf_node");
  pf::RosSender rs(pNode);
  pf::Robot robot(init_state, corners);
  pf::ParticleFilter pf(&robot, 10000);

  RCLCPP_INFO(pNode->get_logger(), "Initial robot state: (%f, %f, %f)", robot.state(0), robot.state(1), robot.state(2));
  // rclcpp::WallRate loop_rate(1);
  rclcpp::WallRate loop_rate(10);
  // for (uint8_t i = 0; i < 10; ++i) {
  while (rclcpp::ok()) {
    float control_alpha = -0.1f;
    float control_d = 0.5f;
    rs.update_timestamp();
    robot.move(control_alpha, control_d);
    robot.sense();
    pf.move(control_alpha, control_d);
    auto pfEstimate = pf.estimate_value();
    // RCLCPP_INFO_STREAM(pNode->get_logger(), "Iter " << i + 1 << ": Ground truth=(" << std::fixed << std::setprecision(2) << robot.state.transpose()
    // << ") | Estimation=(" << pfEstimate.transpose() << ")");
    RCLCPP_INFO_STREAM(
        pNode->get_logger(),
        "Ground truth=(" << std::fixed << std::setprecision(2) << robot.state.transpose() << ") | Estimation=(" << pfEstimate.transpose() << ")");
    rs.send_ground_truth_as_Pose2D(robot.state);
    rs.send_estimate_as_Pose2D(pfEstimate);
    rs.send_ground_truth_as_tf2(robot.state);
    rs.send_estimation_as_tf2(pfEstimate);
    rs.send_bearings_as_arrows(robot.z);
    auto poses = pf.get_particles_poses();
    rs.send_particles_as_pointcloud(poses);
    rclcpp::spin_some(pNode);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}