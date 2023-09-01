#pragma once
#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pf {
class RosSender {
 public:
  RosSender(std::shared_ptr<rclcpp::Node> pNode);
  ~RosSender();
  void send_ground_truth_as_tf2(Eigen::Vector3f state);
  void send_estimation_as_tf2(Eigen::Vector3f state);
  void send_ground_truth_as_Pose2D(Eigen::Vector3f state);
  void send_estimate_as_Pose2D(Eigen::Vector3f state);
  void send_bearings_as_arrows(Eigen::Vector4f bearings);
  void send_particles_as_pointcloud(std::vector<std::pair<float, float>>& particles);
  void update_timestamp();

 private:
  rclcpp::Time currTime;
  std::shared_ptr<rclcpp::Node> pNode;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pPosePublisher;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pEstimatePublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pBearingsPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pCloudPublisher;
  tf2_ros::TransformBroadcaster tfBroadcaster;
  tf2::Quaternion q;
  geometry_msgs::msg::TransformStamped tfStamped;
  geometry_msgs::msg::Pose2D poseMsg;
};
}  // namespace pf