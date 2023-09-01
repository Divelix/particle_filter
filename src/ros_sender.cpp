#include <particle_filter/ros_sender.hpp>

namespace pf {
RosSender::RosSender(std::shared_ptr<rclcpp::Node> pNode): pNode(pNode), tfBroadcaster(pNode) {
  pPosePublisher = pNode->create_publisher<geometry_msgs::msg::Pose2D>("/ground_truth", 10);
  pEstimatePublisher = pNode->create_publisher<geometry_msgs::msg::Pose2D>("/position", 10);
  pBearingsPublisher = pNode->create_publisher<visualization_msgs::msg::MarkerArray>("/bearings", 10);
  pCloudPublisher = pNode->create_publisher<sensor_msgs::msg::PointCloud2>("/particles", 10);
}
RosSender::~RosSender() {}
void RosSender::send_ground_truth_as_tf2(Eigen::Vector3f state) {
  // Broadcast TF
  tfStamped.header.stamp = currTime;
  tfStamped.header.frame_id = "map";
  tfStamped.child_frame_id = "ground_truth";
  tfStamped.transform.translation.x = state(0);
  tfStamped.transform.translation.y = state(1);
  tfStamped.transform.translation.z = 0;
  q.setRPY(0.0, 0.0, state(2));
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();
  tfBroadcaster.sendTransform(tfStamped);
}
void RosSender::send_estimation_as_tf2(Eigen::Vector3f state) {
  // Broadcast TF
  tfStamped.header.stamp = currTime;
  tfStamped.header.frame_id = "map";
  tfStamped.child_frame_id = "particle_filter";
  tfStamped.transform.translation.x = state(0);
  tfStamped.transform.translation.y = state(1);
  tfStamped.transform.translation.z = 0;
  q.setRPY(0.0, 0.0, state(2));
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();
  tfBroadcaster.sendTransform(tfStamped);
}
void RosSender::send_ground_truth_as_Pose2D(Eigen::Vector3f state) {
  poseMsg.x = state(0);
  poseMsg.y = state(1);
  poseMsg.theta = state(2);
  pPosePublisher->publish(poseMsg);
}
void RosSender::send_estimate_as_Pose2D(Eigen::Vector3f state) {
  poseMsg.x = state(0);
  poseMsg.y = state(1);
  poseMsg.theta = state(2);
  pEstimatePublisher->publish(poseMsg);
}
void RosSender::send_bearings_as_arrows(Eigen::Vector4f bearings) {
  visualization_msgs::msg::MarkerArray markersMsg;
  for (int i = 0; i < bearings.size(); ++i) {
    visualization_msgs::msg::Marker markerMsg;
    markerMsg.header.frame_id = "ground_truth";
    markerMsg.header.stamp = currTime;
    markerMsg.ns = "arrows";
    markerMsg.id = i;
    markerMsg.type = visualization_msgs::msg::Marker::ARROW;
    markerMsg.action = visualization_msgs::msg::Marker::ADD;
    markerMsg.pose.position.x = 0.0;
    markerMsg.pose.position.y = 0.0;
    markerMsg.pose.position.z = 0.0;
    q.setRPY(0.0, 0.0, bearings(i));
    markerMsg.pose.orientation.x = q.x();
    markerMsg.pose.orientation.y = q.y();
    markerMsg.pose.orientation.z = q.z();
    markerMsg.pose.orientation.w = q.w();
    markerMsg.scale.x = 1.0;
    markerMsg.scale.y = 0.1;
    markerMsg.scale.z = 0.1;
    markerMsg.color.r = 1.0;
    markerMsg.color.g = 0.0;
    markerMsg.color.b = 0.0;
    markerMsg.color.a = 1.0;
    // Add the marker to the MarkerArray
    markersMsg.markers.push_back(markerMsg);
  }
  pBearingsPublisher->publish(markersMsg);
}

void RosSender::send_particles_as_pointcloud(std::vector<std::pair<float, float>>& particles) {
  sensor_msgs::msg::PointCloud2 pcMsg;
  pcMsg.header.frame_id = "map";
  pcMsg.header.stamp = currTime;
  pcMsg.height = 1;
  pcMsg.width = particles.size();
  pcMsg.fields.resize(3);
  pcMsg.fields[0].name = "x";
  pcMsg.fields[0].offset = 0;
  pcMsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pcMsg.fields[0].count = 1;
  pcMsg.fields[1].name = "y";
  pcMsg.fields[1].offset = 4;
  pcMsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pcMsg.fields[1].count = 1;
  pcMsg.fields[2].name = "z";
  pcMsg.fields[2].offset = 8;
  pcMsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pcMsg.fields[2].count = 1;
  pcMsg.is_bigendian = false;
  pcMsg.point_step = 12;
  pcMsg.row_step = pcMsg.point_step * pcMsg.width;
  pcMsg.is_dense = true;
  pcMsg.data.resize(pcMsg.row_step * pcMsg.height);
  sensor_msgs::PointCloud2Iterator<float> iter_x(pcMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pcMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pcMsg, "z");
  for (const auto& particle: particles) {
    *iter_x = particle.first;
    *iter_y = particle.second;
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  pCloudPublisher->publish(pcMsg);
}
void RosSender::update_timestamp() { currTime = pNode->now(); }
}  // namespace pf