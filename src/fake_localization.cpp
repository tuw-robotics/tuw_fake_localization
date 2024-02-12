#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tuw_fake_localization/fake_localization.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

FakeLocalization::FakeLocalization(const std::string &node_name, bool intra_process_comms)
    : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  declare_default_parameter<std::string>("world_frame_id", "world", "Frame id of the world");
  declare_default_parameter<std::string>("map_frame_id", "map", "Frame id of the map");
  declare_default_parameter<std::string>("odom_frame_id", "odom", "Frame id of the odometry");
  declare_default_parameter<std::string>("base_frame_id", "base_link", "Frame id of the base link");
  declare_default_parameter<double>("offset_x", 0.0, "The x offset between the origin of the world (simulator) coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offset_y", 0.0, "The y offset between the origin of the world (simulator) coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offset_z", 0.0, "The z offset between the origin of the world (simulator) coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offset_yaw", 0.0, "The yaw offset between the origin of the world (simulator) coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offset_pitch", 0.0, "The pitch offset between the origin of the world (simulator) coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offset_roll", 0.0, "The roll offset between the origin of the world (simulator) coordinate frame and the map coordinate frame published by fake_localization.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  this->get_parameter<std::string>("world_frame_id", world_frame_id_);
  RCLCPP_INFO(get_logger(), "world_frame_id is set to: [%s]", world_frame_id_.c_str());
  this->get_parameter<std::string>("map_frame_id", map_frame_id_);
  RCLCPP_INFO(get_logger(), "map_frame_id is set to: [%s]", map_frame_id_.c_str());
  this->get_parameter<std::string>("odom_frame_id", odom_frame_id_);
  RCLCPP_INFO(get_logger(), "odom_frame_id is set to: [%s]", odom_frame_id_.c_str());
  this->get_parameter<std::string>("base_frame_id", base_frame_id_);
  RCLCPP_INFO(get_logger(), "base_frame_id is set to: [%s]", base_frame_id_.c_str());

  double offset_x, offset_y, offset_z, offset_yaw, offset_pitch, offset_roll;
  this->get_parameter<double>("offset_x", offset_x);
  this->get_parameter<double>("offset_y", offset_y);
  this->get_parameter<double>("offset_y", offset_z);
  this->get_parameter<double>("offset_roll", offset_roll);
  this->get_parameter<double>("offset_pitch", offset_pitch);
  this->get_parameter<double>("offset_yaw", offset_yaw);
  RCLCPP_INFO(get_logger(),
              "The offset [x, y, z; r, p y] is set to: [%f m, %f m, %f m; %f rad, %f rad, %f rad]",
              offset_x, offset_y, offset_y, offset_roll, offset_pitch, offset_yaw);

  tf2::Quaternion q;
  q.setRPY(-offset_roll, -offset_pitch, -offset_yaw);
  tf_world_map_ = tf2::Transform(q, tf2::Vector3(-offset_x, -offset_y, -offset_z));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  RCLCPP_INFO(get_logger(), "on_configure() called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_activate(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_activate(state);

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() called.");

  sub_ground_truth_ = create_subscription<nav_msgs::msg::Odometry>("odom_ground_truth", 10, std::bind(&FakeLocalization::callback_ground_truth, this, _1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_deactivate(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_deactivate(state);

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".
  sub_ground_truth_.reset();
  tf_broadcaster_.reset();
  tf_listener_.reset();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_shutdown(const rclcpp_lifecycle::State &state)
{
  sub_ground_truth_.reset();
  tf_broadcaster_.reset();
  tf_listener_.reset();

  RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown called from state %s.",
      state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void FakeLocalization::callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg_odom)
{

  // auto &q = msg_odom->pose.pose.orientation;
  // RCLCPP_INFO(this->get_logger(), "callback_ground_truth %f, %f, %f", p.x, p.y, p.z);

  geometry_msgs::msg::TransformStamped tf_odom;
  try
  {
    tf_odom = tf_buffer_->lookupTransform(
        odom_frame_id_, base_frame_id_, msg_odom->header.stamp, rclcpp::Duration::from_seconds(0.1));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        odom_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
    return;
  }

  tf2::fromMsg(tf_odom.transform, tf_odom_base_);
  tf2::fromMsg(msg_odom->pose.pose, tf_world_base_);
  tf_map_odom_ = tf_world_base_ * tf_odom_base_.inverse() * tf_world_map_.inverse();
  auto stamp = tf2_ros::fromMsg(msg_odom->header.stamp);
  tf2::TimePoint transform_expiration = stamp;

  sendWorldToMapTransform(transform_expiration);
  sendMapToOdomTransform(transform_expiration);
}

void FakeLocalization::sendWorldToMapTransform(const tf2::TimePoint &transform_expiration)
{
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = world_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = map_frame_id_;
  tf2::convert(tf_world_map_, tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}

void FakeLocalization::sendMapToOdomTransform(const tf2::TimePoint &transform_expiration)
{
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = map_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf2::convert(tf_map_odom_, tmp_tf_stamped.transform);
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}
