#include "tuw_fake_localization/fake_localization.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

FakeLocalization::FakeLocalization(const std::string &node_name, bool intra_process_comms)
    : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{

  declare_default_parameter<std::string>("global_frame_id", "/map", "The frame in which to publish the global_frame_id→odom_frame_id transform over tf.");
  declare_default_parameter<std::string>("odom_frame_id", "odom", "The name of the odometric frame of the robot");
  declare_default_parameter<std::string>("base_frame_id", "base_link", "The base frame of the robot");
  declare_default_parameter<std::string>("mode", "perfect_odom", "perfect_odom");
  declare_default_parameter<double>("offest_x", 0.0, "The x offset between the origin of the simulator coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offest_y", 0.0, "The y offset between the origin of the simulator coordinate frame and the map coordinate frame published by fake_localization.");
  declare_default_parameter<double>("offest_yaw", 0.0, "The yaw offset between the origin of the simulator coordinate frame and the map coordinate frame published by fake_localization.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  std::string mode;
  this->get_parameter<std::string>("mode", mode);

  if (mode.compare("perfect_odom") == 0){
    mode_ = PERFECT_ODOM;
    RCLCPP_INFO(get_logger(), "mode is set on: perfect_odom");
  } else if (mode.compare("ground_truth_odom") == 0) {
    mode_ = GROUND_TRUTH_ODOM;
    RCLCPP_INFO(get_logger(), "mode is set on: ground_truth_odom");
  }else if (mode.compare("ground_truth_odom") == 0) {
    mode_ = NA;
    RCLCPP_INFO(get_logger(), "mode not defined!");
  }


  this->get_parameter<std::string>("global_frame_id", global_frame_id_);
  RCLCPP_INFO(get_logger(), "global_frame_id is set on: [%s]", global_frame_id_.c_str());
  this->get_parameter<std::string>("odom_frame_id", odom_frame_id_);
  RCLCPP_INFO(get_logger(), "odom_frame_id is set on: [%s]", odom_frame_id_.c_str());
  this->get_parameter<std::string>("base_frame_id", base_frame_id_);
  RCLCPP_INFO(get_logger(), "base_frame_id is set on: [%s]", base_frame_id_.c_str());

  double offest_x, offest_y, offest_yaw;
  this->get_parameter<double>("offest_x", offest_x);
  this->get_parameter<double>("offest_y", offest_y);
  this->get_parameter<double>("offest_yaw", offest_yaw);
  RCLCPP_INFO(get_logger(), "The offest [x, y, yaw] is set on: [%f m, %f m, %f rad]", offest_x, offest_y, offest_yaw);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, -offest_yaw);
  offset_tf_ = tf2::Transform(q, tf2::Vector3(-offest_x, -offest_y, 0.0));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_activate(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_activate(state);

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("base_pose_ground_truth", 10, std::bind(&FakeLocalization::callback_odom, this, _1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_deactivate(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_deactivate(state);

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".
  sub_odom_.reset();
  tf_broadcaster_.reset();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FakeLocalization::on_shutdown(const rclcpp_lifecycle::State &state)
{
  sub_odom_.reset();
  tf_broadcaster_.reset();

  RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void FakeLocalization::callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg_odom)
{
  RCLCPP_INFO(this->get_logger(), "callback_odom");
  auto stamp = tf2_ros::fromMsg(msg_odom->header.stamp);
  tf2::TimePoint transform_expiration = stamp;
  sendMapToOdomTransform(transform_expiration);
}

void FakeLocalization::sendMapToOdomTransform(const tf2::TimePoint &transform_expiration)
{
  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  tmp_tf_stamped.header.frame_id = global_frame_id_;
  tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
  tmp_tf_stamped.child_frame_id = odom_frame_id_;
  tf_broadcaster_->sendTransform(tmp_tf_stamped);
}
