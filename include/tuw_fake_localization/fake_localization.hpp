
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <bondcpp/bond.hpp>
#include <bond/msg/constants.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;
class FakeLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FakeLocalization(const std::string &node_name, bool intra_process_comms = false);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &state);

private:
  template <typename T>
  void declare_default_parameter(
      const std::string &name,
      const T &value_default,
      const std::string &description)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = description;
    this->declare_parameter<T>(name, value_default, descriptor);
  }

  std::unique_ptr<bond::Bond> bond_{nullptr};
  void create_bond();

  void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg_odom);
  void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg_odom);

  void sendWorldToMapTransform(const tf2::TimePoint &transform_expiration);
  void sendMapToOdomTransform(const tf2::TimePoint &transform_expiration);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  std::string world_frame_id_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  tf2::Transform tf_world_base_;
  tf2::Transform tf_world_map_;
  tf2::Transform tf_map_odom_;
  tf2::Transform tf_odom_base_;
  geometry_msgs::msg::TransformStamped tf_stamped_odom_;
  geometry_msgs::msg::TransformStamped tf_stamped_ground_truth_;
};
