
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/string_utils.hpp"

#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;
class FakeLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:

  explicit FakeLocalization(const std::string &node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(node_name,
                                        rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {

    declare_default_parameter<std::string>("global_frame_id", "map", "map frame");
    declare_default_parameter<std::string>("odom_frame_id", "fake_odom", "odom frame");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {

    this->get_parameter<std::string>("global_frame_id", global_frame_id_);
    this->get_parameter<std::string>("odom_frame_id", odom_frame_id_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
 
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_activate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&FakeLocalization::callback_odom, this, _1));


    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_deactivate(state);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
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
  on_shutdown(const rclcpp_lifecycle::State &state)
  {
    sub_odom_.reset();
    tf_broadcaster_.reset();

    RCUTILS_LOG_INFO_NAMED(
        get_name(),
        "on shutdown is called from state %s.",
        state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

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

  void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg_odom)
  {
    RCLCPP_INFO(this->get_logger(), "callback_odom");
    auto stamp = tf2_ros::fromMsg(msg_odom->header.stamp);
    tf2::TimePoint transform_expiration = stamp;
    sendMapToOdomTransform(transform_expiration);
  }

  void sendMapToOdomTransform(const tf2::TimePoint &transform_expiration)
  {

    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = global_frame_id_;
    tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
    tmp_tf_stamped.child_frame_id = odom_frame_id_;
    tf_broadcaster_->sendTransform(tmp_tf_stamped);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string global_frame_id_;
  std::string odom_frame_id_;
};

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char *argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<FakeLocalization> node = std::make_shared<FakeLocalization>("localization");

  exe.add_node(node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}