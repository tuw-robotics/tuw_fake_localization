#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tuw_fake_localization/fake_localization.hpp"

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  std::shared_ptr<FakeLocalization> node = std::make_shared<FakeLocalization>("localization");
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();

  return 0;
}