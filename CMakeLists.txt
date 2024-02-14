cmake_minimum_required(VERSION 3.8)
project(tuw_fake_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)

include_directories(include)
add_library(fake_localization src/fake_localization.cpp)
target_include_directories(fake_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fake_localization PUBLIC c_std_99 cxx_std_17) 
ament_target_dependencies(
  fake_localization
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "bondcpp"
  "bondcpp"
)


add_executable(fake_localization_node  src/standalone_fake_localization.cpp)
target_include_directories(fake_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fake_localization_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(fake_localization_node fake_localization)
ament_target_dependencies(
  fake_localization_node
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
)

install(TARGETS fake_localization_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
