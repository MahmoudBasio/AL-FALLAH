#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>("trimming_executor", options);

  // Keep the executor alive while MoveIt waits for robot state/action responses.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  move_group.startStateMonitor(5.0);

  RCLCPP_INFO(node->get_logger(), "End effector: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(), "Waiting for robot state...");
  rclcpp::sleep_for(2s);

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

  // Phase 1 target geometry used by tree_model/path_generator.
  constexpr double tree_center_x = 0.90;
  constexpr double tree_center_y = 0.00;
  constexpr double tree_center_z = 1.00;
  constexpr double box_width = 0.55;
  constexpr double box_depth = 0.55;
  constexpr double box_height = 1.00;
  constexpr int rows = 8;

  const double x = tree_center_x - box_width / 2.0;
  const double y_min = tree_center_y - box_depth / 2.0;
  const double y_max = tree_center_y + box_depth / 2.0;
  const double z_min = tree_center_z - box_height / 2.0;
  const double z_max = tree_center_z + box_height / 2.0;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(rows * 2);

  // Preserve the current end-effector orientation and raster the front face.
  for (int i = 0; i < rows; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(rows - 1);
    const double z = z_min + t * (z_max - z_min);

    const double first_y = (i % 2 == 0) ? y_min : y_max;
    const double second_y = (i % 2 == 0) ? y_max : y_min;

    auto p1 = current_pose;
    p1.position.x = x;
    p1.position.y = first_y;
    p1.position.z = z;
    waypoints.push_back(p1);

    auto p2 = current_pose;
    p2.position.x = x;
    p2.position.y = second_y;
    p2.position.z = z;
    waypoints.push_back(p2);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(
    waypoints,
    0.02,  // eef_step
    0.0,   // jump_threshold
    trajectory);

  RCLCPP_INFO(
    node->get_logger(),
    "Cartesian path achieved: %.1f%%",
    fraction * 100.0);

  // The Phase 1 TIAGo simulation reached approximately 78.9% from the initial base pose.
  // Phase 2 addresses the remaining path with base repositioning.
  if (fraction > 0.70) {
    RCLCPP_INFO(node->get_logger(), "Executing trimming path...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    const auto result = move_group.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Trimming trajectory execution succeeded.");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Trimming trajectory execution failed.");
    }
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "Reachable fraction is below the Phase 1 execution threshold (70%%)."
    );
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return 0;
}
