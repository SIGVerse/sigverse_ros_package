#ifndef SIGVERSE_TURTLEBOT3_MANIPULATION_HARDWARE__SIGVERSE_TURTLEBOT3_SYSTEM_HPP_
#define SIGVERSE_TURTLEBOT3_MANIPULATION_HARDWARE__SIGVERSE_TURTLEBOT3_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace sigverse
{
class SIGVerseTurtleBot3System : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  const double kJointCommandDelta = 0.001;  // 0.001 rad = 0.057 degrees
  // const double kJointUpdateInterval = 0.1;  // sec

  bool HasJointCommandChanged() const noexcept;

  std::unordered_map<std::string, double> joint_position_map_; 
  std::unordered_map<std::string, double> joint_position_command_map_;
  std::unordered_map<std::string, double> last_sent_joint_pos_cmd_map_;

  realtime_tools::RealtimeBuffer<std::unordered_map<std::string, double>> joint_states_buffer_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>> realtime_joint_trajectory_pub_;
  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>> joint_trajectory_pub_;

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> thread_executor_;
  std::thread spin_thread_;

  rclcpp::Time last_sent_joint_pos_time_{0,0};

  double joint_trajectory_pub_interval_;
};
}  // namespace sigverse
#endif  // SIGVERSE_TURTLEBOT3_MANIPULATION_HARDWARE__SIGVERSE_TURTLEBOT3_SYSTEM_HPP_
