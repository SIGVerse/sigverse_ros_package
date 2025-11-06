#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sigverse_turtlebot3_manipulation_hardware/sigverse_turtlebot3_system.hpp"

namespace sigverse
{
auto logger = rclcpp::get_logger("sigverse_turtlebot3_system");

hardware_interface::CallbackReturn
SIGVerseTurtleBot3System::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(logger, "TB3 hardware_interface on_init -Start-");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto &joint : info_.joints) 
  {
    joint_position_map_        [joint.name] = 0.0;
    joint_position_command_map_[joint.name] = 0.0;
    for (const auto &state_interface : joint.state_interfaces) 
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION && !state_interface.initial_value.empty()) 
      {
        joint_position_map_        [joint.name] = std::stod(state_interface.initial_value);
        joint_position_command_map_[joint.name] = std::stod(state_interface.initial_value);
      }
    }
  }

  joint_states_buffer_.writeFromNonRT(joint_position_map_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SIGVerseTurtleBot3System::export_state_interfaces()
{
  RCLCPP_INFO(logger, "TB3 hardware_interface export_state_interfaces -Start-");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (const auto &joint : info_.joints) 
  {
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint_position_map_.at(joint.name));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SIGVerseTurtleBot3System::export_command_interfaces()
{
  RCLCPP_INFO(logger, "TB3 hardware_interface export_command_interfaces -Start-");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (const auto &joint : info_.joints) 
  {
    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint_position_command_map_.at(joint.name));
  }

//  command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[6].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[0]));
//  command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[7].name, hardware_interface::HW_IF_POSITION, &dxl_gripper_commands_[1]));

  return command_interfaces;
}

hardware_interface::CallbackReturn 
SIGVerseTurtleBot3System::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "TB3 hardware_interface on_activate -Start-");

  node_ = std::make_shared<rclcpp::Node>("sigverse_tb3_hw_bridge");
  thread_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  thread_executor_->add_node(node_);

  joint_trajectory_pub_interval_ = std::stod(info_.hardware_parameters.at("joint_trajectory_pub_interval"));
  last_sent_joint_pos_time_ = node_->get_clock()->now();
  
  // Subscribe JointStates
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>
  (
    info_.hardware_parameters["joint_state_topic"], rclcpp::QoS(10).best_effort(),
    [this](const sensor_msgs::msg::JointState &msg)
    {
      if (msg.name.size() != msg.position.size()) { return; } // safety

      // Non-RT context: build a snapshot map and push to RealtimeBuffer
      auto snapshot = joint_position_map_;

      for (size_t i = 0; i < msg.name.size(); i++) 
      {
        if (snapshot.count(msg.name[i]) > 0)
        {
          snapshot[msg.name[i]] = msg.position[i];
        }
      }
      joint_states_buffer_.writeFromNonRT(snapshot);
    }
  );

  // Publisher for single-point JointTrajectory toward simulator
  joint_trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(info_.hardware_parameters["joint_trajectory_topic"], rclcpp::SystemDefaultsQoS());
  realtime_joint_trajectory_pub_ = std::make_unique<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>>(joint_trajectory_pub_);

  spin_thread_ = std::thread([this]{ thread_executor_->spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
SIGVerseTurtleBot3System::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "TB3 hardware_interface on_deactivate -Start-");

  if (thread_executor_) thread_executor_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  joint_state_sub_.reset();
  thread_executor_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type 
SIGVerseTurtleBot3System::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "TB3 hardware_interface read -Start-");

  if (const auto* latest = joint_states_buffer_.readFromRT()) 
  {
    for (auto& [name, position] : joint_position_map_) 
    {
      if (latest->count(name) > 0) 
      {
        position = latest->at(name);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type 
SIGVerseTurtleBot3System::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "TB3 hardware_interface write -Start-");

  if (joint_position_command_map_.empty()) { return hardware_interface::return_type::OK; }

  if (!has_joint_command_changed()) return hardware_interface::return_type::OK;

  // build single-point JointTrajectory
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = node_->get_clock()->now();

  trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory_point;
  joint_trajectory.joint_names    .reserve(joint_position_command_map_.size());
  joint_trajectory_point.positions.reserve(joint_position_command_map_.size());

  for (const auto &key_value : joint_position_command_map_) 
  {
    joint_trajectory.joint_names    .push_back(key_value.first);
    joint_trajectory_point.positions.push_back(key_value.second);
  }

  // Single immediate point
  joint_trajectory_point.time_from_start = rclcpp::Duration(0,0);
  joint_trajectory.points.emplace_back(std::move(joint_trajectory_point));

  realtime_joint_trajectory_pub_->lock();
  realtime_joint_trajectory_pub_->msg_ = joint_trajectory;
  realtime_joint_trajectory_pub_->unlockAndPublish();

  last_sent_joint_pos_cmd_map_ = joint_position_command_map_;
  last_sent_joint_pos_time_ = node_->get_clock()->now();

  return hardware_interface::return_type::OK;
}

bool SIGVerseTurtleBot3System::has_joint_command_changed() const noexcept
{
  rclcpp::Time now = node_->get_clock()->now();

  if ((now - last_sent_joint_pos_time_) < rclcpp::Duration::from_seconds(joint_trajectory_pub_interval_)) 
  {
    return false;
  }

  if (joint_position_command_map_.size() != last_sent_joint_pos_cmd_map_.size()) { return true; }

  for (const auto &key_value : joint_position_command_map_) 
  {
    auto it = last_sent_joint_pos_cmd_map_.find(key_value.first);

    if (it == last_sent_joint_pos_cmd_map_.end()) 
    {
      return true; // missing key
    }
    if (std::abs(key_value.second - it->second) > kJointCommandDelta) 
    {
      return true; // value changed
    }
  }
  return false; // no change
}

}  // namespace sigverse

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sigverse::SIGVerseTurtleBot3System,
  hardware_interface::SystemInterface)
