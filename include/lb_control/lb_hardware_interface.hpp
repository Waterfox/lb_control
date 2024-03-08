#ifndef LB_CONTROL__LLB_HARDWARE_INTERFACE_HPP
#define LB_CONTROL__LLB_HARDWARE_INTERFACE_HPP

// #include <hardware_interface/hardware_interface.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <mutex>


// class LBHardwareInterface : public hardware_interface::HardwareInterface
class LBHardwareInterface: public rclcpp::Node
{
public:
explicit LBHardwareInterface(std::string node_name);

sensor_msgs::msg::JointState get_mcu_feedback();
sensor_msgs::msg::JointState mcu_feedback_;
std::mutex mcu_feedback_mutex_;
sensor_msgs::msg::JointState drive_msg;
sensor_msgs::msg::JointState msg;
void mcu_feedback_callback(sensor_msgs::msg::JointState::SharedPtr fb_msg);
// void mcu_feedback_callback(sensor_msgs::msg::JointState &);
void drive_command(std::vector<double> );


private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr drive_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mcu_feedback_sub_;

};

#endif // MY_ROBOT_CONTROL_LB_HARDWARE_INTERFACE_HPP