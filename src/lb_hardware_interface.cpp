#include "lb_control/lb_hardware_interface.hpp"
#include <vector>


LBHardwareInterface::LBHardwareInterface(std::string node_name="LB_hardware_interface"): Node(node_name) 
{
    // Initialize empty joint state message
  int num_joints_ = 4;
  mcu_feedback_.name.resize(num_joints_, "wheelX_joint");
  mcu_feedback_.position.resize(num_joints_, 0.0);
  mcu_feedback_.velocity.resize(num_joints_, 0.0);

  // drive_msg.name.resize(num_joints_, "wheelX_joint");  // NO MEMORY ALLOCATED IN MCU
  // drive_msg.position.resize(num_joints_, 0.0);
  drive_msg.velocity.resize(num_joints_, 0.0);
  // drive_msg.name = {"wheel1_joint", "wheel2_joint", "wheel3_joint", "wheel4_joint"};

  msg.name.resize(num_joints_, "wheelX_joint");
  msg.position.resize(num_joints_, 0.0);
  msg.velocity.resize(num_joints_, 0.0);
  msg.name = {"wheel1_joint", "wheel2_joint", "wheel3_joint", "wheel4_joint"};

  
  // for (auto i = 0u; i<4; i++){  //hardcoded number of joints for now
  //   mcu_feedback_.name.push_back("wheelX_joint");
  //   mcu_feedback_.velocity.push_back((double) 0.0);
  //   mcu_feedback_.position.push_back((double) 0.0);
  // }

  // for (auto i = 0u; i<4; i++){  //hardcoded number of joints for now
  //   drive_msg.name.push_back("wheelX_joint");
  //   drive_msg.velocity.push_back((double) 0.0);
  //   drive_msg.position.push_back((double) 0.0);
  // }
  // drive_msg.name = {"wheel1_joint", "wheel2_joint", "wheel3_joint", "wheel4_joint"};

  // for (auto i = 0u; i<4; i++){  //hardcoded number of joints for now
  //   msg.name.push_back("wheelX_joint");
  //   msg.velocity.push_back((double) 0.0);
  //   msg.position.push_back((double) 0.0);
  // }

  
  mcu_feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "enc_feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&LBHardwareInterface::mcu_feedback_callback, this, std::placeholders::_1));

  drive_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "mcu_cmd_motor",
    10);
    // rclcpp::SensorDataQoS());
}


sensor_msgs::msg::JointState LBHardwareInterface::get_mcu_feedback() 
{
  //CPR declares new message, guards 
  // sensor_msgs::msg::JointState msg;
  {
    std::lock_guard<std::mutex> guard(mcu_feedback_mutex_);
    msg = mcu_feedback_;
  }
  return msg;
}
    

void LBHardwareInterface::mcu_feedback_callback(sensor_msgs::msg::JointState::SharedPtr fb_msg)
// void LBHardwareInterface::mcu_feedback_callback(sensor_msgs::msg::JointState & fb_msg)
{
  std::lock_guard<std::mutex> guard(mcu_feedback_mutex_);
  mcu_feedback_ = *fb_msg;

}

void LBHardwareInterface::drive_command(std::vector<double> hw_commands_)
{
  // sensor_msgs::msg::JointState drive_msg;
  //TODO, bring in joint names from ROS control - not required by MCU

  for (auto i = 0u; i<hw_commands_.size(); i++){
    drive_msg.velocity[i] = hw_commands_[i];
  }

  drive_pub_->publish(drive_msg);
}
