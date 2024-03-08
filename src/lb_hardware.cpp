#include "lb_control/lb_hardware.hpp"




hardware_interface::CallbackReturn LBHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
      return hardware_interface::CallbackReturn::ERROR;
  }


  hardware_interface::CallbackReturn ret;
  // Get Hardware name and joints
  ret = getHardwareInfo(info);

  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
      return ret;
  }

  
  read_msg.name.resize(num_joints_, "wheelX_joint");
  read_msg.position.resize(num_joints_, 0.0);
  read_msg.velocity.resize(num_joints_, 0.0);

  //initialize memory for the read_msg used in read fn
  // for (auto i = 0u; i<10; i++){  //hardcoded number of joints for now
  //   read_msg.name.push_back("wheelX_joint");
  //   read_msg.velocity.push_back((double) 0.0);
  //   read_msg.position.push_back((double) 0.0);
  // }

  // Initialize hardware interface
  return initHardwareInterface();
}


hardware_interface::CallbackReturn LBHardware::initHardwareInterface()
{
  node_ = std::make_shared<LBHardwareInterface>("LB_hardware_interface");

  if (node_ == nullptr)
  {
      return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Initialized Hardware Interface Node");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LBHardware::getHardwareInfo(const hardware_interface::HardwareInfo & info)
{
  // Get info from URDF
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_name_ = info_.name;
  num_joints_ = info_.joints.size();

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Name: %s", hw_name_.c_str());

  // Check for valid number of joints (littlebot has 4 joints)
  if (num_joints_ != 4)
  {
    RCLCPP_ERROR(rclcpp::get_logger(hw_name_), "Invalid number of joints %u", num_joints_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Number of Joints %u", num_joints_);

  hw_states_position_.resize(num_joints_);
  hw_states_position_offset_.resize(num_joints_);
  hw_states_velocity_.resize(num_joints_);
  hw_commands_.resize(num_joints_);

  //Probably not needed here
  // for (auto i = 0u; i<4;i++){
  //   hw_states_position_.push_back(0.0);
  //   hw_states_position_offset_.push_back(0.0);
  //   hw_states_velocity_.push_back(0.0);
  //   hw_commands_.push_back(0.0);
  // }

  // RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Number of Joints check %u", hw_states_position_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LBHardware::export_state_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Export States");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < num_joints_; i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LBHardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Import Commands");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < num_joints_; i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn LBHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (std::isnan(hw_states_position_[i])) {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System Successfully started!");
    RCLCPP_INFO(
    rclcpp::get_logger(hw_name_),
    "activated with positions (RF, W1: %f, RR, W2: %f, LF, W3: %f, LR, W4 %f)",
    hw_states_position_[0], hw_states_position_[1], hw_states_position_[2], hw_states_position_[3]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LBHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "Stopping ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger(hw_name_), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type LBHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Reading from hardware");


  // Update joints from the hardware interface
  rclcpp::spin_some(node_);
  read_msg = node_->get_mcu_feedback();  
  
  RCLCPP_DEBUG(
    rclcpp::get_logger(hw_name_),
    "Received LINEAR distance information (RF: %f, RR: %f, LF: %f, LRL %f)",
    read_msg.position[0], read_msg.position[1], read_msg.position[2], read_msg.position[3]);

    
  //TODO the MCU should output Linear velocities

  // for (auto i = 0u; i < hw_states_position_.size(); i++) {
  for (auto i = 0u; i < 4; i++) {
    //Positions
    // delta = read_msg.position[i]; 
    //The mcu is reporting deltas. 
    //This may be a problem if each publish is not consumed
    hw_states_position_[i] += read_msg.position[i]; 

    //Velocities
    //TODO The MCU is not yet populating LINEAR velocities
    // hw_states_velocity_[i] = read_msg.velocity[i];
  }
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type LBHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Writing to hardware");

  //writeCommandsToHardware();
  node_->drive_command(hw_commands_);


  RCLCPP_DEBUG(rclcpp::get_logger(hw_name_), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  LBHardware, hardware_interface::SystemInterface)