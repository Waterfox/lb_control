#ifndef LB_CONTROL__LB_HARDWARE_HPP_
#define LB_CONTROL__LB_HARDWARE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lb_control/lb_hardware_interface.hpp"


class LBHardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(LBHardware)


        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;


        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;


        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;


        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;


        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;


        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;



    protected:
        void writeCommandsToHardware();  //MAYBE
        void updateJointsFromHardware();  // MAYBE
        virtual hardware_interface::CallbackReturn getHardwareInfo(const hardware_interface::HardwareInfo & info);
        // virtual hardware_interface::CallbackReturn validateJoints();
        virtual hardware_interface::CallbackReturn initHardwareInterface();
        std::shared_ptr<LBHardwareInterface> node_;
        
        

        uint8_t num_joints_;
        std::map<std::string, uint8_t> wheel_joints_;
        std::string hw_name_;

        sensor_msgs::msg::JointState read_msg;
        // Store the command for the robot
        std::vector<double> hw_commands_;
        std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

        // sensor_msgs::msg::JointState msg;
        double delta;
};

#endif //LB_CONTROL__LB_HARDWARE_HPP_