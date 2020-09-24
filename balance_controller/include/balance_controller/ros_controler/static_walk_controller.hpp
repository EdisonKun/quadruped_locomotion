#pragma once
#include "controller_interface/controller.h"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"
#include "controller_manager/controller_manager.h"
#include "pluginlib/class_list_macros.hpp"

#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "quadruped_model/quadrupedkinematics.h"

namespace balance_controller{
class static_walk_controller: public controller_interface::Controller<hardware_interface::RobotStateInterface>
{


public:
    static_walk_controller();
    ~static_walk_controller();
    bool init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle& nodehandle);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);

private:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::RobotStateHandle robot_state_handle_;
    unsigned int n_joints_;



};

}//namespace
