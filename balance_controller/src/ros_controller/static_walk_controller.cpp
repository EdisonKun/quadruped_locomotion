#include "balance_controller/ros_controler/static_walk_controller.hpp"

#include "quadruped_model/quadrupedkinematics.h"
#include "ros/ros.h"

namespace balance_controller{
static_walk_controller::static_walk_controller()
{

}//static_walk_controller

static_walk_controller::~static_walk_controller()
{

}

bool static_walk_controller::init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle &nodehandle)
{
    ROS_INFO("Initializing static walk controller");

    urdf::Model urdf;
    if (!urdf.initParam("/robot_description"))
    {
        ROS_ERROR("failed to prase urdf file");
        return false;
    }

    std::string param_name = "joints";
    if(!nodehandle.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nodehandle.getNamespace() << ").");
        return false;
    }

    n_joints_ = joint_names_.size();
    if(n_joints_ == 0 )
    {
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }

    for (unsigned int i = 0; i < n_joints_; i++) {
        try {
            joints_.push_back(hardware->joint_position_interfaces_.getHandle(joint_names_[i]));
            ROS_INFO("Get %s Handle", joint_names_[i].c_str());
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("Exception Thrown :" << ex.what());
            return false;
        }

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return false;
        }
    }
    robot_state_handle_ = hardware->getHandle("base_controller");
    ROS_INFO("static walk controller is to be initialized");
    for (unsigned int i = 0; i < 12; i++) {
        robot_state_handle_.getJointEffortWrite()[i] = 0;
        robot_state_handle_.motor_status_word_[i] = 0;
    }
    for (unsigned int i = 0; i < 4; i++) {
        robot_state_handle_.foot_contact_[i] = 1;
    }

    return true;
}

void static_walk_controller::update(const ros::Time &time, const ros::Duration &period)
{


}

void static_walk_controller::starting(const ros::Time &time)
{

}

void static_walk_controller::stopping(const ros::Time &time)
{

}



}//namespace
PLUGINLIB_EXPORT_CLASS(balance_controller::static_walk_controller, controller_interface::ControllerBase)
