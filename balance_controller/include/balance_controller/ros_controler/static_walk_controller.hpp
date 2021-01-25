#pragma once
#include "controller_interface/controller.h"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"
#include "controller_manager/controller_manager.h"
#include "pluginlib/class_list_macros.hpp"

#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "quadruped_model/quadrupedkinematics.h"

#include "free_gait_core/free_gait_core.hpp"
#include "state_switcher/StateSwitcher.hpp"
#include "balance_controller/motion_control/VirtualModelController.hpp"
#include "balance_controller/contact_force_distribution/ContactForceDistribution.hpp"
#include "std_msgs/Int8MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64.h"

#include "kindr_ros/kindr_ros.hpp"
#include "sim_assiants/FootContacts.h"

#include "quadruped_model_CppAD/Quadruped_optimization.h"

#include "std_srvs/Empty.h"
#include "free_gait_msgs/optimize.h"

#include "actionlib/client/simple_action_client.h"
#include "free_gait_msgs/ExecuteStepsAction.h"

namespace balance_controller{
class static_walk_controller: public controller_interface::Controller<hardware_interface::RobotStateInterface>
{

    typedef std::unordered_map<free_gait::LimbEnum, std::unique_ptr<StateSwitcher>, EnumClassHash> LimbState;
    typedef std::unordered_map<free_gait::LimbEnum, ros::Time, EnumClassHash> LimbDuration;
    typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
    typedef std::unordered_map<free_gait::LimbEnum, double, EnumClassHash> LimbPhase;
    typedef std::unordered_map<free_gait::LimbEnum, free_gait::Vector, EnumClassHash> LimbVector;

public:
    static_walk_controller();
    ~static_walk_controller();
    bool init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle& nodehandle);
    void update(const ros::Time& time, const ros::Duration& period);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time);
    bool optimization_solve(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& res);
    double computeTorqueFromPositionCommand(const double& command, int i, const ros::Duration& period);
    void enforceJointLimits(double& command, unsigned int index);

private:
    void baseCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msgs);
    bool logDataCapture(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res);
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    hardware_interface::RobotStateHandle robot_state_handle_;
    unsigned int n_joints_;

    int log_length_;
    bool log_data_;
    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;

    std::shared_ptr<free_gait::State> robot_state_;
    std::vector<control_toolbox::Pid> pid_controllers_;

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    std::shared_ptr<ContactForceDistribution> contact_distribution_;
    std::shared_ptr<VirtualModelController> virtual_model_controller_;

    Position base_desired_position_;
    RotationQuaternion base_desired_rotation_;
    LinearVelocity base_desired_linear_velocity_;
    LocalAngularVelocity base_desired_angular_velocity_;

    std::vector<std_msgs::Int8MultiArray> motor_status_word_;

    //data log
    std::vector<geometry_msgs::WrenchStamped> virtual_force_torque_, desired_virtual_force_torque_;
    std::vector<nav_msgs::Odometry> base_command_pose_, base_actual_pose_;
    std::vector<sensor_msgs::JointState> joint_command_, joint_actual_;
    LimbVector surface_normals_;
    std::vector<sim_assiants::FootContacts> foot_desired_contact_;
    LimbVector foot_positions, foot_velocities, foot_accelerations, real_contact_force_, stored_foot_positions;
    std::vector<free_gait_msgs::RobotState> desired_robot_state_, actual_robot_state_;

    std::vector<double> torques_square_;
    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer;
    ros::Subscriber base_command_sub_;
    ros::ServiceServer log_data_srv_;

    ros::Publisher joint_command_pub_, base_command_pub_, base_actual_pub_, joint_actual_pub_,
        desired_robot_state_pub_, actual_robot_state_pub_, torques_square_pub_;


    LimbFlag real_contact_, is_cartisian_motion_, is_footstep_, is_legmode_;


    //optimize service
    boost::recursive_mutex r_mutex_;
    ros::ServiceServer optimize_srv_;
    ros::ServiceClient client_cli_;

    //action
    std::string actionServerTopic_;
    free_gait_msgs::ExecuteStepsGoal step_goal;
    std::unique_ptr<actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>> stepActionClient_;
};

}//namespace
