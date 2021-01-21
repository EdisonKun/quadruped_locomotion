#include "balance_controller/ros_controler/static_walk_controller.hpp"

#include "ros/ros.h"
#include "thread"

namespace balance_controller{
static_walk_controller::static_walk_controller()
{
    log_length_ = 10000;

    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);

    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);

    robot_state_.reset(new free_gait::State);
    robot_state_->initialize(limbs_, branches_);

    for (auto limb : limbs_) {
        surface_normals_[limb] = Vector(0,0,1);
        is_cartisian_motion_[limb] = false;
        is_footstep_[limb] = false;
        is_legmode_[limb] = false;
    }
}//static_walk_controller

static_walk_controller::~static_walk_controller()
{

}//~static_walk_controller

bool static_walk_controller::init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle &nodehandle)
{
    ROS_INFO("Initializing static walk controller");

    contact_distribution_.reset(new ContactForceDistribution(nodehandle, robot_state_));
    virtual_model_controller_.reset(new VirtualModelController(nodehandle, robot_state_, contact_distribution_));

    urdf::Model urdf;
    if (!urdf.initParam("/robot_description"))
    {
        ROS_ERROR("failed to prase urdf file");
        return false;
    }

    if(!contact_distribution_->loadParameters())
    {
        ROS_INFO("CFD load parameters failed");
    }
    if(!virtual_model_controller_->loadParameters())
    {
        ROS_INFO("VMC load parameters failed");
    }

    std::string param_name = "joints";
    if(!nodehandle.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << nodehandle.getNamespace() << ").");
        return false;
    }
    if(!nodehandle.getParam("log_data", log_data_))
    {
        ROS_ERROR("Can't find parameter of 'log_data'");
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
            joints_.push_back(hardware->joint_effort_interfaces_.getHandle(joint_names_[i]));
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
        joint_urdfs_.push_back(joint_urdf);
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


    commands_buffer.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    optimize_srv_ = nodehandle.advertiseService("/optimize_solve", &static_walk_controller::optimization_solve, this);
    client_cli_ = nodehandle.serviceClient<free_gait_msgs::optimize>("/optimize");

    base_command_sub_ = nodehandle.subscribe<free_gait_msgs::RobotState>("/desired_robot_state", 1, &static_walk_controller::baseCommandCallback, this);

    return true;
}

void static_walk_controller::update(const ros::Time &time, const ros::Duration &period)
{
    ROS_INFO_STREAM_ONCE("running the static walk controller");
    sensor_msgs::JointState joint_command, joint_actual;
    joint_command.effort.resize(12);
    joint_command.position.resize(12);
    joint_command.name.resize(12);
    joint_actual.name.resize(12);
    joint_actual.position.resize(12);
    joint_actual.velocity.resize(12);
    joint_actual.effort.resize(12);

    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;

    std_msgs::Int8MultiArray status_word;
    status_word.data.resize(12);

    for (unsigned int i = 0; i < 12; i++) {
        all_joint_positions(i) = robot_state_handle_.getJointPositionRead()[i];
        all_joint_velocities(i) = robot_state_handle_.getJointVelocityRead()[i];
        all_joint_efforts(i) = robot_state_handle_.getJointEffortRead()[i];
        joint_actual.position[i] = all_joint_positions(i);
        joint_actual.velocity[i] = all_joint_velocities(i);
        joint_actual.effort[i] = all_joint_efforts(i);
        status_word.data[i] = robot_state_handle_.motor_status_word_[i];
    }
    std::vector<double> & commands = *commands_buffer.readFromRT();

    RotationQuaternion base_orientation = RotationQuaternion(robot_state_handle_.getOrientation()[0],
                                                             robot_state_handle_.getOrientation()[1],
                                                             robot_state_handle_.getOrientation()[2],
                                                             robot_state_handle_.getOrientation()[3]);

    for (unsigned int i = 0; i < 4; i++) {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        robot_state_->setSupportLeg(limb, true);
        robot_state_->setSurfaceNormal(limb,Vector(0,0,1));
    }

    //set desired position to robot_state;
    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position_);
    robot_state_->setOrientationBaseToWorld(base_desired_rotation_);
    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity_);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity_);

    robot_state_->setCurrentLimbJoints(all_joint_positions);
    robot_state_->setCurrentLimbJointVelocities(all_joint_velocities);

    //set current position to robot_state;
    Pose current_base_pose = Pose(Position(robot_state_handle_.getPosition()[0],
                                           robot_state_handle_.getPosition()[1],
                                           robot_state_handle_.getPosition()[2]),
                                  RotationQuaternion(robot_state_handle_.getOrientation()[0],
                                                     robot_state_handle_.getOrientation()[1],
                                                     robot_state_handle_.getOrientation()[2],
                                                     robot_state_handle_.getOrientation()[3]));

    robot_state_->setPoseBaseToWorld(current_base_pose);
//    std::cout << "the actual position is " << current_base_pose.getPosition() << std::endl;
    robot_state_->setBaseStateFromFeedback(LinearVelocity(robot_state_handle_.getLinearVelocity()[0],
                                                         robot_state_handle_.getLinearVelocity()[1],
                                                         robot_state_handle_.getLinearVelocity()[2]),
                                          LocalAngularVelocity(robot_state_handle_.getAngularVelocity()[0],
                                                               robot_state_handle_.getAngularVelocity()[1],
                                                               robot_state_handle_.getAngularVelocity()[2]));
    if(!virtual_model_controller_->compute())
    {
        ROS_ERROR("VMC compute failed");
        ROS_WARN_STREAM(virtual_model_controller_);
    }


    for (int i = 0; i < 4; i++) {
        free_gait::JointEffortsLeg joint_torque_limb = robot_state_->getJointEffortsForLimb(static_cast<free_gait::LimbEnum>(i));

        int start_index = i * 3;
        for (int j = 0; j < 3; j++) {
            double joint_torque_command = joint_torque_limb(j);
            int index = start_index + j;

            robot_state_handle_.getJointEffortWrite()[index] = joint_torque_command;
            robot_state_handle_.mode_of_joint_[i] = 4;// joint mode,profile or others
            joint_command.name[index] = joint_names_[index];
            joint_command.effort[index] = joint_torque_command;
            joint_command.position[index] = commands[index];
            joint_actual.name[index] = joint_names_[index];

        }
    }

    for (unsigned int i = 0; i < 4; i++) {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        if(robot_state_->isSupportLeg(limb))
        {
//            std::cout << " the leg " << i << " is support leg" << std::endl;
            joints_[3 * i + 0].setCommand(joint_command.effort[3 * i]);
            joints_[3 * i + 1].setCommand(joint_command.effort[3 * i + 1]);
            joints_[3 * i + 2].setCommand(joint_command.effort[3 * i + 2]);
        }
    }


//    std::cout << "log_data_ value is " << log_data_ << std::endl;

    if(log_data_)
    {
        std::cout << "in the log_data " << std::endl;
        motor_status_word_.push_back(status_word);
        geometry_msgs::WrenchStamped vmc_force_torque, desired_vmc_ft;
        vmc_force_torque.header.frame_id = "/base_link";
        vmc_force_torque.header.stamp = ros::Time::now();
        desired_vmc_ft.header.frame_id = "/base_link";
        desired_vmc_ft.header.stamp = ros::Time::now();

        Force net_force;
        Torque net_torque;
        virtual_model_controller_->getDistributedVirtualForceAndTorqueInBaseFrame(net_force, net_torque);//base force and torques
        std::cout << " the distributed virtual force and torque is " << net_force << net_torque << std::endl;
        std::cout << " the desired virtual force and torque is " << virtual_model_controller_->getDesiredVirtualForceInBaseFrame()
            << virtual_model_controller_->getDesiredVirtualTorqueInBaseFrame()<< std::endl;

//        kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getDesiredVirtualForceInBaseFrame().vector()),
//                                           desired_vmc_ft.wrench.force);
//        kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getDesiredVirtualTorqueInBaseFrame().vector()),
//                                           desired_vmc_ft.wrench.torque);
        kindr_ros::convertToRosGeometryMsg(Position(net_force.vector()),
                                           vmc_force_torque.wrench.force);
        kindr_ros::convertToRosGeometryMsg(Position(net_torque.vector()),
                                           vmc_force_torque.wrench.torque);

        virtual_force_torque_.push_back(vmc_force_torque);
        desired_virtual_force_torque_.push_back(desired_vmc_ft);

        free_gait_msgs::RobotState desired_robot_state, actual_robot_state;
        desired_robot_state.lf_target.target_position.resize(1);
        desired_robot_state.lf_target.target_velocity.resize(1);
        desired_robot_state.lf_target.target_force.resize(1);

        actual_robot_state.lf_target.target_position.resize(1);
        actual_robot_state.lf_target.target_velocity.resize(1);
        actual_robot_state.lf_target.target_force.resize(1);

        desired_robot_state.rf_target.target_position.resize(1);
        desired_robot_state.rf_target.target_velocity.resize(1);
        desired_robot_state.rf_target.target_force.resize(1);

        actual_robot_state.rf_target.target_position.resize(1);
        actual_robot_state.rf_target.target_velocity.resize(1);
        actual_robot_state.rf_target.target_force.resize(1);

        desired_robot_state.rh_target.target_position.resize(1);
        desired_robot_state.rh_target.target_velocity.resize(1);
        desired_robot_state.rh_target.target_force.resize(1);

        actual_robot_state.rh_target.target_position.resize(1);
        actual_robot_state.rh_target.target_velocity.resize(1);
        actual_robot_state.rh_target.target_force.resize(1);

        desired_robot_state.lh_target.target_position.resize(1);
        desired_robot_state.lh_target.target_velocity.resize(1);
        desired_robot_state.lh_target.target_force.resize(1);

        actual_robot_state.lh_target.target_position.resize(1);
        actual_robot_state.lh_target.target_velocity.resize(1);
        actual_robot_state.lh_target.target_force.resize(1);

        geometry_msgs::Pose desired_pose, actual_pose;
        geometry_msgs::Twist desired_twist, actual_twist;
        nav_msgs::Odometry desire_odom, actual_odom;

        Pose current_pose_in_base;
        Position base_desired_position_in_base = current_base_pose.getRotation().inverseRotate(base_desired_position_);
        current_pose_in_base.getPosition() = current_base_pose.getRotation().inverseRotate(current_base_pose.getPosition());
        current_pose_in_base.getRotation() = current_base_pose.getRotation();
        LinearVelocity desired_vel_in_base, current_vel_in_base;
        LinearVelocity current_vel_in_world(robot_state_handle_.getLinearVelocity()[0],
                                            robot_state_handle_.getLinearVelocity()[1],
                                            robot_state_handle_.getLinearVelocity()[2]);
        desired_vel_in_base = current_base_pose.getRotation().inverseRotate(base_desired_linear_velocity_);
        current_vel_in_base = current_base_pose.getRotation().inverseRotate(current_vel_in_world);


        kindr_ros::convertToRosGeometryMsg(base_desired_position_in_base, desired_pose.position);
        kindr_ros::convertToRosGeometryMsg(base_desired_rotation_, desired_pose.orientation);
        kindr_ros::convertToRosGeometryMsg(desired_vel_in_base, desired_twist.linear);
        kindr_ros::convertToRosGeometryMsg(base_desired_angular_velocity_, desired_twist.angular);
        kindr_ros::convertToRosGeometryMsg(current_vel_in_base, actual_twist.linear);
        kindr_ros::convertToRosGeometryMsg(current_pose_in_base, actual_pose);

        actual_twist.angular.x = robot_state_handle_.getAngularVelocity()[0];
        actual_twist.angular.y = robot_state_handle_.getAngularVelocity()[1];
        actual_twist.angular.z = robot_state_handle_.getAngularVelocity()[2];
        desire_odom.pose.pose = desired_pose;
        desire_odom.twist.twist = desired_twist;
        actual_odom.pose.pose = actual_pose;
        actual_odom.twist.twist = actual_twist;

        base_actual_pose_.push_back(actual_odom);
        base_command_pose_.push_back(desire_odom);
        joint_command_.push_back(joint_command);
        joint_actual_.push_back(joint_actual);

        std::vector<sensor_msgs::JointState> joint_states_leg, joint_commands_leg;
        joint_states_leg.resize(4);
        joint_commands_leg.resize(4);
        std::vector<free_gait::Force> real_contact_forces;
        real_contact_forces.resize(4);
        sim_assiants::FootContacts desired_contact;
        desired_contact.foot_contacts.resize(4);

        for(int i = 0; i < 4; i++)
        {
            free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
            Force contact_force_in_base = contact_distribution_->getLegInfo(limb).desiredContactForce_;
            Force contact_force_in_world = robot_state_->getOrientationBaseToWorld().rotate(contact_force_in_base);//contact force of base;
            desired_contact.foot_contacts[i].contact_force.wrench.force.x = contact_force_in_base(0);
            desired_contact.foot_contacts[i].contact_force.wrench.force.y = contact_force_in_base(1);
            desired_contact.foot_contacts[i].contact_force.wrench.force.z = contact_force_in_base(2);
            //            ROS_INFO("Log surface normals");
            desired_contact.foot_contacts[i].surface_normal.vector.x = surface_normals_.at(limb)(0);
            desired_contact.foot_contacts[i].surface_normal.vector.y = surface_normals_.at(limb)(1);
            desired_contact.foot_contacts[i].surface_normal.vector.z = surface_normals_.at(limb)(2);

            //            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::StanceNormal)
            //              desired_contact.foot_contacts[i].is_contact = true;
            //            if(limbs_desired_state.at(limb)->getState() == StateSwitcher::States::SwingNormal)
            //              desired_contact.foot_contacts[i].is_contact = false;
            desired_contact.foot_contacts[i].is_contact = robot_state_->isSupportLeg(limb);//real_contact_.at(limb);


            Eigen::Vector3d joint_torque_leg;
            for(int j = 0;j<3;j++)
            {
                joint_states_leg[i].effort.push_back(joint_actual.effort[3*i+j]);
                joint_states_leg[i].position.push_back(joint_actual.position[3*i+j]);
                joint_states_leg[i].velocity.push_back(joint_actual.velocity[3*i+j]);
                joint_states_leg[i].name.push_back(joint_names_[3*i+j]);
                joint_torque_leg(i) = joint_actual.effort[3*i+j];

                joint_commands_leg[i].effort.push_back(joint_command.effort[3*i+j]);
                joint_commands_leg[i].position.push_back(joint_command.position[3*i+j]);
                joint_commands_leg[i].name.push_back(joint_names_[3*i+j]);

            }


        }
        foot_desired_contact_.push_back(desired_contact);

        desired_robot_state.base_pose = desire_odom;
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::LF_LEG).vector()),
                                           desired_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::RF_LEG).vector()),
                                           desired_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::RH_LEG).vector()),
                                           desired_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(Position(foot_positions.at(free_gait::LimbEnum::LH_LEG).vector()),
                                           desired_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::LF_LEG).vector()),
                                           desired_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::RF_LEG).vector()),
                                           desired_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::RH_LEG).vector()),
                                           desired_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(LinearVelocity(foot_velocities.at(free_gait::LimbEnum::LH_LEG).vector()),
                                           desired_robot_state.lh_target.target_velocity[0].vector);

        desired_robot_state.lf_target.target_force[0].vector = desired_contact.foot_contacts[0].contact_force.wrench.force;
        desired_robot_state.rf_target.target_force[0].vector = desired_contact.foot_contacts[1].contact_force.wrench.force;
        desired_robot_state.rh_target.target_force[0].vector = desired_contact.foot_contacts[2].contact_force.wrench.force;
        desired_robot_state.lh_target.target_force[0].vector = desired_contact.foot_contacts[3].contact_force.wrench.force;

        desired_robot_state.lf_leg_joints = joint_commands_leg[0];
        desired_robot_state.rf_leg_joints = joint_commands_leg[1];
        desired_robot_state.rh_leg_joints = joint_commands_leg[2];
        desired_robot_state.lh_leg_joints = joint_commands_leg[3];

        desired_robot_state_.push_back(desired_robot_state);
        actual_robot_state.base_pose = actual_odom;
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_position[0].point);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_position[0].point);

        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LF_LEG),
                                           actual_robot_state.lf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RF_LEG),
                                           actual_robot_state.rf_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::RH_LEG),
                                           actual_robot_state.rh_target.target_velocity[0].vector);
        kindr_ros::convertToRosGeometryMsg(robot_state_->getEndEffectorVelocityInBaseForLimb(free_gait::LimbEnum::LH_LEG),
                                           actual_robot_state.lh_target.target_velocity[0].vector);

        actual_robot_state.lf_leg_joints = joint_states_leg[0];
        actual_robot_state.rf_leg_joints = joint_states_leg[1];
        actual_robot_state.rh_leg_joints = joint_states_leg[2];
        actual_robot_state.lh_leg_joints = joint_states_leg[3];

//        actual_robot_state.lf_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::LF_LEG).z();
//        actual_robot_state.rf_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::RF_LEG).z();
//        actual_robot_state.rh_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::RH_LEG).z();
//        actual_robot_state.lh_leg_mode.surface_normal.vector.z = real_contact_force_.at(free_gait::LimbEnum::LH_LEG).z();

        actual_robot_state_.push_back(actual_robot_state);


    }


}

void static_walk_controller::starting(const ros::Time &time)
{
    joint_actual_.clear();
    joint_command_.clear();
    foot_desired_contact_.clear();
    desired_robot_state_.clear();
    actual_robot_state_.clear();
    virtual_force_torque_.clear();
    desired_virtual_force_torque_.clear();
    motor_status_word_.clear();
    base_actual_pose_.clear();
    base_command_pose_.clear();
    for(int i = 0;i<joints_.size();i++)
    {
        joints_[i].setCommand(robot_state_handle_.getJointEffortRead()[i]);
    }
}

void static_walk_controller::stopping(const ros::Time &time)
{
    for(int i = 0;i<joints_.size();i++)
    {
        joints_[i].setCommand(0);
    }
}

void static_walk_controller::baseCommandCallback(const free_gait_msgs::RobotStateConstPtr &robot_state_msgs)
{
    ROS_INFO_STREAM_ONCE("in the base command call back");
    base_desired_position_ = Position(robot_state_msgs->base_pose.pose.pose.position.x,
                                     robot_state_msgs->base_pose.pose.pose.position.y,
                                     robot_state_msgs->base_pose.pose.pose.position.z);
    base_desired_rotation_ = RotationQuaternion(robot_state_msgs->base_pose.pose.pose.orientation.w,
                                               robot_state_msgs->base_pose.pose.pose.orientation.x,
                                               robot_state_msgs->base_pose.pose.pose.orientation.y,
                                               robot_state_msgs->base_pose.pose.pose.orientation.z);
    base_desired_linear_velocity_ = LinearVelocity(robot_state_msgs->base_pose.twist.twist.linear.x,
                                                  robot_state_msgs->base_pose.twist.twist.linear.y,
                                                  robot_state_msgs->base_pose.twist.twist.linear.z);
    base_desired_angular_velocity_ = LocalAngularVelocity(robot_state_msgs->base_pose.twist.twist.angular.x,
                                                         robot_state_msgs->base_pose.twist.twist.angular.y,
                                                         robot_state_msgs->base_pose.twist.twist.angular.z);

    Position foot_position, foot_velocity, foot_acceleration;

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lf_target.target_position[0].point,
                                         foot_position);
    foot_positions[free_gait::LimbEnum::LF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lf_target.target_velocity[0].vector,
                                         foot_velocity);
    foot_velocities[free_gait::LimbEnum::LF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lf_target.target_acceleration[0].vector,
                                         foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LF_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rf_target.target_position[0].point,
                                         foot_position);
    foot_positions[free_gait::LimbEnum::RF_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rf_target.target_velocity[0].vector,
                                         foot_velocity);
    foot_velocities[free_gait::LimbEnum::RF_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rf_target.target_acceleration[0].vector,
                                         foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RF_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rh_target.target_position[0].point,
                                         foot_position);
    foot_positions[free_gait::LimbEnum::RH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rh_target.target_velocity[0].vector,
                                         foot_velocity);
    foot_velocities[free_gait::LimbEnum::RH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->rh_target.target_acceleration[0].vector,
                                         foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::RH_LEG] = Vector(foot_acceleration.toImplementation());

    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lh_target.target_position[0].point,
                                         foot_position);
    foot_positions[free_gait::LimbEnum::LH_LEG] = Vector(foot_position.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lh_target.target_velocity[0].vector,
                                         foot_velocity);
    foot_velocities[free_gait::LimbEnum::LH_LEG] = Vector(foot_velocity.toImplementation());
    kindr_ros::convertFromRosGeometryMsg(robot_state_msgs->lh_target.target_acceleration[0].vector,
                                         foot_acceleration);
    foot_accelerations[free_gait::LimbEnum::LH_LEG] = Vector(foot_acceleration.toImplementation());

    robot_state_->setPositionWorldToBaseInWorldFrame(base_desired_position_);
    robot_state_->setOrientationBaseToWorld(RotationQuaternion(base_desired_rotation_));
    robot_state_->setLinearVelocityBaseInWorldFrame(base_desired_linear_velocity_);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_desired_angular_velocity_);

//    std::cout << "-------------in the call back function---------------" << std::endl;
//    std::cout << " the desired position is " << base_desired_position_ << std::endl;

    if(robot_state_msgs->lf_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
    }
    if(robot_state_msgs->lf_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = true;
    }
    if(robot_state_msgs->lf_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
    }
    if(robot_state_msgs->lf_leg_mode.name == "footstep")
    {
        is_footstep_.at(free_gait::LimbEnum::LF_LEG) = true;
        is_cartisian_motion_.at(free_gait::LimbEnum::LF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LF_LEG) = false;
    }

    if(robot_state_msgs->rf_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
    }
    if(robot_state_msgs->rf_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = true;
    }
    if(robot_state_msgs->rf_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
    }
    if(robot_state_msgs->rf_leg_mode.name == "footstep")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RF_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RF_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RF_LEG) = false;
    }

    if(robot_state_msgs->rh_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
    }
    if(robot_state_msgs->rh_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = true;
    }
    if(robot_state_msgs->rh_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
    }
    if(robot_state_msgs->rh_leg_mode.name == "footstep")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::RH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::RH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::RH_LEG) = false;
    }

    if(robot_state_msgs->lh_leg_mode.name == "joint")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
    }
    if(robot_state_msgs->lh_leg_mode.name == "leg_mode")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = true;
    }
    if(robot_state_msgs->lh_leg_mode.name == "cartesian")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
    }
    if(robot_state_msgs->lh_leg_mode.name == "footstep")
    {
        is_cartisian_motion_.at(free_gait::LimbEnum::LH_LEG) = false;
        is_footstep_.at(free_gait::LimbEnum::LH_LEG) = true;
        is_legmode_.at(free_gait::LimbEnum::LH_LEG) = false;
    }

    if(robot_state_msgs->lf_leg_mode.support_leg){
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,
                                       Vector(robot_state_msgs->lf_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->lf_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->lf_leg_mode.surface_normal.vector.z));

    } else {
        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(free_gait::LimbEnum::LF_LEG, false);
        robot_state_->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,Vector(0,0,1));

    };
    if(robot_state_msgs->rf_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),
                                       Vector(robot_state_msgs->rf_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->rf_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->rf_leg_mode.surface_normal.vector.z));
    } else {
        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(1), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(1),Vector(0,0,1));

    };
    if(robot_state_msgs->rh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),
                                       Vector(robot_state_msgs->rh_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->rh_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->rh_leg_mode.surface_normal.vector.z));
       } else {
        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(2), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(2),Vector(0,0,1));
       };
    if(robot_state_msgs->lh_leg_mode.support_leg){
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),
                                       Vector(robot_state_msgs->lh_leg_mode.surface_normal.vector.x,
                                              robot_state_msgs->lh_leg_mode.surface_normal.vector.y,
                                              robot_state_msgs->lh_leg_mode.surface_normal.vector.z));
    } else {
        ROS_WARN("NO Contact !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(3), false);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(3),Vector(0,0,1));

    };
}

bool static_walk_controller::optimization_solve(std_srvs::Empty::Request& req,
                                                std_srvs::Empty::Response& res)
{
    std::cout << " now in the service optimization solve" << std::endl;
    free_gait_msgs::optimize srv;

    /**
        convert robot_state to ros msgs;
    */
    free_gait_msgs::RobotState actual_robot_state_msgs;

    Pose current_pose_in_base;
    Pose current_base_pose = Pose(Position(robot_state_handle_.getPosition()[0],
                                           robot_state_handle_.getPosition()[1],
                                           robot_state_handle_.getPosition()[2]),
                                  RotationQuaternion(robot_state_handle_.getOrientation()[0],
                                                     robot_state_handle_.getOrientation()[1],
                                                     robot_state_handle_.getOrientation()[2],
                                                     robot_state_handle_.getOrientation()[3]));
    current_pose_in_base.getPosition() = current_base_pose.getRotation().inverseRotate(current_base_pose.getPosition());
    current_pose_in_base.getRotation() = current_base_pose.getRotation();
    geometry_msgs::Pose actual_pose;
    kindr_ros::convertToRosGeometryMsg(current_pose_in_base, actual_pose);
    actual_robot_state_msgs.base_pose.pose.pose = actual_pose;

    // leg parameters,actual joint position,velocity and torque;
    sensor_msgs::JointState joint_actual;
    joint_actual.name.resize(12);
    joint_actual.position.resize(12);
    joint_actual.velocity.resize(12);
    joint_actual.effort.resize(12);

    for(unsigned int i=0; i<12; i++)
    {
        joint_actual.position[i] = robot_state_handle_.getJointPositionRead()[i];
        joint_actual.velocity[i] = robot_state_handle_.getJointVelocityRead()[i];
        joint_actual.effort[i] = robot_state_handle_.getJointEffortRead()[i];
    }

    std::vector<sensor_msgs::JointState> joint_states_leg;
    joint_states_leg.resize(4);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            joint_states_leg[i].effort.push_back(joint_actual.effort[3 * i + j]);
            joint_states_leg[i].position.push_back(joint_actual.position[3*i+j]);
            joint_states_leg[i].velocity.push_back(joint_actual.velocity[3*i+j]);
            joint_states_leg[i].name.push_back(joint_names_[3*i+j]);
        }
    }
    actual_robot_state_msgs.lf_leg_joints = joint_states_leg[0];
    actual_robot_state_msgs.rf_leg_joints = joint_states_leg[1];
    actual_robot_state_msgs.rh_leg_joints = joint_states_leg[2];
    actual_robot_state_msgs.lh_leg_joints = joint_states_leg[3];

    // whether the leg is support leg;
    std::vector<free_gait_msgs::LegMode> leg_mode;
    leg_mode.resize(4);
    for (int i = 0; i < 4; i++) {
        leg_mode[i].support_leg = true;
    }
    actual_robot_state_msgs.lf_leg_mode.support_leg = leg_mode[0].support_leg;
    actual_robot_state_msgs.rf_leg_mode.support_leg = leg_mode[1].support_leg;
    actual_robot_state_msgs.rh_leg_mode.support_leg = leg_mode[2].support_leg;
    actual_robot_state_msgs.lh_leg_mode.support_leg = leg_mode[3].support_leg;

    srv.request.robot_state = actual_robot_state_msgs;

    //base force and torque
    geometry_msgs::WrenchStamped desired_vmc_ft;
    desired_vmc_ft.header.frame_id = "/base_link";
    desired_vmc_ft.header.stamp = ros::Time::now();
    kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getDesiredVirtualForceInBaseFrame().vector()),
                                       desired_vmc_ft.wrench.force);
    kindr_ros::convertToRosGeometryMsg(Position(virtual_model_controller_->getDesiredVirtualTorqueInBaseFrame().vector()),
                                       desired_vmc_ft.wrench.torque);

    srv.request.desired_force = desired_vmc_ft;
    if (client_cli_.call(srv))
    {
        ROS_INFO_STREAM("the response is " << srv.response.success);
    }else
    {
        ROS_ERROR("Failed to call service yummmmmmmmy");
        return false;
    }
    ROS_INFO_STREAM("SUCCESS, Ignore the error message~");

    return true;

}


}//namespace
PLUGINLIB_EXPORT_CLASS(balance_controller::static_walk_controller, controller_interface::ControllerBase)
