/*
 *  test.cpp
 *  Descriotion: test the VMC algorithm
 *
 *  Created on: Mar 15, 2019
 *  Author: KANGPENG
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

// gtest
#include "base_motion_planning/base_motion_planning.h"
#include "balance_controller/contact_force_distribution/ContactForceDistribution.hpp"
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include "Eigen/Core"
#include "Eigen/SparseCore"

#include "grid_map_core/Polygon.hpp"
#include "unordered_map"
#include "balance_controller/motion_control/VirtualModelController.hpp"

using namespace balance_controller;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_distribution_test_node");
    ros::NodeHandle nh("~");
    BaseMotionPlanning base_motion_planning_test;
    Pose initial_pose;
    initial_pose.getPosition() << 0,0,0.5;
    initial_pose.getRotation().setIdentity();

    std::shared_ptr<free_gait::State> Robot_state;
    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);

    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);

    Robot_state.reset(new free_gait::State);
    Robot_state->initialize(limbs_, branches_);

    base_motion_planning_test.base_motion(initial_pose);
    double height_1;
    height_1 = 0.2;
    double height_2;
    height_2 = 0;
    std::vector<JointPositionsLimb> lf_joint_positions;
    std::vector<valuetype> lf_foot_position;
    base_motion_planning_test.generate_lf_motion(height_1, height_2, lf_joint_positions, lf_foot_position);
    base_motion_planning_test.generate_rf_motion(height_1, height_2, lf_joint_positions, lf_foot_position);
    base_motion_planning_test.generate_rh_motion(height_1, height_2, lf_joint_positions, lf_foot_position);
    base_motion_planning_test.generate_lh_motion(height_1, height_2, lf_joint_positions, lf_foot_position);

    std::vector<LimbEnum> leg_enum;
    leg_enum.resize(4);
    leg_enum[0] = LimbEnum::LF_LEG;
    leg_enum[1] = LimbEnum::RF_LEG;
    leg_enum[2] = LimbEnum::RH_LEG;
    leg_enum[3] = LimbEnum::LH_LEG;

    grid_map::Polygon support_area;
    free_gait::Stance current_stance;
    double test_time = 0;
    std::vector<std::vector<int>> leg_contacts;
    std::vector<int> contact_state;
    contact_state.resize(6);

    //store the foot force;
    typedef std::unordered_map<LimbEnum, Force, EnumClassHash> Leg_Force;
    Leg_Force leg_force_map;

    typedef kindr::VectorTypeless<double,24> twenty_4_values;
    twenty_4_values all_joint_torques, all_joint_angles, all_foot_forces;
    std::vector<twenty_4_values> all_joint_torques_vector, all_joint_angles_vector, all_foot_forces_vec;

    std::shared_ptr<quadruped_model::QuadrupedKinematics> quadruped_kin;
    quadruped_kin.reset(new quadruped_model::QuadrupedKinematics);

    //calculate the base_force and torque;
    std::shared_ptr<balance_controller::ContactForceDistribution> contact_force_test;
    contact_force_test.reset(new balance_controller::ContactForceDistribution(nh,Robot_state));

    std::shared_ptr<balance_controller::VirtualModelController> virtual_model_control_;
    virtual_model_control_.reset(new balance_controller::VirtualModelController(nh,Robot_state,contact_force_test));

    contact_force_test->loadParameters();
    virtual_model_control_->loadParameters();

    RotationQuaternion base_rotation;
    base_rotation.setIdentity();
    Robot_state->setOrientationBaseToWorld(base_rotation);//desired rotation

    Position base_position;
    base_position.setZero();
    Robot_state->setPositionWorldToBaseInWorldFrame(base_position);//desired position

    Pose base_in_world_;
    base_in_world_.getPosition().setZero();
    base_in_world_.getRotation().setIdentity();
    Robot_state->setPoseBaseToWorld(base_in_world_);//actual position and rotation

    LinearVelocity target_linear_velocity;//target linear velocity
    target_linear_velocity.setZero();
    Robot_state->setLinearVelocityBaseInWorldFrame(target_linear_velocity);

    LocalAngularVelocity local_angular_velocity;//target angular velocity;
    local_angular_velocity.setZero();
    Robot_state->setAngularVelocityBaseInBaseFrame(local_angular_velocity);

    LinearVelocity base_linear_velocity;//base feedback velocity;
    LocalAngularVelocity base_angular_velocity;
    base_linear_velocity.setZero();
    base_angular_velocity.setZero();
    Robot_state->setBaseStateFromFeedback(base_linear_velocity,base_angular_velocity);

    free_gait::Vector surface_normal;
    surface_normal << 0, 0, 1;
    Robot_state->setSurfaceNormal(free_gait::LimbEnum::LF_LEG,surface_normal);
    Robot_state->setSurfaceNormal(free_gait::LimbEnum::RF_LEG,surface_normal);
    Robot_state->setSurfaceNormal(free_gait::LimbEnum::LH_LEG,surface_normal);
    Robot_state->setSurfaceNormal(free_gait::LimbEnum::RH_LEG,surface_normal);


    while(test_time < 40)
    {
        std::vector<leg_info> leg_info;
        leg_info = base_motion_planning_test.GetFootPosition(test_time);
        current_stance.clear();

        for(unsigned int i = 0; i < 4; i++)
        {
            if(leg_info[i].is_contact)
            {
                current_stance[leg_enum[i]] = free_gait::Position(leg_info[i].foot_position);
            }
//            std::cout << "in " << i << "the foot position is " << leg_info[i].foot_position << std::endl;
            Robot_state->setSupportLeg(leg_enum[i], leg_info[i].is_contact);
            Robot_state->setTargetFootPositionInBaseForLimb(Position(leg_info[i].foot_position), leg_enum[i]);
            contact_state[i] = bool(leg_info[i].is_contact);
        }

//        std::cout << current_stance.size() << std::endl;
        Robot_state->setSupportFootStance(current_stance);
        std::vector<free_gait::Position> footholdOrdered;
        free_gait::getFootholdsCounterClockwiseOrdered(current_stance, footholdOrdered);
        for (auto foothold : footholdOrdered) {
            support_area.addVertex(foothold.vector().head<2>());
        }
        support_area.offsetInward(0.00);
        grid_map::Position base_position;
        base_position << 0,0;
        bool isinside;
        isinside = support_area.isInside(base_position);
        contact_state[4] = isinside;
        contact_state[5] = int(support_area.nVertices());
//        std::cout << "in time " << test_time << std::endl;
        test_time = test_time + 0.025;

        if(isinside == false)
        {
            for (unsigned int i =0; i < 4; i++) {
                std::cout <<"leg foot is " <<leg_info[i].foot_position << std::endl;
            }
        }
        support_area.removeVertices();
        leg_contacts.push_back(contact_state);

        //calculate the force
        free_gait::Pose base_pose;
        base_pose.getPosition() << 0,0,0;
        base_pose.getRotation().setIdentity();
        Robot_state->setPoseBaseToWorld(base_pose);


        Force force_foot;
        Torque torque_foot;

        virtual_model_control_->compute();
        if(!virtual_model_control_->compute())
        {
            ROS_WARN_STREAM("*virtual_model_controller_");
        }
        virtual_model_control_->getDistributedVirtualForceAndTorqueInBaseFrame(force_foot, torque_foot);
        std::cout << " the force foot is " << force_foot << std::endl;
        std::cout << " the force torq is " << torque_foot << std::endl;

        //now get the foot position and do the inverse kinematics calculation;

        std::vector<JointPositions> vector_joints_positions;
        JointPositions joint_positions_1, joint_positions_2;
        JointPositionsLimb joint_angles;
        free_gait::JointPositions joint_positions;

        Eigen::MatrixXd jacobian_1, jacobian_2;

        JointTorques joint_torques_1, joint_torques_2;
        JointTorquesLimb joint_torque_limb;

        for(auto iter = current_stance.begin(); iter != current_stance.end(); iter++)
        {
//            std::cout << "Position 1 is" << Position(iter->second) << std::endl;
            quadruped_kin->InverseKinematicsSolve(Position(iter->second),iter->first,joint_angles, joint_angles,"IN_LEFT");

            all_joint_angles(3 * int(iter->first)) = joint_angles.x();
            all_joint_angles(3 * int(iter->first) + 1) = joint_angles.y();
            all_joint_angles(3 * int(iter->first) + 2) = joint_angles.z();

            joint_positions(3 * int(iter->first)) = joint_angles.x();
            joint_positions(3 * int(iter->first) + 1) = joint_angles.y();
            joint_positions(3 * int(iter->first) + 2) = joint_angles.z();
//            std::cout << "item is " << int(iter->first) << std::endl;
//            std::cout << "joint_positions is " << joint_positions << std::endl;
        }//get the joint angles


        Robot_state->setAllJointPositions(joint_positions);

        contact_force_test->computeForceDistribution(force_foot, torque_foot);//calculate each foot force;

        for (unsigned int i = 0; i < 4; i++) {
            leg_force_map[leg_enum[i]] = contact_force_test->getLegInfo(leg_enum[i]).desiredContactForce_;
            all_foot_forces(3 * i) = leg_force_map[leg_enum[i]].x();
            all_foot_forces(3 * i + 1) = leg_force_map[leg_enum[i]].y();
            all_foot_forces(3 * i + 2) = leg_force_map[leg_enum[i]].z();
        }

        //calculate each leg joint torque;
        for(auto iter = current_stance.begin();iter != current_stance.end(); iter++)
        {
            quadruped_kin->InverseKinematicsSolve(Position(iter->second),iter->first,joint_angles, joint_angles,"IN_LEFT");
            quadruped_kin->AnalysticJacobian(joint_angles,iter->first,jacobian_1);
            joint_torque_limb << jacobian_1.block<3,3>(0,0).transpose()* leg_force_map.at(iter->first).toImplementation();
            all_joint_torques(3 * int(iter->first)) = joint_torque_limb.x();
            all_joint_torques(3 * int(iter->first) + 1) = joint_torque_limb.y();
            all_joint_torques(3 * int(iter->first) + 2) = joint_torque_limb.z();
        }

        //the second configuration

        for (auto iter = current_stance.begin(); iter != current_stance.end(); iter++) {
            quadruped_kin->InverseKinematicsSolve(Position(iter->second),iter->first,joint_angles,joint_angles,"OUT_LEFT");

            all_joint_angles(12 + 3 * int(iter->first)) = joint_angles.x();
            all_joint_angles(12 + 3 * int(iter->first) + 1) = joint_angles.y();
            all_joint_angles(12 + 3 * int(iter->first) + 2) = joint_angles.z();

            joint_positions(3 * int(iter->first)) = joint_angles.x();
            joint_positions(3 * int(iter->first) + 1) = joint_angles.y();
            joint_positions(3 * int(iter->first) + 2) = joint_angles.z();
//            std::cout << "joint_torque 2 is " << joint_torques_1 << std::endl;
        }
        Robot_state->setAllJointPositions(joint_positions);
        contact_force_test->computeForceDistribution(force_foot, torque_foot);

        for (unsigned int i = 0; i < 4; i++) {
            leg_force_map[leg_enum[i]] = contact_force_test->getLegInfo(leg_enum[i]).desiredContactForce_;
            all_foot_forces(12 + 3 * i) = leg_force_map[leg_enum[i]].x();
            all_foot_forces(12 + 3 * i + 1) = leg_force_map[leg_enum[i]].y();
            all_foot_forces(12 + 3 * i + 2) = leg_force_map[leg_enum[i]].z();
        }

        //calculate each leg joint;
         for (auto iter = current_stance.begin(); iter != current_stance.end(); iter++) {
             quadruped_kin->InverseKinematicsSolve(Position(iter->second),iter->first,joint_angles,joint_angles,"OUT_LEFT");
             quadruped_kin->AnalysticJacobian(joint_angles,iter->first,jacobian_2);
             joint_torque_limb << jacobian_2.block<3,3>(0,0).transpose()*leg_force_map.at(iter->first).toImplementation();
             all_joint_torques(12 + 3 * int(iter->first)) = joint_torque_limb.x();
             all_joint_torques(12 + 3 * int(iter->first) + 1) = joint_torque_limb.y();
             all_joint_torques(12 + 3 * int(iter->first) + 2) = joint_torque_limb.z();
         }


//        std::cout << "joint_position is " << joint_positions_1 << std::endl;

        all_joint_torques_vector.push_back(all_joint_torques);
        all_joint_angles_vector.push_back(all_joint_angles);
        all_foot_forces_vec.push_back(all_foot_forces);

    }
    base_motion_planning_test.SaveAsFile("leg_contacts.txt", leg_contacts);
    base_motion_planning_test.SaveAsFile_1_vector("all_joints_angles.txt",all_joint_angles_vector);
    base_motion_planning_test.SaveAsFile_1_vector("all_joints_torque.txt",all_joint_torques_vector);
    base_motion_planning_test.SaveAsFile_1_vector("foot_force.txt",all_foot_forces_vec);

    return 1;
}
