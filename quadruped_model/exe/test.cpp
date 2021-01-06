/*
 *  kinematicsTest.cpp
 *  Descriotion:
 *
 *  Created on: date, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "quadruped_model/quadrupedkinematics.h"
#include "quadruped_model/QuadrupedModel.hpp"

#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

#include "iostream"
#include "fstream"
#include "sstream"

using namespace std;
using namespace quadruped_model;
int main(int argc, char **argv)
{
    quadruped_model::QuadrupedKinematics QK;
    JointPositionsLimb lf_joint, lh_joint;
    lf_joint << 0, 0, -0;
    lh_joint << 0, 1.4, -2.4;
    Pose lf_pose, lh_pose;

    QK.FowardKinematicsSolve(lh_joint,LimbEnum::LH_LEG, lh_pose);
    std::cout << lh_pose.getPosition() << std::endl;

    QK.getPositionFootToHipInHipFrame(LimbEnum::LH_LEG, lh_pose.getPosition());
    std::cout << lh_pose.getPosition() << std::endl;

    std::cout << QK.getPositionBaseToHipInBaseFrame(LimbEnum::LH_LEG) << std::endl;

    QK.FowardKinematicsSolve(lf_joint,LimbEnum::LF_LEG, lf_pose);
    std::cout <<"lf_position" << lf_pose.getPosition() << std::endl;



    Pose fd_lf_pose, fd_lh_pose;
    fd_lf_pose.getPosition() << 0.275, 0.38, -0.33;
    QK.InverseKinematicsSolve(fd_lf_pose.getPosition(), LimbEnum::LF_LEG, lf_joint, lf_joint, "OUT_LEFT");
    std::cout << "lf_joint " << lf_joint << std::endl;

    fd_lf_pose.getPosition() << 0.275, 0.38, -0.33;
    QK.InverseKinematicsSolve(fd_lf_pose.getPosition(), LimbEnum::LF_LEG, lf_joint, lf_joint, "IN_LEFT");
    std::cout << "lf_joint " << lf_joint << std::endl;



//    fd_lh_pose.getPosition() << -0.125, 0.38, 0;
//    QK.InverseKinematicsSolve(fd_lh_pose.getPosition(), LimbEnum::LH_LEG, lh_joint, lh_joint, "IN_LEFT");

//    std::cout << "lf joint is " << lf_joint << std::endl;
//    std::cout << "lh joint is " << lh_joint << std::endl;


    std::cout <<"base_to_hip in base" << QK.getPositionBaseToHipInBaseFrame(LimbEnum::LF_LEG) << std::endl;
//    Position foot_position_in_hip;
//    std::cout << QK.getPositionFootToHipInHipFrame(LimbEnum::LF_LEG, foot_position_in_hip) << std::endl;
    Eigen::MatrixXd Jacobian_LF(6,3);
    lf_joint << 0.400495, 0.700463, 0.275494;
    QK.AnalysticJacobian(lf_joint, LimbEnum::LF_LEG, Jacobian_LF);

    Eigen::Vector3d force(0,0,1);

    Eigen::Matrix3d jacobian_vel = Jacobian_LF.block<3,3>(0,0);
    std::cout << "jacobian_lf is " << std::endl << Jacobian_LF << std::endl;
    std::cout << "jacobian_vel is " << std::endl << jacobian_vel << std::endl;
    std::cout << "jacobian_vel.transpose.inverse is " << std::endl << jacobian_vel.transpose().inverse() << std::endl;
//    std::cout << "the down part is " << std::endl << Jacobian_LF.block<3,3>(3,0).transpose().inverse() << std::endl;
    std::cout << "the jacobian is " << std::endl <<Jacobian_LF << std::endl;

    Eigen::Vector3d joint_torque;
    joint_torque = jacobian_vel.transpose() * force;

    std::cout << "the force is " << joint_torque << std::endl;





//    QK.FowardKinematicsSolve(lh_joint, LimbEnum::LH_LEG, lh_pose);

//    std::cout << "lf_pose is " << lf_pose.getPosition() << std::endl;
//    std::cout << "lh_pose is " << lh_pose.getPosition() << std::endl;
    return 1;
}

