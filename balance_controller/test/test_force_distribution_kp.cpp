/*
 *  test.cpp
 *  Descriotion: test the VMC algorithm
 *
 *  Created on: Oct 14, 2020
 *  Author: Edison Kun
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

// gtest
#include <gtest/gtest.h>

#include "ros/ros.h"

#include "balance_controller/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "balance_controller/contact_force_distribution/ContactJointTorqueDistribution.hpp"

#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include "Eigen/Core"
#include "Eigen/SparseCore"

#include <grid_map_core/Polygon.hpp>
#include "balance_controller/motion_control/VirtualModelController.hpp"
#include <Eigen/Geometry>
#include "Eigen/Sparse"

#include "ifopt/bounds.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "force_distribution_test_node");
  ros::NodeHandle nh("~");

  std::shared_ptr<free_gait::State> Robot_state;
  std::shared_ptr<balance_controller::VirtualModelController> virtual_model_controller_;
  std::shared_ptr<balance_controller::ContactJointTorqueDistribution> contact_joint_torque_test;
  Robot_state.reset(new free_gait::State);
  contact_joint_torque_test.reset(new balance_controller::ContactJointTorqueDistribution(nh,Robot_state));
  virtual_model_controller_.reset(new balance_controller::VirtualModelController(nh,Robot_state,contact_joint_torque_test));

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


  Robot_state->initialize(limbs_, branches_);
  Robot_state->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
  Robot_state->setSupportLeg(free_gait::LimbEnum::RF_LEG, true);
  Robot_state->setSupportLeg(free_gait::LimbEnum::RH_LEG, true);
  Robot_state->setSupportLeg(free_gait::LimbEnum::LH_LEG, true);

  free_gait::JointPositions joint_positions;
  free_gait::JointPositionsLeg joint_angles;
  free_gait::Pose pose_1;
  pose_1.getPosition()<< 0,0,0;
  pose_1.getRotation().setIdentity();

  free_gait::JointPositions all_joint_positons;
  Position lf_position,rf_position,lh_position,rh_position;
  lf_position << 0.38,0.305,-0.45;
  Robot_state->InverseKinematicsSolve(lf_position,free_gait::LimbEnum::LF_LEG,joint_angles,joint_angles,"IN_LEFT");
  Robot_state->setJointPositionsForLimb(free_gait::LimbEnum::LF_LEG, joint_angles);
  all_joint_positons.toImplementation().segment(0,3) << joint_angles.x(),joint_angles.y(),joint_angles.z();

  rf_position << 0.38,-0.305,-0.45;
  Robot_state->InverseKinematicsSolve(rf_position,free_gait::LimbEnum::RF_LEG,joint_angles,joint_angles,"IN_LEFT");
  all_joint_positons.toImplementation().segment(3,3) << joint_angles.x(),joint_angles.y(),joint_angles.z();

  lh_position << -0.38,0.305,-0.45;
  Robot_state->InverseKinematicsSolve(lh_position,free_gait::LimbEnum::LH_LEG,joint_angles,joint_angles,"IN_LEFT");
  all_joint_positons.toImplementation().segment(9,3) << joint_angles.x(),joint_angles.y(),joint_angles.z();

  rh_position << -0.38,-0.305,-0.45;
  Robot_state->InverseKinematicsSolve(rh_position,free_gait::LimbEnum::RH_LEG,joint_angles,joint_angles,"IN_LEFT");
  all_joint_positons.toImplementation().segment(6,3) << joint_angles.x(),joint_angles.y(),joint_angles.z();

  Robot_state->setJointPositions(all_joint_positons);


  free_gait::Pose base_pose;
  base_pose.getPosition() << 0,0,0;
  base_pose.getRotation().setIdentity();
  Robot_state->setPoseBaseToWorld(base_pose);
  contact_joint_torque_test->loadParameters();
  //virtual controller~~~~~~~~~~~~~~~~~~
  std::cout << "test the virtual controller" << std::endl;
  virtual_model_controller_->loadParameters();

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

  std::cout << "prepare to compute" << std::endl;

  std::cout << "the desired force is " << virtual_model_controller_->getDesiredVirtualForceInBaseFrame() << std::endl;
  std::cout << "the desired torque is " << virtual_model_controller_->getDesiredVirtualTorqueInBaseFrame() << std::endl;

  if(!virtual_model_controller_->compute())
  {
      ROS_ERROR("VMC compute failed");
      ROS_WARN_STREAM(*virtual_model_controller_);
  }
  std::cout << "the desired force is " << virtual_model_controller_->getDesiredVirtualForceInBaseFrame() << std::endl;
  std::cout << "the desired torque is " << virtual_model_controller_->getDesiredVirtualTorqueInBaseFrame() << std::endl;

  Eigen::SparseMatrix<double, Eigen::RowMajor> jacobian_matrix_inv;
  jacobian_matrix_inv = contact_joint_torque_test->getTheJacobianMatrixInv();

  Eigen::VectorXd foot_force, joint_torque;
  foot_force.resize(12);
  joint_torque.resize(contact_joint_torque_test->getTheOptimizationResults().size());

  joint_torque = contact_joint_torque_test->getTheOptimizationResults();
  foot_force = jacobian_matrix_inv * joint_torque;
  std::cout << "foot_force is " << foot_force << std::endl;

  free_gait::Force leg_force;
  Force force_in_base;
  Torque torque_in_base;
  contact_joint_torque_test->getNetForceAndTorqueOnBase(force_in_base,torque_in_base);
  std::cout << "force is " << force_in_base <<std::endl << "torque is " << torque_in_base << std::endl;
  std::cout << "the optimization result is " << contact_joint_torque_test->getTheOptimizationResults().transpose() << std::endl;

  std::cout << "finish the calculation" << std::endl;



//  Force force_foot;
//  Torque torque_foot;
//  force_foot << 0,0,100;
//  torque_foot << 0,0,0;

//  contact_joint_torque_test->computeForceDistribution(force_foot, torque_foot);



//  std::vector<free_gait::Force> foot_force;
//  foot_force.resize(4);
//  foot_force.at(0) = contact_joint_torque_test->getLegInfo(free_gait::LimbEnum::LF_LEG).desiredContactForce_;
//  foot_force.at(1) = contact_joint_torque_test->getLegInfo(free_gait::LimbEnum::RF_LEG).desiredContactForce_;
//  foot_force.at(2) = contact_joint_torque_test->getLegInfo(free_gait::LimbEnum::LH_LEG).desiredContactForce_;
//  foot_force.at(3) = contact_joint_torque_test->getLegInfo(free_gait::LimbEnum::RH_LEG).desiredContactForce_;
//  std::cout << "the desired foot force LF is " << foot_force[0] << std::endl;
//  std::cout << "the desired foot force RF is " << foot_force[1] << std::endl;
//  std::cout << "the desired foot force LH is " << foot_force[2] << std::endl;
//  std::cout << "the desired foot force RH is " << foot_force[3] << std::endl;

//  std::cout << contact_joint_torque_test->getMinimalNormalGroundForce() << std::endl;//minimal contact force

//  std::shared_ptr<quadruped_model::QuadrupedKinematics> quadruped_kinematics;
//  quadruped_kinematics.reset(new quadruped_model::QuadrupedKinematics);

//  free_gait::Position lf_position;
//  lf_position << 0.3,0.3,-0.5;
//  free_gait::JointPositionsLeg lf_joints;
//  free_gait::Pose pose;
//  quadruped_kinematics->InverseKinematicsSolve(lf_position, free_gait::LimbEnum::LF_LEG,lf_joints,lf_joints,"IN_LEFT");
//  std::cout << "lf_joint 1" << lf_joints << std::endl;
//  quadruped_kinematics->FowardKinematicsSolve(lf_joints, free_gait::LimbEnum::LF_LEG, pose);
//  std::cout << "lf_joint 1 forward is " << pose.getPosition() << std::endl;
//  Eigen::MatrixXd jacobian;
//  quadruped_kinematics->AnalysticJacobian(lf_joints,free_gait::LimbEnum::LF_LEG, jacobian);

//  free_gait::Force lf_force;
//  lf_force << 10,10,100;

//  free_gait::Torque lf_joint_torque;
//  Eigen::Matrix3d force_jacobian;
//  force_jacobian = jacobian.block<3,3>(0,0).transpose();

//  lf_joint_torque << force_jacobian * lf_force.toImplementation();
//  std::cout << "lf_joints 1 is " << lf_joints << std::endl;
//  std::cout << "lf_joint_torque is " << lf_joint_torque << std::endl;


//  quadruped_kinematics->InverseKinematicsSolve(lf_position, free_gait::LimbEnum::LF_LEG,lf_joints,lf_joints,"OUT_LEFT");
//  std::cout << "lf_joint 2" << lf_joints << std::endl;
//  quadruped_kinematics->FowardKinematicsSolve(lf_joints, free_gait::LimbEnum::LF_LEG, pose);
//  std::cout << "lf_joint 2 forward is " << pose.getPosition() << std::endl;

//  Eigen::MatrixXd jacobian2;
//  quadruped_kinematics->AnalysticJacobian(lf_joints,free_gait::LimbEnum::LF_LEG, jacobian2);

//  std::cout << "lf_joints 2 is " << lf_joints << std::endl;
//  free_gait::Torque lf_joint_torque2;
//  Eigen::Matrix3d force_jacobian2;
//  force_jacobian2 = jacobian2.block<3,3>(0,0).transpose();
//  lf_joint_torque2 << force_jacobian2 * lf_force.toImplementation();
//  std::cout << "lf_joint_torque 2 is " << lf_joint_torque2 << std::endl;

//  //get the support polygon
//  //judge the support leg;


//  grid_map::Polygon support_area;

//  free_gait::Stance current_stance;
//  current_stance[free_gait::LimbEnum::LF_LEG] = free_gait::Position(0.3, 0.2, -0.5);
//  current_stance[free_gait::LimbEnum::LH_LEG] = free_gait::Position(-0.3, 0.2, -0.5);
//  current_stance[free_gait::LimbEnum::RH_LEG] = free_gait::Position(-0.3, -0.2, -0.5);
//  current_stance[free_gait::LimbEnum::RF_LEG] = free_gait::Position(0.3, -0.2, -0.5);

//  Robot_state->setSupportFootStance(current_stance);

//  std::vector<free_gait::Position> footholdOrdered;
//  free_gait::getFootholdsCounterClockwiseOrdered(current_stance, footholdOrdered);
//  for (auto foothold : footholdOrdered) {
//      support_area.addVertex(foothold.vector().head<2>());
//  }
//  support_area.offsetInward(0.15);
//  grid_map::Position base_position_test;
//  base_position_test << 0,0;
//  bool isinside;
//  isinside = support_area.isInside(base_position_test);
//  std::cout <<"isinside" << isinside << std::endl;

//  std::cout << (Eigen::Matrix3d::Identity().replicate(1, 7))<< std::endl;

//  using namespace std;
//  Eigen::MatrixXf m(4,4);
//  m <<  1, 2, 3, 4,
//      5, 6, 7, 8,
//      9,10,11,12,
//      13,14,15,16;
//  cout << "Block in the middle" << endl;
//  cout << m.block<1,2>(2,2) << endl << endl;//position ,2,2. block size 1,1
//  for (int i = 1; i <= 3; ++i)
//  {
//      cout << "Block of size " << i << "x" << i << endl;
//      cout << m.block(0,0,i,i) << endl << endl;
//  }






}




