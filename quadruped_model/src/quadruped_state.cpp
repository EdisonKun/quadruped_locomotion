/*
 *  quadruped_state.cpp
 *  Descriotion:
 *
 *  Created on: Mar 15, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "quadruped_model/quadruped_state.h"
namespace quadruped_model {
//JointPositions QuadrupedState::joint_positions_ = JointPositions(Eigen::VectorXd::Zero(12,1));
JointPositions QuadrupedState::joint_positions_ = JointPositions().setZero();
JointPositions QuadrupedState::joint_positions_feedback_ = JointPositions(Eigen::VectorXd::Zero(12,1));
JointVelocities QuadrupedState::joint_velocities_ = JointVelocities(Eigen::VectorXd::Zero(12,1));

QuadrupedState::QuadrupedState()
  : QuadrupedKinematics(),
    robot_mass_(25.0),
    CoM_in_base_(Position(0,0,0))
{
//  QuadrupedKinematics::LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
//  QK = new QuadrupedKinematics()
//  QK.LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
//  LoadRobotDescriptionFromFile("/home/hitstar/catkin_ws/src/quadruped_locomotion-dev/quadruped_model/urdf/simpledog.urdf");
  std::cout<<"Constructor QuadrupedState"<<std::endl;
  limb_mass_ = limb_mass({
                         {LimbEnum::LF_LEG, 3.5},
                         {LimbEnum::RF_LEG, 3.5},
                         {LimbEnum::RH_LEG, 3.5},
                         {LimbEnum::LH_LEG, 3.5},
                       });
//  setPoseBaseToWorld(Pose(Position(0,0,0), RotationQuaternion()));
//  joint_positions_ << 0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14;
//  setCurrentLimbJoints(joint_positions_);
};

//QuadrupedState::QuadrupedState(const QuadrupedState& other)
//    :

//{

//}

QuadrupedState::~QuadrupedState()
{
 std::cout<<"QuadrupedState Destroied"<<std::endl;
};
bool QuadrupedState::Initialize()
{
    std::cout<<"Initialize QuadrupedState"<<std::endl;
    setLimbConfigure("><");
    setPoseBaseToWorld(Pose(Position(0,0,0), RotationQuaternion()));
    joint_positions_ << 0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14,0,1.57,-3.14;
    //allJointPositionsToReach_ = joint_positions_;
    setCurrentLimbJoints(joint_positions_);
}
Position QuadrupedState::getCenterOfMassInBase()
{
  return CoM_in_base_;
}
double QuadrupedState::getRobotMass()
{
  return robot_mass_;
}

Position QuadrupedState::getPositionLegBaseToCoMInBaseFrame(const LimbEnum& limb) const
{
  switch (limb) {
    case LimbEnum::LF_LEG:
      return Position(0.3, 0.2, 0.0) - CoM_in_base_;
    case LimbEnum::RF_LEG:
      return Position(0.3, -0.2, 0.0) - CoM_in_base_;
    case LimbEnum::LH_LEG:
      return Position(-0.3, 0.2, 0.0) - CoM_in_base_;
    case LimbEnum::RH_LEG:
      return Position(-0.3, -0.2, 0.0) - CoM_in_base_;
    default:
      throw std::runtime_error("Position QuadrupedState::getPositionLegBaseToCoMInBaseFrame(const LimbEnum& limb) const something went wrong.");
  }
}
double QuadrupedState::getLegMass(const LimbEnum& limb) const
{
  return limb_mass_.at(limb);
}
const Position QuadrupedState::getPositionWorldToBaseInWorldFrame() const
{
  return poseInWorldFrame_.getPosition();//TODO(Shunyao):How to update poseInWorldFrame
};


bool QuadrupedState::setPoseBaseToWorld(const Pose pose)
{
  //! WSHY: feedback
  poseInWorldFrame_ = pose;
  return true;
};
bool QuadrupedState::setBaseStateFromFeedback(const LinearVelocity& base_linear_velocity,
                                              const LocalAngularVelocity& base_angular_velocity)
{
  base_feedback_linear_velocity_ = base_linear_velocity;
  base_feedback_angular_velocity_ = base_angular_velocity;
  return true;
}
const Position QuadrupedState::getPositionWorldToFootInWorldFrame(const LimbEnum& limb)
{
  Position foot_in_base, base_in_world;
//  setCurrentLimbJoints(joint_positions_);
  foot_in_base = getPositionBaseToFootInBaseFrame(limb);
  base_in_world = poseInWorldFrame_.getPosition();
  return poseInWorldFrame_.getRotation().rotate(foot_in_base) + base_in_world;//poseInWorldFrame_.getPosition() + getPositionWorldToFootInWorldFrame(limb);
}
const Position QuadrupedState::getPositionBaseToFootInBaseFrame(const LimbEnum& limb)
{
//  std::cout<<"get here"<<std::endl;
  JointPositionsLimb jointPositions = current_limb_joints_.at(limb);
  std::cout<<"in getPositionBaseToFootInBaseFrame() jointPositions  "<<jointPositions<<std::endl;
  Pose foot_pose;
  FowardKinematicsSolve(jointPositions, limb, foot_pose);
  footPoseInBaseFrame_[limb] = foot_pose;
    std::cout<<"in getPositionBaseToFootInBaseFrame()  "<<foot_pose.getPosition()<<std::endl;
  return foot_pose.getPosition();
}

Position QuadrupedState::getPositionBaseToFootInBaseFrame(const LimbEnum& limb, const JointPositionsLimb& jointPositions)
{
  Pose pose_base_to_foot_in_base;
//  std::cout<<hip_pose_in_base_.at(LimbEnum::LF_LEG)<<std::endl;
  std::cout<<"in getPositionBaseToFootInBaseFrame() jointPositions  "<<jointPositions<<std::endl;
  FowardKinematicsSolve(jointPositions, limb, pose_base_to_foot_in_base);
  return pose_base_to_foot_in_base.getPosition();
}

const RotationQuaternion QuadrupedState::getOrientationBaseToWorld() const
{
  return  poseInWorldFrame_.getRotation();
}

const JointPositions& QuadrupedState::getJointPositionsToReach() const
{
    return allJointPositionsToReach_;
}

JointPositions& QuadrupedState::getJointPositions()
{
  //TODO(shunyao): `joint_position_` was declared as a static member, which means
  //  it doesn't change with the copy of class, so it always has one value, but is
  //there necessary to use static member? And how to fix it?
  return joint_positions_;// joint command
}
JointVelocities& QuadrupedState::getJointVelocities()
{
  return joint_velocities_;
}

const LinearVelocity QuadrupedState::getLinearVelocityBaseInWorldFrame() const
{
  return base_feedback_linear_velocity_;
}
const LocalAngularVelocity QuadrupedState::getAngularVelocityBaseInBaseFrame() const
{
  return base_feedback_angular_velocity_;
}
const LinearVelocity QuadrupedState::getTargetLinearVelocityBaseInWorldFrame() const
{
  return base_target_linear_velocity_;
}
const LocalAngularVelocity QuadrupedState::getTargetAngularVelocityBaseInBaseFrame() const
{
  return base_target_angular_velocity_;
}

bool QuadrupedState::setPositionWorldToBaseInWorldFrame(const Position position)
{
  //! WSHY: Target
  positionWorldToBaseInWorldFrame_ = position;
//  poseInWorldFrame_.getPosition() = position;
  return true;
}
bool QuadrupedState::setOrientationBaseToWorld(const RotationQuaternion rotation)
{
  //! WSHY: Target
  orientationBaseToWorld_ = rotation;
//  poseInWorldFrame_.getRotation() = rotation;
  return true;
}

const Position QuadrupedState::getTargetPositionWorldToBaseInWorldFrame() const
{
  return positionWorldToBaseInWorldFrame_;
}
const RotationQuaternion QuadrupedState::getTargetOrientationBaseToWorld() const
{
  return orientationBaseToWorld_;
}

bool QuadrupedState::setLinearVelocityBaseInWorldFrame(const LinearVelocity linear_velocity)
{
  base_target_linear_velocity_ = linear_velocity;
  return true;
}
bool QuadrupedState::setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity local_angular_velocity)
{
  base_target_angular_velocity_ = local_angular_velocity;
  return true;
}

bool QuadrupedState::setJointPositions(const JointPositions joint_positions)
{
  joint_positions_ = joint_positions;
//  allJointPositionsToReach_ = joint_positions_;
  setCurrentLimbJoints(joint_positions_);
  return true;
}
bool QuadrupedState::setJointVelocities(const JointVelocities joint_velocoties)
{
  return true;
}

bool QuadrupedState::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(const Position& positionBaseToFootInBaseFrame,
                                                            const LimbEnum& limb,
                                                            JointPositionsLimb& jointPositions)
{
//  std::cout<<"in quadruped state"<<current_limb_joints_.at(limb)<<std::endl;
  JointPositionsLimb joint_position_last = current_limb_joints_.at(limb);
//  std::cout<<limb<<std::endl;
  if(!InverseKinematicsSolve(positionBaseToFootInBaseFrame,limb,joint_position_last,jointPositions, limb_configure_.at(limb)))
  {
      ROS_WARN("Wrong Solved Joint Positions, Keep it as last");
      jointPositions = joint_position_last;
      return false;

  }
//  std::cout<<"***********************************"<<std::endl
//     <<jointPositions<<std::endl
//    <<"**********************************"<<std::endl;
  return true;

}
LinearVelocity QuadrupedState::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                               const JointVelocitiesLimb& jointVelocities)
{
  //TODO(Shunyao) : where to update joint Positions
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd joint_diff(3);
  joint_diff << jointVelocities(0), jointVelocities(1), jointVelocities(2);
  AnalysticJacobian(current_limb_joints_.at(limb), limb, jacobian);
  Eigen::VectorXd pose_diff = jacobian * joint_diff;
  return LinearVelocity(pose_diff(0), pose_diff(1), pose_diff(2));

}

Eigen::Matrix3d QuadrupedState::getTranslationJacobianFromBaseToFootInBaseFrame(const LimbEnum& limb)
{
  Eigen::MatrixXd jacobian;
  AnalysticJacobian(current_limb_joints_.at(limb), limb, jacobian);
  return jacobian.block(0,0,3,3);
}

void QuadrupedState::setCurrentLimbJoints(JointPositions all_joints_position)
{
  /****************
* TODO(Shunyao) : update current joint position, feedback,
****************/
  allJointPositionsToReach_ = all_joints_position;
  current_limb_joints_[LimbEnum::LF_LEG] = JointPositionsLimb(all_joints_position(0),all_joints_position(1),all_joints_position(2));
  current_limb_joints_[LimbEnum::RF_LEG] = JointPositionsLimb(all_joints_position(3),all_joints_position(4),all_joints_position(5));
  current_limb_joints_[LimbEnum::RH_LEG] = JointPositionsLimb(all_joints_position(6),all_joints_position(7),all_joints_position(8));
  current_limb_joints_[LimbEnum::LH_LEG] = JointPositionsLimb(all_joints_position(9),all_joints_position(10),all_joints_position(11));
  for(auto leg : limb_configure_)
    {
      Pose foot_pose;
      FowardKinematicsSolve(current_limb_joints_.at(leg.first), leg.first, foot_pose);
      footPoseInBaseFrame_[leg.first] = foot_pose;
//      foot_positions_[leg.first] = foot_pose.getPosition();

    }
//  setSupportFootStance(foot_positions_);

}

bool QuadrupedState::setLimbConfigure(const std::string leg_configure)
{
  // Notes: bend to the positive direction of base coordinate X is IN, leg In the positive
  //        direction of coordinate Y is LEFT
  if(leg_configure == "<<"){//Checked OK
      limb_configure_[LimbEnum::LF_LEG] = "IN_LEFT";
      limb_configure_[LimbEnum::LH_LEG] = "IN_LEFT";
      limb_configure_[LimbEnum::RF_LEG] = "OUT_LEFT";
      limb_configure_[LimbEnum::RH_LEG] = "OUT_LEFT";
    }
  if(leg_configure == "<>"){//Checked OK
      limb_configure_[LimbEnum::LF_LEG] = "OUT_LEFT";
      limb_configure_[LimbEnum::LH_LEG] = "IN_LEFT";
      limb_configure_[LimbEnum::RF_LEG] = "IN_LEFT";
      limb_configure_[LimbEnum::RH_LEG] = "OUT_LEFT";
    }
  if(leg_configure == "><"){ //Checked OK
      limb_configure_[LimbEnum::LF_LEG] = "IN_LEFT";
      limb_configure_[LimbEnum::LH_LEG] = "OUT_LEFT";
      limb_configure_[LimbEnum::RF_LEG] = "OUT_LEFT";
      limb_configure_[LimbEnum::RH_LEG] = "IN_LEFT";
    }
  if(leg_configure == ">>"){
      limb_configure_[LimbEnum::LF_LEG] = "OUT_LEFT";
      limb_configure_[LimbEnum::LH_LEG] = "OUT_LEFT";
      limb_configure_[LimbEnum::RF_LEG] = "IN_LEFT";
      limb_configure_[LimbEnum::RH_LEG] = "IN_LEFT";
    }

}

}; //namespace
