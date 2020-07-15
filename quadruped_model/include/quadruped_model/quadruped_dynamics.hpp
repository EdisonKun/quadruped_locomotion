#pragma once
#include"rbdl/Model.h"
#include"rbdl/rbdl.h"
#include"rbdl/addons/urdfreader/urdfreader.h"
#include"ros/ros.h"
#include"ros/package.h"
#include"string.h"
#include"iostream"

#include"ct/core/core.h"
#include"ct/rbd/Common"

namespace quadruped_model {
class RobotDynamics
{
public:
    RobotDynamics();

private:
    RigidBodyDynamics::Model* quadruped_model_;

};//class RobotDynamics

}//namespace quadruped_model
