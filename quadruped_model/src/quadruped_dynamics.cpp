#include "quadruped_model/quadruped_dynamics.hpp"
namespace quadruped_model {

RobotDynamics::RobotDynamics()
{
    quadruped_model_ = new RigidBodyDynamics::Model();
    std::string model_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model.urdf";
    char* model_urdf_dir = (char*) model_urdf_dir_str.c_str();
    std::cout << model_urdf_dir_str <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(model_urdf_dir,quadruped_model_, false, false);
}

unsigned int RobotDynamics::NumofJoint()
{
//    std::cout << " the gravity is " << quadruped_model_->gravity  << std::endl;
    return quadruped_model_->dof_count;
}

RobotDynamics::~RobotDynamics()
{
    std::cout << "delete the robot dynamics" << std::endl;
}

RigidBodyDynamics::Model* RobotDynamics::GetModel()
{
    return quadruped_model_;
}


}//namespace quadruped_model

int main(int argc, char *argv[])
{
    rbdl_check_api_version (RBDL_API_VERSION);
    quadruped_model::RobotDynamics robot_dynamics_test;
    VectorNd Q = VectorNd::Zero(robot_dynamics_test.NumofJoint());
    VectorNd QDot = VectorNd::Zero(robot_dynamics_test.NumofJoint());
    VectorNd Tau = VectorNd::Zero(robot_dynamics_test.NumofJoint());
    VectorNd QDDot = VectorNd::Zero(robot_dynamics_test.NumofJoint());

    InverseDynamics(*robot_dynamics_test.GetModel(), Q, QDot, Tau, QDDot);
    std::cout << robot_dynamics_test.NumofJoint() << std::endl;

    std::cout << Tau << std::endl;

//    std::cout << robot_dynamics_test.NumofJoint() << std::endl;

    return 0;
}



