#include "quadruped_model/quadruped_dynamics.hpp"
namespace quadruped_model {

RobotDynamics::RobotDynamics()
{
    quadruped_model_ = new RigidBodyDynamics::Model();
    std::string model_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model.urdf";
    char* model_urdf_dir = (char*) model_urdf_dir_str.c_str();
    std::cout << model_urdf_dir_str <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(model_urdf_dir,quadruped_model_, true, true);

//    std::string rf_leg_urdf_dir_str = ros::package::getPath("quadruped_model") + "/urdf/quadruped_model_lf_leg.urdf";
//    char* rf_leg_urdf_dir = (char*)rf_leg_urdf_dir_str.c_str();
//    RigidBodyDynamics::Addons::URDFReadFromFile(rf_leg_urdf_dir, quadruped_model_, false, false);
//    quadruped_model_->IsBodyId(0);

}

}//namespace quadruped_model

int main(int argc, char *argv[])
{
    rbdl_check_api_version (RBDL_API_VERSION);
    quadruped_model::RobotDynamics robot_dynamics_test;
    return 0;
}



