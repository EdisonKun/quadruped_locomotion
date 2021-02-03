#include "free_gait_msgs/optimize.h"
#include "ros/ros.h"
#include "quadruped_model_CppAD/Quadruped_optimization.h"

bool robot_state_callback(free_gait_msgs::optimize::Request&  req,
                          free_gait_msgs::optimize::Response& res)
{

    std::shared_ptr<free_gait::State> robot_state;
    robot_state.reset(new free_gait::State);
    /**
     * @brief Convert msgs to robot_state;
     * @return
     */
    Pose current_pose_in_world;
    current_pose_in_world.getPosition().x() = req.robot_state.base_pose.pose.pose.position.x;
    current_pose_in_world.getPosition().y() = req.robot_state.base_pose.pose.pose.position.y;
    current_pose_in_world.getPosition().z() = req.robot_state.base_pose.pose.pose.position.z;

    current_pose_in_world.getRotation().w() = req.robot_state.base_pose.pose.pose.orientation.w;
    current_pose_in_world.getRotation().x() = req.robot_state.base_pose.pose.pose.orientation.x;
    current_pose_in_world.getRotation().y() = req.robot_state.base_pose.pose.pose.orientation.y;
    current_pose_in_world.getRotation().z() = req.robot_state.base_pose.pose.pose.orientation.z;

    robot_state->setPoseBaseToWorld(current_pose_in_world);

    std::vector<quadruped_model::JointTorquesLimb> joint_torques;
    joint_torques.resize(4);

    joint_torques[0](0) = req.robot_state.lf_leg_joints.effort[0];
    joint_torques[0](1) = req.robot_state.lf_leg_joints.effort[1];
    joint_torques[0](2) = req.robot_state.lf_leg_joints.effort[2];
    robot_state->setJointEffortsForLimb(static_cast<free_gait::LimbEnum>(0),joint_torques[0]);

    joint_torques[1](0) = req.robot_state.rf_leg_joints.effort[0];
    joint_torques[1](1) = req.robot_state.rf_leg_joints.effort[1];
    joint_torques[1](2) = req.robot_state.rf_leg_joints.effort[2];
    robot_state->setJointEffortsForLimb(static_cast<free_gait::LimbEnum>(1),joint_torques[1]);

    joint_torques[2](0) = req.robot_state.rh_leg_joints.effort[0];
    joint_torques[2](1) = req.robot_state.rh_leg_joints.effort[1];
    joint_torques[2](2) = req.robot_state.rh_leg_joints.effort[2];
    robot_state->setJointEffortsForLimb(static_cast<free_gait::LimbEnum>(2),joint_torques[2]);

    joint_torques[3](0) = req.robot_state.lh_leg_joints.effort[0];
    joint_torques[3](1) = req.robot_state.lh_leg_joints.effort[1];
    joint_torques[3](2) = req.robot_state.lh_leg_joints.effort[2];
    robot_state->setJointEffortsForLimb(static_cast<free_gait::LimbEnum>(3),joint_torques[3]);

    std::vector<quadruped_model::JointPositionsLimb> joint_positions;
    joint_positions.resize(4);

    joint_positions[0](0) = req.robot_state.lf_leg_joints.position[0];
    joint_positions[0](1) = req.robot_state.lf_leg_joints.position[1];
    joint_positions[0](2) = req.robot_state.lf_leg_joints.position[2];
    robot_state->setJointPositionsForLimb(static_cast<free_gait::LimbEnum>(0),joint_positions[0]);

    joint_positions[1](0) = req.robot_state.rf_leg_joints.position[0];
    joint_positions[1](1) = req.robot_state.rf_leg_joints.position[1];
    joint_positions[1](2) = req.robot_state.rf_leg_joints.position[2];
    robot_state->setJointPositionsForLimb(static_cast<free_gait::LimbEnum>(1),joint_positions[1]);

    joint_positions[2](0) = req.robot_state.rh_leg_joints.position[0];
    joint_positions[2](1) = req.robot_state.rh_leg_joints.position[1];
    joint_positions[2](2) = req.robot_state.rh_leg_joints.position[2];
    robot_state->setJointPositionsForLimb(static_cast<free_gait::LimbEnum>(2),joint_positions[2]);

    joint_positions[3](0) = req.robot_state.lh_leg_joints.position[0];
    joint_positions[3](1) = req.robot_state.lh_leg_joints.position[1];
    joint_positions[3](2) = req.robot_state.lh_leg_joints.position[2];
    robot_state->setJointPositionsForLimb(static_cast<free_gait::LimbEnum>(3),joint_positions[3]);

    for (unsigned int i = 0; i < 4; i++) {
        robot_state->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
    }

    quadruped_model::Quad_Kin_CppAD quadkin(robot_state);

    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t nx = 24;
    Dvector xi(nx);
    Dvector xl(nx),xu(nx);

    xi[0] = req.robot_state.lf_leg_joints.position[0];
    xi[1] = req.robot_state.lf_leg_joints.position[1];
    xi[2] = req.robot_state.lf_leg_joints.position[2];

    xi[3] = req.robot_state.rf_leg_joints.position[0];
    xi[4] = req.robot_state.rf_leg_joints.position[1];
    xi[5] = req.robot_state.rf_leg_joints.position[2];

    xi[6] = req.robot_state.rh_leg_joints.position[0];
    xi[7] = req.robot_state.rh_leg_joints.position[1];
    xi[8] = req.robot_state.rh_leg_joints.position[2];

    xi[9] = req.robot_state.lh_leg_joints.position[0];
    xi[10] = req.robot_state.lh_leg_joints.position[1];
    xi[11] = req.robot_state.lh_leg_joints.position[2];

    xi[12] = req.robot_state.lf_leg_joints.effort[0];
    xi[13] = req.robot_state.lf_leg_joints.effort[1];
    xi[14] = req.robot_state.lf_leg_joints.effort[2];

    xi[15] = req.robot_state.rf_leg_joints.effort[0];
    xi[16] = req.robot_state.rf_leg_joints.effort[1];
    xi[17] = req.robot_state.rf_leg_joints.effort[2];

    xi[18] = req.robot_state.rh_leg_joints.effort[0];
    xi[19] = req.robot_state.rh_leg_joints.effort[1];
    xi[20] = req.robot_state.rh_leg_joints.effort[2];

    xi[21] = req.robot_state.lh_leg_joints.effort[0];
    xi[22] = req.robot_state.lh_leg_joints.effort[1];
    xi[23] = req.robot_state.lh_leg_joints.effort[2];
//    std::cout << " xi is " << xi << std::endl;

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    ADvector x_veri(nx);

    for (unsigned int i = 0; i < nx; i++) {
        x_veri[i] = xi[i];
    }
    quadkin.PrepareLegLoading();
    quadkin.Angles_Torques_Initial(x_veri);
    quadkin.PrepareOptimization();

    Eigen::MatrixXd base_force;
    base_force.resize(6,1);
    Eigen::MatrixXd joint_torque_twelve;
    joint_torque_twelve.resize(12,1);
    for (unsigned int i = 0; i < 12; i++) {
        joint_torque_twelve(i,0) = xi[12 + i];
    }
    base_force = quadkin.GetADoubleMatrix()*quadkin.GetFootDoubleJacobian()*joint_torque_twelve;

    std::cout << " the desired force is " << std::endl;
    std::cout << req.desired_force.wrench.force.x <<" " << req.desired_force.wrench.force.y <<" " << req.desired_force.wrench.force.z << " "<<
        req.desired_force.wrench.torque.x << " "<< req.desired_force.wrench.torque.y<< " " << req.desired_force.wrench.torque.z << std::endl;//pitch
    std::cout << base_force.transpose() << std::endl;
    ROS_INFO_STREAM("Success~");
    return true;

}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "optimize_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/optimize", robot_state_callback);
    ROS_INFO_STREAM("ready to get the robot state in the base force estimate");
    ros::spin();

    return 0;

}






