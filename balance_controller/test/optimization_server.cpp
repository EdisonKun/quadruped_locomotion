#include "free_gait_msgs/optimize.h"
#include "ros/ros.h"
#include "quadruped_model_CppAD/Quadruped_optimization.h"

bool robot_state_callback(free_gait_msgs::optimize::Request&  req,
                          free_gait_msgs::optimize::Response& res)
{
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t nx = 27;
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

    //avoid the singlarity point;
    xu[0] = 0.65; xl[0] = -xu[0];
    xu[1] = 2;    xl[1] = -xu[1];
    xu[2] = -0.1; xl[2] = -3;
    xu[3] = 0.65; xl[3] = -xu[3];
    xu[4] = 2;    xl[4] = -xu[4];
    xu[5] = 3;    xl[5] = 0.1;
    xu[6] = 0.65; xl[6] = -xu[6];
    xu[7] = 2;    xl[7] = -xu[7];
    xu[8] = -0.1; xl[8] = -3;
    xu[9] = 0.65; xl[9] = -xu[9];
    xu[10] = 2;   xl[10]= - xu[10];
    xu[11] = 3;   xl[11]= 0.1;
    for (unsigned int i = 12; i < 24; i++) {
        xu[i] = 65;
        xl[i] = - xu[i];
        xi[i] = 2;
    }

    size_t con_num = 10 + 12;//constraint number
    Dvector gl(con_num), gu(con_num);
    gl[0] = req.desired_force.wrench.force.x; gu[0] = gl[0];
    gl[1] = req.desired_force.wrench.force.y; gu[1] = gl[1];
    gl[2] = req.desired_force.wrench.force.z; gu[2] = gl[2];//force in the z direction;
    gl[3] = req.desired_force.wrench.torque.x;gu[3] = gl[3];//roll
    gl[4] = req.desired_force.wrench.torque.y;gu[4] = gl[4];//yaw
    gl[5] = req.desired_force.wrench.torque.z;gu[5] = gl[5];//pitch
    std::cout << " the constraint force is " << std::endl;
    for (unsigned int i = 0; i < 6; i++) {
        std::cout << gl[i] << std::endl;
    }

    if(gl[2] < 0)
    {
        gl[6] = -1.0e5;gu[6] = 0;//force direction
        gl[7] = -1.0e5;gu[7] = 0;
        gl[8] = -1.0e5;gu[8] = 0;
        gl[9] = -1.0e5;gu[9] = 0;
    }else {
        gl[6] = 0;gu[6] = 1.0e5;//force direction
        gl[7] = 0;gu[7] = 1.0e5;
        gl[8] = 0;gu[8] = 1.0e5;
        gl[9] = 0;gu[9] = 1.0e5;
    }

    //calculate the current foot position in the world frame;


    iit::simpledog::JointState joint_angles;
    for (unsigned int i = 0; i < 12; i++) {
        joint_angles(i) = xi[i];
    }

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

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    ADvector x_veri(nx);

    for (unsigned int i = 0; i < nx; i++) {
        x_veri[i] = xi[i];
    }
    quadkin.Angles_Torques_Initial(x_veri);

    quadruped_model::Pose_cppad basepose_cppad;
    basepose_cppad.getPosition() << req.robot_state.base_pose.pose.pose.position.x,
        req.robot_state.base_pose.pose.pose.position.y,
        req.robot_state.base_pose.pose.pose.position.z;
    basepose_cppad.getRotation().setIdentity();
    quadkin.SetBaseInWorld(basepose_cppad);


    std::cout << " before set the base position. the foot positin is ........." << std::endl;
    quadkin.CppadPositionPrintf(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG));

    gl[10] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).x());gu[10] = gl[10];
    gl[11] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).y());gu[11] = gl[11];
    gl[12] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).z());gu[12] = gl[12];

    gl[13] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).x());gu[13] = gl[13];
    gl[14] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).y());gu[14] = gl[14];
    gl[15] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).z());gu[15] = gl[15];

    gl[16] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RH_LEG).x());gu[16] = gl[16];
    gl[17] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RH_LEG).y());gu[17] = gl[17];
    gl[18] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RH_LEG).z());gu[18] = gl[18];

    gl[19] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).x());gu[19] = gl[19];
    gl[20] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).y());gu[20] = gl[20];
    gl[21] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).z());gu[21] = gl[21];



    xi[24] = req.robot_state.base_pose.pose.pose.position.z;// base constraints in the z direction;
    xu[24] = 0.5; xl[24] = 0.0;

    //add base constraints in the x-y direction;
    xi[25] = req.robot_state.base_pose.pose.pose.position.x;
    xu[25] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).x());
    xl[25] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).x());

    xi[26] = req.robot_state.base_pose.pose.pose.position.y;
    xu[26] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).y());
    xl[26] = CppAD::Value(quadkin.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).y());


    FG_eval fg_eval;

    fg_eval.SetRobotState(robot_state);

    std::string options;
    // turn off any printing
    options += "Integer print_level  1\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     20000\n";
    // approximate accuracy in first order necessary conditions;
    // see Mathematical Programming, Volume 106, Number 1,
    // Pages 25-57, Equation (6)
    options += "Numeric tol          1e-3\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);

    std::cout << "********solution status********" << solution.status << std::endl;
    std::cout << "Joint angle is : " << std::endl;
    for (unsigned int i = 0; i < 12; i++) {
        std::cout.precision(4);
        std::cout.width(8);
        std::cout << solution.x[i] <<" ";
    }
    std::cout << std::endl;

    std::cout << "Joint torque is : " << std::endl;
    for (unsigned int i = 12; i < 24; i++) {
        std::cout.precision(4);
        std::cout.width(8);
        std::cout << solution.x[i] <<" ";
    }
    std::cout << std::endl;

    std::cout << "base position is " << solution.x[25] << " " << solution.x[26] << " " << solution.x[24] << std::endl;
    std::cout << "------------------*-----------------" << std::endl;

    std::shared_ptr<free_gait::State> Robot_state;
    Robot_state.reset(new free_gait::State);

    std::vector<free_gait::LimbEnum> limbs_;
    std::vector<free_gait::BranchEnum> branches_;
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);

    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);


    Robot_state->initialize(limbs_, branches_);
    Robot_state->setSupportLeg(free_gait::LimbEnum::LF_LEG, true);
    Robot_state->setSupportLeg(free_gait::LimbEnum::RF_LEG, true);
    Robot_state->setSupportLeg(free_gait::LimbEnum::RH_LEG, true);
    Robot_state->setSupportLeg(free_gait::LimbEnum::LH_LEG, true);


    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    for (unsigned int i = 0; i < nx; i++) {
        x_veri[i] = solution.x[i];
    }


    quadruped_model::Quad_Kin_CppAD quadKinCPPAD(Robot_state);
    quadKinCPPAD.PrepareLegLoading();
    quadKinCPPAD.Angles_Torques_Initial(x_veri);
    quadKinCPPAD.PrepareOptimization();
    /*****test the constraints********//*
    std::cout << " the constraints of lf foot is : " << std::endl;
    quadKinCPPAD.CppadPositionPrintf(lf_foot_position);

    std::cout << " the constraints of rf foot is : " << std::endl;
    quadKinCPPAD.CppadPositionPrintf(rf_foot_position);*/

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_, jac_;
    A_ = quadKinCPPAD.GetAMatrix();
    jac_ = quadKinCPPAD.GetFootJacobian();

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> torques;
    torques.resize(12,1);

    double torques_square;
    torques_square = 0;
    for (unsigned int i = 0; i < 12; i++) {
        torques(i,0) = x_veri[i+12];
        torques_square = torques_square + CppAD::Value(x_veri[i + 12] * x_veri[i + 12]);
    }
    std::cout << " the torques square is " << torques_square << std::endl;


    quadruped_model::Pose_cppad base_pose;
    base_pose.getPosition() << x_veri[25],x_veri[26],x_veri[24];
    base_pose.getRotation().setIdentity();
    quadKinCPPAD.SetBaseInWorld(base_pose);
    std::cout << " after set the base position. the foot position is........." << std::endl;
    quadruped_model::Position_cppad foot_position;
    foot_position = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG);// base frame is the world frame;
    quadKinCPPAD.CppadPositionPrintf(foot_position);

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> final;
    final.resize(6,1);
    final = A_ * jac_ * torques;//base force
    std::cout << "base force is :" << std::endl;
    quadKinCPPAD.EigenMatrixPrintf(final.transpose());

    std::cout << "foot force is : " << std::endl;
    final = jac_ * torques;//foot force;
    quadKinCPPAD.EigenMatrixPrintf(final.transpose());
    ROS_INFO_STREAM("Success~");
    res.success = true;
    return true;

}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "optimize_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("/optimize", robot_state_callback);
    ROS_INFO_STREAM("ready to get the robot state");
    ros::spin();

    return 0;

}






