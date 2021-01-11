/*
 *  Quadruped_model_cppad_test.cpp
 *  Descriotion: to test the quadruped_model_cppad_lib
 *
 *  Created on: 1205, 2020
 *  Author: Edison Kun
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
// gtest
#include "quadruped_model_CppAD/Quadruped_Kin_CppAD.h"
#include "iostream"
#include "cppad/ipopt/solve.hpp"
#include "cppad/example/cppad_eigen.hpp"
#include <Eigen/LU>


namespace  {
using CppAD::AD;
class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& x)
    {
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

        free_gait::Pose base_pose;
        base_pose.getPosition() << 0,0,0;
        base_pose.getRotation().setIdentity();
        Robot_state->setPoseBaseToWorld(base_pose);

        quadruped_model::Quad_Kin_CppAD quadKinCPPAD(Robot_state);
        quadKinCPPAD.PrepareLegLoading();

        quadKinCPPAD.Angles_Torques_Initial(x);
        quadKinCPPAD.PrepareOptimization();

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_, jac_;
        A_ = quadKinCPPAD.GetAMatrix();
        jac_ = quadKinCPPAD.GetFootJacobian();

        quadKinCPPAD.EigenMatrixPrintf(A_);

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> torques;
        torques.resize(12,1);

        for (unsigned int i = 0; i < 12; i++) {
            fg[0] = fg[0] + x[i+12] * x[i+12];
            torques(i,0) = x[i+12];
        }

        CppAD::AD<double> base_z;
        base_z = x[24];

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> final;
        final.resize(6,1);
        final = A_ * jac_ * torques;//base force

        for (unsigned int i = 0; i < 6; i++) {
            fg[i + 1] = final(i,0);
        }//1~6

        //add leg force, the force in the z direction should be bigger than zero?nope, smaller than zero;
        std::vector<Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic>> foot_force;
        foot_force.resize(4);
        for (unsigned int i = 0; i < 4; i++) {
            foot_force.at(i).resize(3,1);
            foot_force.at(i) = Eigen::Matrix<CppAD::AD<double>, 12, 1>(jac_ * torques).segment(3 * i, 3 * i + 3);//all leg force;
            fg[7 + i] = foot_force.at(i)(2,0);
        }//7~10

        //add foot position constraints;

        quadruped_model::Pose_cppad basepose_cppad;
        basepose_cppad.getPosition() << 0,0,base_z;
        basepose_cppad.getRotation().setIdentity();
        quadKinCPPAD.SetBaseInWorld(basepose_cppad);

        fg[11] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).x();
        fg[12] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).y();
        fg[13] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG).z();

        fg[14] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).x();
        fg[15] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).y();
        fg[16] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::RF_LEG).z();

        fg[17] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::RH_LEG).x();
        fg[18] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::RH_LEG).y();
        fg[19] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::RH_LEG).z();

        fg[20] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).x();
        fg[21] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).y();
        fg[22] = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LH_LEG).z();

    }
};
}


int main(int argc, char *argv[])
{
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t nx = 25;
    Dvector xi(nx);
    Dvector xl(nx),xu(nx);
    xi[0] = 0;
    xi[1] = 1.4;
    xi[2] = -2.4;
    xi[3] = 0;
    xi[4] = -1.4;
    xi[5] = 2.4;
    xi[6] = 0;
    xi[7] = 1.4;
    xi[8] = -2.4;
    xi[9] = 0;
    xi[10] = -1.4;
    xi[11] = 2.4;

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

    xi[24] = 0.0;
    xu[24] = 0.3; xl[24] = 0.0;


    size_t con_num = 10 + 12;//constraint number
    Dvector gl(con_num), gu(con_num);
    for (unsigned int i = 0; i < 2; i++) {
        gl[i] = -10;
        gu[i] = 10;
    }
    gl[2] = -600;gu[2] = -600;//force in the z direction;
    gl[3] = 0;gu[3] = 0;//roll
    gl[4] = 0;gu[4] = 0;//yaw
    gl[5] = 0;gu[5] = 0;//pitch

    gl[6] = -1.0e5;gu[6] = 0;//force direction
    gl[7] = -1.0e5;gu[7] = 0;
    gl[8] = -1.0e5;gu[8] = 0;
    gl[9] = -1.0e5;gu[9] = 0;

//        gl[6] = 0;gu[6] = 1.0e5;//force direction
//        gl[7] = 0;gu[7] = 1.0e5;
//        gl[8] = 0;gu[8] = 1.0e5;
//        gl[9] = 0;gu[9] = 1.0e5;

    //calculate the current foot position in the world frame;


    iit::simpledog::JointState joint_angles;
    joint_angles << 0, 1.4, -2.4, 0, -1.4, 2.4, 0, 1.4, -2.4, 0, -1.4, 2.4;
    iit::simpledog::HomogeneousTransforms motion_trans;

    quadruped_model::Position_cppad lf_foot_position;
    lf_foot_position.vector() = motion_trans.fr_base_X_LF_FOOT(joint_angles).block(0,3,3,1);
    gl[10] = CppAD::Value(lf_foot_position.x()); gu[10] = CppAD::Value(lf_foot_position.x());
    gl[11] = CppAD::Value(lf_foot_position.y()); gu[11] = CppAD::Value(lf_foot_position.y());
    gl[12] = CppAD::Value(lf_foot_position.z()); gu[12] = CppAD::Value(lf_foot_position.z());

    quadruped_model::Position_cppad rf_foot_position;
    rf_foot_position.vector() = motion_trans.fr_base_X_RF_FOOT(joint_angles).block(0,3,3,1);
    gl[13] = CppAD::Value(rf_foot_position.x());gu[13] = CppAD::Value(rf_foot_position.x());
    gl[14] = CppAD::Value(rf_foot_position.y());gu[14] = CppAD::Value(rf_foot_position.y());
    gl[15] = CppAD::Value(rf_foot_position.z());gu[15] = CppAD::Value(rf_foot_position.z());


    quadruped_model::Position_cppad rh_foot_position;
    rh_foot_position.vector() = motion_trans.fr_base_X_RH_FOOT(joint_angles).block(0,3,3,1);
    gl[16] = CppAD::Value(rh_foot_position.x());gu[16] = CppAD::Value(rh_foot_position.x());
    gl[17] = CppAD::Value(rh_foot_position.y());gu[17] = CppAD::Value(rh_foot_position.y());
    gl[18] = CppAD::Value(rh_foot_position.z());gu[18] = CppAD::Value(rh_foot_position.z());

    quadruped_model::Position_cppad lh_foot_position;
    lh_foot_position.vector() = motion_trans.fr_base_X_LH_FOOT(joint_angles).block(0,3,3,1);
    gl[19] = CppAD::Value(lh_foot_position.x());gu[19] = CppAD::Value(lh_foot_position.x());
    gl[20] = CppAD::Value(lh_foot_position.y());gu[20] = CppAD::Value(lh_foot_position.y());
    gl[21] = CppAD::Value(lh_foot_position.z());gu[21] = CppAD::Value(lh_foot_position.z());

    FG_eval fg_eval;

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

    std::cout << "base position is " << solution.x[24] << std::endl;
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
    ADvector x_veri(25);

    for (unsigned int i = 0; i < nx; i++) {
        x_veri[i] = solution.x[i];
    }


    quadruped_model::Quad_Kin_CppAD quadKinCPPAD(Robot_state);
    quadKinCPPAD.PrepareLegLoading();
    quadKinCPPAD.Angles_Torques_Initial(x_veri);
    quadKinCPPAD.PrepareOptimization();
    /*****test the constraints********/
    std::cout << " the constraints of lf foot is : " << std::endl;
    quadKinCPPAD.CppadPositionPrintf(lf_foot_position);

    std::cout << " the constraints of rf foot is : " << std::endl;
    quadKinCPPAD.CppadPositionPrintf(rf_foot_position);

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_, jac_;
    A_ = quadKinCPPAD.GetAMatrix();
    jac_ = quadKinCPPAD.GetFootJacobian();

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> torques;
    torques.resize(12,1);

    for (unsigned int i = 0; i < x_veri.size()/2; i++) {
        torques(i,0) = x_veri[i+12];
    }

    std::cout << " before set the base position>........." << std::endl;
    quadruped_model::Position_cppad foot_position;
    foot_position = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG);// base frame is the world frame;
    quadKinCPPAD.CppadPositionPrintf(foot_position);

    std::cout << " after set the base position>........." << std::endl;
    quadruped_model::Pose_cppad base_pose;
    base_pose.getPosition() << 0,0,x_veri[24];
    base_pose.getRotation().setIdentity();
    quadKinCPPAD.SetBaseInWorld(base_pose);
    foot_position = quadKinCPPAD.GetFootPositionInWorldframe(free_gait::LimbEnum::LF_LEG);//base frame is above the world frame; unchanged position;
    quadKinCPPAD.CppadPositionPrintf(foot_position);

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> final;
    final.resize(6,1);
    final = A_ * jac_ * torques;//base force
    std::cout << "base force is :" << std::endl;
    quadKinCPPAD.EigenMatrixPrintf(final.transpose());

    std::cout << "foot force is : " << std::endl;
    final = jac_ * torques;//foot force;
    quadKinCPPAD.EigenMatrixPrintf(final.transpose());

    return 0;
}
