
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
#include <cmath>


namespace  {
using CppAD::AD;
class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& x)
    {
        quadruped_model::Quad_Kin_CppAD quadKinCPPAD(Robot_state);
        quadKinCPPAD.PrepareLegLoading();

        quadKinCPPAD.Angles_Torques_Initial(x);
        quadKinCPPAD.PrepareOptimization();

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_, jac_;
        A_ = quadKinCPPAD.GetAMatrix();
        jac_ = quadKinCPPAD.GetFootJacobian();


        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> torques;
        torques.resize(12,1);
        double w;
        w=0.5;

        for (unsigned int i = 0; i < 12; i++) {
            fg[0] = fg[0] + x[i+12] * x[i+12];
            torques(i,0) = x[i+12];
        }
        double s;
        s = 1;

        CppAD::AD<double> base_x, base_y, base_z;
        base_z = x[24];
        base_x = x[25];
        base_y = x[26];

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> final;
        final.resize(6,1);
        final = A_ * jac_ * torques;//base force

        for (unsigned int i = 0; i < 6; i++) {
            fg[i + 1] = final(i,0);
        }//1~6

        //add leg force, the force in the z direction should be bigger than zero?nope, smaller than zero;
        std::vector<Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic>> foot_force;
        foot_force.resize(4);

        int support_leg;
        support_leg = 4;

        Eigen::Matrix3d foot_force_aver;
        foot_force_aver(0,0) = base_force(0,0);
        foot_force_aver(1,0) = base_force(1,0);
        foot_force_aver(2,0) = base_force(2,0);

        foot_force_aver = foot_force_aver / 3.0;
        std::cout << " fooooooot force average is " << foot_force_aver(0,0) << " " << foot_force_aver(1,0) << " " << foot_force_aver(2,0) << std::endl;

        ADvector foot_force_advector;
        foot_force_advector.resize(12);
        for (unsigned int i = 0; i < 12; i++) {
            foot_force_advector[i] = (jac_ * torques)(i,0);
        }

        fg[0] = fg[0] + s* ((foot_force_advector[0] - foot_force_aver(0,0)) * (foot_force_advector[0] - foot_force_aver(0,0))
                             + (foot_force_advector[1] - foot_force_aver(1,0)) * (foot_force_advector[1] - foot_force_aver(1,0))
                             + (foot_force_advector[2] - foot_force_aver(2,0)) * (foot_force_advector[2] - foot_force_aver(2,0))
                             + (foot_force_advector[3] - foot_force_aver(0,0)) * (foot_force_advector[3] - foot_force_aver(0,0))
                             + (foot_force_advector[4] - foot_force_aver(1,0)) * (foot_force_advector[4] - foot_force_aver(1,0))
                             + (foot_force_advector[5] - foot_force_aver(2,0)) * (foot_force_advector[5] - foot_force_aver(2,0))
                             + (foot_force_advector[6] - foot_force_aver(0,0)) * (foot_force_advector[6] - foot_force_aver(0,0))
                             + (foot_force_advector[7] - foot_force_aver(1,0)) * (foot_force_advector[7] - foot_force_aver(1,0))
                             + (foot_force_advector[8] - foot_force_aver(2,0)) * (foot_force_advector[8] - foot_force_aver(2,0))
                             + (foot_force_advector[9] - foot_force_aver(0,0)) * (foot_force_advector[9] - foot_force_aver(0,0))
                             + (foot_force_advector[10] - foot_force_aver(1,0)) * (foot_force_advector[10] - foot_force_aver(1,0))
                             + (foot_force_advector[11] - foot_force_aver(2,0)) * (foot_force_advector[11] - foot_force_aver(2,0))
                             );

        for (unsigned int i = 0; i < 4; i++) {
            foot_force.at(i).resize(3,1);
            foot_force.at(i) = Eigen::Matrix<CppAD::AD<double>, 12, 1>(jac_ * torques).segment(3 * i, 3 * i + 3);//all leg force;
            fg[7 + i] = foot_force.at(i)(2,0);
        }//7~10

        //add foot position constraints;
        std::cout << " add foot position constraints" << std::endl;
        quadruped_model::Pose_cppad basepose_cppad;
        basepose_cppad.getPosition() << base_x,base_y,base_z;
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

        std::cout << " add the x-y direction force constraints" << std::endl;
        //add the constraints of force in the x-y direction; convert it to the unequal formula;
        double friction_para = 0.4;
        fg[23] = foot_force.at(0)(2,0) * friction_para - std::fabs(CppAD::Value(foot_force.at(0)(0,0)));//lf leg - x
        fg[24] = foot_force.at(0)(2,0) * friction_para - std::fabs(CppAD::Value(foot_force.at(0)(1,0)));//lf leg - y

        fg[25] = foot_force.at(1)(2,0) * friction_para - std::fabs(CppAD::Value(foot_force.at(1)(0,0)));//rf leg - x
        fg[26] = foot_force.at(1)(2,0) * friction_para - std::fabs(CppAD::Value(foot_force.at(1)(1,0)));//rf leg - y

        std::cout << "finsih the optimization load" << std::endl;

//        fg[27] = foot_force.at(2)(2,0) * friction_para - foot_force.at(2)(0,0);//rh leg - x
//        fg[28] = foot_force.at(2)(2,0) * friction_para - foot_force.at(2)(1,0);//rh leg - y

//        fg[29] = foot_force.at(3)(2,0) * friction_para - foot_force.at(3)(0,0);//lh leg - x
//        fg[30] = foot_force.at(3)(2,0) * friction_para - foot_force.at(3)(1,0);//lh leg - y

    }

public:
    std::shared_ptr<free_gait::State> Robot_state;
    Eigen::MatrixXd base_force;
    void SetRobotState()
    {
        std::cout << " success set the robot state" << std::endl;
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
    }

    void SetRobotState(std::shared_ptr<free_gait::State>& RobotState)
    {
        std::cout << " success set the robot state" << std::endl;
        Robot_state.reset(new free_gait::State);
        Robot_state = RobotState;
    }

    void SetBaseforce(Eigen::MatrixXd& base_force_external)
    {
        base_force.resize(6,1);
        base_force = base_force_external;
    }
};
}
