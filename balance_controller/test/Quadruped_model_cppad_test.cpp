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

        free_gait::Pose base_pose;
        base_pose.getPosition() << 0,0,0;
        base_pose.getRotation().setIdentity();
        Robot_state->setPoseBaseToWorld(base_pose);

        quadruped_model::Quad_Kin_CppAD quadKinCPPAD(Robot_state);
        quadKinCPPAD.PrepareLegLoading();

        quadKinCPPAD.Angles_Torques_Initial(x);
        quadKinCPPAD.Store_Foot_Position();
        quadKinCPPAD.PrepareOptimization();

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_, jac_;
        A_ = quadKinCPPAD.GetAMatrix();
        jac_ = quadKinCPPAD.GetFootJacobian();

        std::cout << "matrix A is " << std::endl;
        for (unsigned int i = 0; i < A_.rows();i++) {
            for (unsigned int j = 0; j < A_.cols(); j++) {
                std::cout << A_(i,j) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "jac_ is " << std::endl;
        for (unsigned int i = 0; i < jac_.rows();i++) {
            for (unsigned int j = 0; j < jac_.cols(); j++) {
                std::cout << jac_(i,j) << " ";
            }
            std::cout << std::endl;
        }

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> torques;
        torques.resize(12,1);

        for (unsigned int i = 0; i < x.size()/2; i++) {
            fg[0] = fg[0] + x[i+12] * x[i+12];
            torques(i,0) = x[i+12];
        }
        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> final;
        final.resize(6,1);
        std::cout << "calculate the final" <<std::endl;
        final = A_ * jac_ * torques;
        std::cout << "final is " << std::endl;
        for (unsigned int i = 0; i < final.rows();i++) {
            for (unsigned int j = 0; j < final.cols(); j++) {
                std::cout << final(i,j) << " ";
            }
            std::cout << std::endl;
        }

        for (unsigned int i = 0; i < 6; i++) {
            fg[i + 1] = final(i,0);
        }

    }
};
}


int main(int argc, char *argv[])
{
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t nx = 24;
    Dvector xi(nx);
    Dvector xl(nx),xu(nx);
    for (unsigned int i = 0; i < nx; i++) {
        xi[i] = 0.1;
        if(i < nx/2)
        {
            xu[0] = 0.7; xl[0] = -xu[0];
            xu[1] = 1; xl[1] = -xu[1];
            xu[2] = 1.2; xl[2] = -xu[2];
            xu[3] = 0.7; xl[3] = -xu[3];
            xu[4] = 1; xl[4] = -xu[4];
            xu[5] = 1.2; xl[5] = -xu[5];
            xu[6] = 0.7; xl[6] = -xu[6];
            xu[7] = 1; xl[7] = -xu[7];
            xu[8] = 1.2; xl[8] = -xu[8];
            xu[9] = 0.7; xl[9] = -xu[9];
            xu[10] = 1; xl[10] = -xu[10];
            xu[11] = 1.2; xl[11] = -xu[11];

        }else {
            xu[i] = 65;
            xl[i] = - xu[i];
        }

    }
    size_t con_num = 6;
    Dvector gl(con_num), gu(con_num);
    for (unsigned int i = 0; i < 2; i++) {
        gl[i] = -100;
        gu[i] = 100;
    }
    gl[2] = 600;gu[2] = 600;
    gl[3] = 0;gu[3] = 0;
    gl[4] = 0;gu[4] = 0;
    gl[5] = 0;gu[5] = 0;

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
    options += "Numeric tol          1e-6\n";
    // derivative testing
    options += "String  derivative_test            second-order\n";
    // maximum amount of random pertubation; e.g.,
    // when evaluation finite diff
    options += "Numeric point_perturbation_radius  0.\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);

    std::cout << "solution status is " << solution.status << std::endl;
    for (unsigned int i = 0; i < nx; i++) {
        std::cout << solution.x[i] << std::endl;
    }

    std::shared_ptr<free_gait::State> Robot_state;
    Robot_state.reset(new free_gait::State);

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


    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    ADvector x_veri(24);

    for (unsigned int i = 0; i < nx; i++) {
        x_veri[i] = solution.x[i];
    }


    quadruped_model::Quad_Kin_CppAD quadKinCPPAD(Robot_state);
    quadKinCPPAD.PrepareLegLoading();
    quadKinCPPAD.Angles_Torques_Initial(x_veri);
    quadKinCPPAD.Store_Foot_Position();
    quadKinCPPAD.PrepareOptimization();

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_, jac_;
    A_ = quadKinCPPAD.GetAMatrix();
    jac_ = quadKinCPPAD.GetFootJacobian();

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> torques;
    torques.resize(12,1);

    for (unsigned int i = 0; i < x_veri.size()/2; i++) {
        torques(i,0) = x_veri[i+12];
    }
    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> final;
    final.resize(6,1);
    std::cout << "calculate the final" <<std::endl;
    final = A_ * jac_ * torques;
    std::cout << "final is " << std::endl;
    for (unsigned int i = 0; i < final.rows();i++) {
        for (unsigned int j = 0; j < final.cols(); j++) {
            std::cout << final(i,j) << " ";
        }
        std::cout << std::endl;
    }

    final = jac_ * torques;
    std::cout << "final 2 is " << std::endl;
    for (unsigned int i = 0; i < final.rows();i++) {
        for (unsigned int j = 0; j < final.cols(); j++) {
            std::cout << final(i,j) << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}
