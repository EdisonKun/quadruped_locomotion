#include "iostream"
#include "cppad/ipopt/solve.hpp"
#include "cppad/example/cppad_eigen.hpp"
#include <Eigen/LU>

#include "HyQ/rcg/jacobians.h"
#include "HyQ/rcg/rbd_types.h"

#include "iit/robots/quadruped_model/rbd_types.h"
#include "iit/robots/quadruped_model/jacobians.h"
#include "iit/robots/quadruped_model/transforms.h"
#include "iit/rbd/robcogen_commons.h"

#include "quadruped_model/quadrupedkinematics.h"
#include "HyQ/rcg/miscellaneous.h"

namespace  {
using CppAD::AD;
class FG_eval{
public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    typedef HyQ::rcg::Matrix<3,3> Matrix33;
    typedef HyQ::rcg::Matrix<6,3> Matrix63;
    void operator()(ADvector& fg, const ADvector& x)
    {
        std::cout << "fg.size is " << fg.size() << std::endl;
        iit::simpledog::JointState angles,torques;
        iit::simpledog::Jacobians jac_quad;

        for (unsigned int i = 0; i < x.size(); i++) {
            angles[i] = x[i];
            torques[i] = x[i + 3];
        }

        fg[0] = torques[0] * torques[0] + torques[1] * torques[1] + torques[2] * torques[2];

        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> pos_jac, pos33, foot_force;
        pos_jac.resize(6,3);
        pos_jac = jac_quad.fr_base_J_LF_FOOT.update(angles);

        pos33.resize(3,3);
        pos33.setIdentity();

        pos33 = pos_jac.block(3,0,3,3);


        foot_force.resize(3,1);
        foot_force.setZero();
        std::cout << "foot force is " << foot_force(1,0) << foot_force(0,2) << foot_force(0,12) << std::endl;
        std::cout << "foot force size is " << foot_force.size() << std::endl;
        foot_force = pos33.transpose().inverse() * torques.block(0,0,3,1);

        std::cout << "foot force is " << foot_force(1,0) << foot_force(0,2) << std::endl;
        fg[1] = foot_force(0,0);
        fg[2] = foot_force(1,0);
        fg[3] = foot_force(2,0);


        std::cout << "torque is " << std::endl;
        for (unsigned int i = 0; i < torques.block(0,0,3,1).rows(); i++) {
            for (unsigned int j = 0; j < torques.block(0,0,3,1).cols(); j++) {
                std::cout << torques.block(0,0,3,1)(i,j) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "the inverse is " << std::endl;
        pos33 = pos33.transpose().inverse();
        for (unsigned int i = 0; i < pos33.rows(); i++) {
            for (unsigned int j = 0; j < pos33.cols(); j++) {
                std::cout << pos33(i,j) << " ";
            }
            std::cout << std::endl;
        }


    }
};
}


int main(int argc, char *argv[])
{
    typedef CPPAD_TESTVECTOR(double) Dvector;
    size_t nx = 6;
    Dvector xi(nx);
    Dvector xl(nx),xu(nx);
    for (unsigned int i = 0; i < nx; i++) {
        xi[i] = 0.2;
        if(i < nx/2)
        {
            xu[i] = 1;
            xl[i] = 0.1;
        }else {
            xu[i] = 65;
            xl[i] = - xu[i];
        }

    }
    size_t con_num = 1;
    Dvector gl(con_num), gu(con_num);
    for (unsigned int i = 0; i < con_num; i++) {
        gl[i] = 10;
        gu[i] = 10;
    }
    gl[2] = 30;gu[2] = 30;

    FG_eval fg_eval;

    std::string options;
    // turn off any printing
    options += "Integer print_level  1\n";
    options += "String  sb           yes\n";
    // maximum number of iterations
    options += "Integer max_iter     2000\n";
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

    return 0;
}

//#include "iostream"
//#include "cppad/ipopt/solve.hpp"
//#include "cppad/example/cppad_eigen.hpp"
//#include "Eigen/LU"

//#include "iit/robots/quadruped_model/rbd_types.h"
//#include "iit/robots/quadruped_model/jacobians.h"

//namespace  {
//using CppAD::AD;
//class FG_eval{
//public:
//    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
//    void operator()(ADvector& fg, const ADvector& x)
//    {
//        std::cout << "fg.size is " << fg.size() << std::endl;

//        AD<double> x1 = x[0];
//        AD<double> x2 = x[1];
//        AD<double> x3 = x[2];

//        fg[0] = x1 * x1 + x2 * x2;

//        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A;
//        A.resize(2,2);
//        A(0,0) = CppAD::cos(x1 * x2 * x3);
//        A(0,1) = 2;
//        A(1,0) = CppAD::sin(x2);
//        A(1,1) = CppAD::AD<double>(x1);

//        iit::simpledog::Matrix<2,2> matrix_test;
//        matrix_test = A;
//        matrix_test.inverse();

//        //        std::cout << "A.inverse is " << A.inverse() << std::endl;

//        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> X;
//        X.resize(2,1);
//        X(0,0) = x[0];
//        X(1,0) = x[1];

//        Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> result;
//        result.resize(2,1);
////        result = A.inverse() * X;
//        result = A * X;

//        fg[1] = result(0,0);
//        fg[2] = result(1,0);

//        iit::simpledog::ScalarTraits::sin(x1);
////        AD<double> x1221 = scalartraits::sin(x1);

//        /**
//              work normal **/
////        fg[1] = cos(x1 * x2 * x3) * x1 + 2 * x2;
////        fg[2] = sin(x2) * x1 + x1 * x2;
//    }
//};
//}

//int main(int argc, char *argv[])
//{
//    typedef CPPAD_TESTVECTOR(double) Dvector;
//    size_t nx = 3;
//    Dvector xi(nx);
//    Dvector xl(nx),xu(nx);
//    for (unsigned int i = 0; i < nx; i++) {
//        xi[i] = 0.2;
//        xu[i] = 5;
//        xl[i] = -xu[i];

//    }//initial value, and minimum angle -1, minimum torque -65;

//    size_t con_num = 2;
//    Dvector gl(con_num), gu(con_num);
//    for (unsigned int i = 0; i < con_num; i++) {
//        gl[i] = 10;
//        gu[i] = 10;
//    }

//    FG_eval fg_eval;

//    std::string options;
//    // turn off any printing
//    options += "Integer print_level  0\n";
//    options += "String  sb           yes\n";
//    // maximum number of iterations
//    options += "Integer max_iter     2000\n";
//    // approximate accuracy in first order necessary conditions;
//    // see Mathematical Programming, Volume 106, Number 1,
//    // Pages 25-57, Equation (6)
//    options += "Numeric tol          1e-6\n";
//    // derivative testing
//    options += "String  derivative_test            second-order\n";
//    // maximum amount of random pertubation; e.g.,
//    // when evaluation finite diff
//    options += "Numeric point_perturbation_radius  0.\n";

//    CppAD::ipopt::solve_result<Dvector> solution;

//    CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);

//    std::cout << "solution status is " << solution.status << std::endl;
//    for (unsigned int i = 0; i < 3; i++) {
//        std::cout << solution.x[i] << std::endl;
//    }

//    return 0;
//}

