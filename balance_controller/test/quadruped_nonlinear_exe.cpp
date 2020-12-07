#include "ifopt/problem.h"
#include "iostream"
#include "ifopt/ipopt_solver.h"

#include "quadruped_nonlinear_test.hpp"

using namespace ifopt;
int main(int argc, char *argv[])
{
    Problem nlp;
    nlp.AddVariableSet(std::make_shared<Ex_twelve_Variables>());
//    nlp.AddConstraintSet(std::make_shared<ExConstraint>());
//    nlp.AddCostSet(std::make_shared<ExCost>());
//    nlp.PrintCurrent();

//    IpoptSolver ipopt;
//    ipopt.SetOption("linear_solver", "mumps");
//    ipopt.SetOption("jacobian_approximation","exact");

//    ipopt.Solve(nlp);
//    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
//    std::cout << x.transpose() << std::endl;

//    SnoptSolver solver;
//    solver.Solve(nlp);

//    std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
    return 0;
}

