/**
This file is to test the unlinear constraints of the quadruped robot.
the variable are q1...q12, tau1...tau12;
*/

//#pragma once

#include "ifopt/variable_set.h"
#include "ifopt/constraint_set.h"
#include "ifopt/cost_term.h"
#include "iostream"
#include "iit/robots/quadruped_model/declarations.h"
#include "iit/robots/quadruped_model/jacobians.h"
namespace ifopt{

using VectorXd = Eigen::VectorXd;
using VecBound = std::vector<Bounds>;

class ExConstraint : public ConstraintSet{
public:
    ExConstraint();
    ExConstraint(const std::string& name);

    /**
     * @brief GetValues
     * @return get the values the constraints
     */
    VectorXd GetValues() const override;

    VecBound GetBounds() const override;

    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

private:


};//class Exconstraint


class Ex_twelve_Variables : public VariableSet{
public:
    Ex_twelve_Variables() : Ex_twelve_Variables("var_set1"){}
    Ex_twelve_Variables(const std::string& name) : VariableSet(24,name)
    {
        AandT_varibles.resize(24);
        AandT_varibles.setZero();
    }

    void SetVariables(const VectorXd& x) override
    {
        for (unsigned int i = 0; i < x.size(); i++) {
            AandT_varibles[i] = x[i];
        }
    }

    VectorXd GetValues() const override
    {
        return VectorXd(AandT_varibles);
    }

    VecBound GetBounds() const override
    {
        VecBound bounds(GetRows());
        double joint_lower_limit = -2.0;
        double joint_upper_limit = 2.0;
        std::cout << "the bounds size is " << bounds.size() << std::endl;
        for (unsigned int i = 0; i < 12; i++) {
            bounds.at(i) = Bounds(joint_lower_limit, joint_upper_limit);
        }

        double torque_limit = 65;
        for (unsigned int i = 12; i < GetRows(); i++) {
            bounds.at(i) = Bounds(-torque_limit, torque_limit);
        }
        return bounds;
    }
private:
    VectorXd AandT_varibles;
};//class EcVariables


class ExCost : public CostTerm{
public:
    ExCost() : ExCost("cost_term1") {}
    ExCost(const std::string& name) : CostTerm(name){}

    double GetCost() const override
    {
        VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
        double cost = 0;
        for (unsigned int i = 12; i < GetRows(); i++) {
            cost = cost + std::pow(x(i),2);
        }
        return cost;
    }

    void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_set1"){
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

            for (unsigned int i = 0; i < 12; i++) {
                jac.coeffRef(0,i) = 0.0;
            }

            for (unsigned int i = 12; i < GetRows(); i++) {
                jac.coeffRef(0,i) = 2 * x(i);
            }
        }
    }
};//class excost
}//namespace




