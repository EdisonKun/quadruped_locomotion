
//test the ifopt
#include "ifopt/variable_set.h"
#include "ifopt/constraint_set.h"
#include "ifopt/cost_term.h"

namespace ifopt{
using Eigen::Vector2d;

class ExVariables : public VariableSet{
public:
    ExVariables() : ExVariables("var_set1"){}
    ExVariables(const std::string& name) : VariableSet(2,name)
    {
        x0_ = 2;
        x1_ = 2;
    }

    void SetVariables(const VectorXd& x) override
    {
        x0_ = x(0);
        x1_ = x(1);
    }

    VectorXd GetValues() const override
    {
        return Vector2d(x0_,x1_);
    }

    VecBound GetBounds() const override
    {
        VecBound bounds(GetRows());
        bounds.at(0) = Bounds(-1.0,0.0);
        bounds.at(1) = NoBound;
        return bounds;
    }
private:
    double x0_,x1_;
};//class EcVariables

class ExConstraint : public ConstraintSet{
public:
    ExConstraint() : ExConstraint("constraint1"){}
    ExConstraint(const std::string& name) : ConstraintSet(1, name) {}

    VectorXd GetValues() const override
    {
        VectorXd g(GetRows());
        Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
        g(0) = std::pow(x(0),2) + x(1);
        return g;
    }

    VecBound GetBounds() const override
    {
        VecBound b(GetRows());
        b.at(0) = Bounds(2.0,2.0);
        return b;
    }

    void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override
    {
        if(var_set == "var_set1"){
            Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
            jac_block.coeffRef(0,0) = 2.0 * x(0);
            jac_block.coeffRef(0,1) = 1.0;
        }
    }

};//class Exconstraint

class ExCost : public CostTerm{
public:
    ExCost() : ExCost("cost_term1") {}
    ExCost(const std::string& name) : CostTerm(name){}

    double GetCost() const override
    {
        Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();
        return -std::pow(x(1)-2,2);
    }

    void FillJacobianBlock(std::string var_set, Jacobian& jac) const override
    {
        if (var_set == "var_set1"){
            Vector2d x = GetVariables()->GetComponent("var_set1")->GetValues();

            jac.coeffRef(0,0) = 0.0;
            jac.coeffRef(0,1) = -2.0 * (x(1)-2.0);
        }
    }
};//class excost
}//namespace




