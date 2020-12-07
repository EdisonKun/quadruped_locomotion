
#include "quadruped_nonlinear_test.hpp"

namespace ifopt{

ExConstraint::ExConstraint(): ExConstraint("constraint1"){}

ExConstraint::ExConstraint(const std::string& name) : ConstraintSet(1, name) {}

VectorXd ExConstraint::GetValues() const
{
    VectorXd g(GetValues());
    VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();


}

VecBound ExConstraint::GetBounds() const
{

}

void ExConstraint::FillJacobianBlock(std::string var_set, Jacobian &jac_block) const
{

}


}//namespace
