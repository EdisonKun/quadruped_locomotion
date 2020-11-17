/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Péter Fankhauser, Christian Gehring, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*!
* @file     ContactForceDistribution.cpp
* @author   Péter Fankhauser, Christian Gehring
* @date     Aug 6, 2013
* @brief
*/

#include "balance_controller/contact_force_distribution/ContactJointTorqueDistribution.hpp"
//#include "loco/common/LegLinkGroup.hpp"

#include <Eigen/Geometry>
//#include "robotUtils/math/LinearAlgebra.hpp"
//#include "sm/numerical_comparisons.hpp"
//#include "OoqpEigenInterface.hpp"
//#include "QuadraticProblemFormulation.hpp"
//#include "robotUtils/loggers/logger.hpp"

//#include "loco/temp_helpers/math.hpp"


using namespace std;
using namespace Eigen;
//using namespace sm;

namespace balance_controller {


ContactJointTorqueDistribution::ContactJointTorqueDistribution(const ros::NodeHandle& node_handle,
                                                   std::shared_ptr<free_gait::State> robot_state)
    : ContactForceDistributionBase(node_handle, robot_state),
    node_handle_(node_handle),
    footDof_(3)//torso, legs, terrain)
{
    solver_.reset(new qp_solver::QuadraticProblemSolver);
    cost_function_.reset(new qp_solver::QuadraticObjectiveFunction);
    constraints_.reset(new qp_solver::LinearFunctionConstraints);
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    for(auto leg : limbs_) {
        legInfos_[leg] = LegInfo();
    }
}

ContactJointTorqueDistribution::~ContactJointTorqueDistribution()
{

}

bool ContactJointTorqueDistribution::addToLogger()
{
    // TODO Is this already implemented somewhere else?
    //  for(const auto& leg : Legs()) {
    //    string name = "contact_force_" + legNamesShort[leg];
    //    VectorCF& contactForce = legInfos_[leg].effectiveContactForce_;
    //    addEigenVector3ToLog(contactForce, name, "N", true);
    //  }

    updateLoggerData();

    return true;
}

bool ContactJointTorqueDistribution::computeForceDistribution(
    const Force& virtualForceInBaseFrame,
    const Torque& virtualTorqueInBaseFrame)
{
    if(!checkIfParametersLoaded()) return false;

    resetOptimization();//! WSHY: reset leg contact force and object function parameters
    prepareLegLoading();

    if (nLegsInForceDistribution_ > 0)
    {
        //! Kun: calculate A, b, S, W, jacobian_matrix
        prepareOptimization(virtualForceInBaseFrame, virtualTorqueInBaseFrame);
        // TODO Move these function calls to a separate method which can be overwritten
        // to have different contact force distributions, or let them be activated via parameters.
        addMinimalForceConstraints();
        addFrictionConstraints();
        addJointMaxLimit();

        // Has to be called as last
        addDesiredLegLoadConstraints(virtualForceInBaseFrame, virtualTorqueInBaseFrame);//!Kun, calculate the force distribution without load factor;
        isForceDistributionComputed_ = solveOptimization();//!kun update the matrix C, and this with the load factor. the C is calculated;
        if (isForceDistributionComputed_)
        {
            computeJointTorques();
        }
    }
    else
    {
        // No leg is part of the force distribution
        isForceDistributionComputed_ = true;
        computeJointTorques();
    }
    if (isLogging_) updateLoggerData();
    return isForceDistributionComputed_;
}

bool ContactJointTorqueDistribution::prepareLegLoading()
{
    nLegsInForceDistribution_ = 0;

    for (auto& legInfo : legInfos_)
    {
        if (robot_state_->isSupportLeg(legInfo.first))//!Kun: check whether the leg is support leg, if true:
        {
            legInfo.second.isPartOfForceDistribution_ = true;
            legInfo.second.isLoadConstraintActive_ = false;
            legInfo.second.indexInStanceLegList_ = nLegsInForceDistribution_;//! WSHY: 0,1,2,3
            legInfo.second.startIndexInVectorX_ = legInfo.second.indexInStanceLegList_ * footDof_;//! WSHY: 0,3,6,9
            nLegsInForceDistribution_++;          
            //! WSHY: what is the useness of Load Factor
            legInfo.second.isLoadConstraintActive_ = true;
        }
        else
        {
            legInfo.second.isPartOfForceDistribution_ = false;
            legInfo.second.isLoadConstraintActive_ = false;
        }
    }

    return true;
}

bool ContactJointTorqueDistribution::prepareOptimization(
    const Force& virtualForce,
    const Torque& virtualTorque)
{
    n_ = footDof_ * nLegsInForceDistribution_;//! WSHY: rows of x, columns of A

    /*
   * Finds x that minimizes f = x' W x, such that Cx = c and d <= Dx <= f.x is the joint torque.
   */
    b_.resize(nElementsVirtualForceTorqueVector_);//! WSHY: b is 6X1
    b_.segment(0, virtualForce.toImplementation().size()) = virtualForce.toImplementation();
    b_.segment(virtualForce.toImplementation().size(), virtualTorque.toImplementation().size()) = virtualTorque.toImplementation();

    S_ = virtualForceWeights_.asDiagonal();

    A_.resize(nElementsVirtualForceTorqueVector_, n_);
    A_.setZero();
    //! WSHY: set the upper 3 rows as identity
    A_.middleRows(0, footDof_) = (Matrix3d::Identity().replicate(1, nLegsInForceDistribution_)).sparseView();//copy 1-n support legs

    //calculate the jacobian
    jacobian_matrix.resize(n_,n_);
    jacobian_matrix_inverse.resize(n_,n_);
    Eigen::MatrixXd jacobian_matrixxd(n_,n_);
    Eigen::MatrixXd jacobian_matrixxd_inv(n_,n_);
    jacobian_matrixxd.setZero();
    jacobian_matrixxd_inv.setZero();
    jacobian_matrix.setIdentity();
    jacobian_matrix_inverse.setIdentity();
    for(auto& legInfo : legInfos_)
    {
        if (robot_state_->isSupportLeg(legInfo.first))
        {
            if(legInfo.second.isPartOfForceDistribution_)
            {
                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(legInfo.first);
                jacobian_matrixxd.block<3,3>(legInfo.second.indexInStanceLegList_ * footDof_, legInfo.second.indexInStanceLegList_*footDof_) = jacobian.transpose();
                jacobian_matrixxd_inv.block<3,3>(legInfo.second.indexInStanceLegList_ * footDof_, legInfo.second.indexInStanceLegList_*footDof_) = jacobian.transpose().inverse();                
            }
        }
    }

    jacobian_matrix = jacobian_matrixxd.sparseView();
    jacobian_matrix_inverse = jacobian_matrixxd_inv.sparseView();/*
    std::cout << "jacobian matrix is "<< std::endl << jacobian_matrix << std::endl;
    std::cout << "jacobian matrix inverse is "<< std::endl << jacobian_matrix_inverse << std::endl;*/


    MatrixXd A_bottomMatrix(footDof_, n_); // TODO replace 3 with footDof_,3xn
    for (auto& legInfo : legInfos_)
    {
        if (legInfo.second.isPartOfForceDistribution_)
        {
            //! WSHY: get r_fi for each leg
            const Vector3d& r = robot_state_->getPositionBaseToFootInBaseFrame(legInfo.first).toImplementation();//legInfo.first->getPositionBaseToFootInBaseFrame().toImplementation();
//            std::cout << "r is " << r << std::endl;
            A_bottomMatrix.block(0, legInfo.second.indexInStanceLegList_ * r.size(), r.size(), r.size()) =
                kindr::getSkewMatrixFromVector(r);
        }
    }
    A_.middleRows(footDof_, A_bottomMatrix.rows()) = A_bottomMatrix.sparseView();

    W_.setIdentity(footDof_ * nLegsInForceDistribution_);
    W_ = W_ * groundForceWeight_;
    if(is_minForceDiff_)
    {
        int rowIndex = 0;
        x_pre_.resize(footDof_ * nLegsInForceDistribution_);
        for (auto& legInfo : legInfos_)
        {
            if (legInfo.second.isPartOfForceDistribution_)
            {
                x_pre_.segment(rowIndex, 3) =  legInfo.second.desiredContactForce_.vector();
                rowIndex = rowIndex + 3;
            }
        }
        H_.setIdentity(footDof_ * nLegsInForceDistribution_);
        H_ = H_ * minForceDiffWeight_;
    }

    return true;
}



bool ContactJointTorqueDistribution::addMinimalForceConstraints()
{
    /* We want each stance leg to have a minimal force in the normal direction to the ground:
   * n.f_i >= n.f_min with n.f_i the normal component of contact force.
   * inequality constraints : d <= Dx <= f
   */
    std::cout << "in the add minimal force constraints" << std::endl;
    int rowIndex = D_.rows(); // = 0
    Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_);  // TODO replace with conservativeResize (available in Eigen 3.2)
    D_.resize(rowIndex + nLegsInForceDistribution_, n_); //n X 3n
    D_.middleRows(0, D_temp.rows()) = D_temp;
    d_.conservativeResize(rowIndex + nLegsInForceDistribution_);//n X 1
    f_.conservativeResize(rowIndex + nLegsInForceDistribution_);
    //  std::cout << "rowindex + nLegsInForceDistribution_"<< rowIndex + nLegsInForceDistribution_ << std::endl;

    const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();

    for (auto& legInfo : legInfos_)
    {

        if (legInfo.second.isPartOfForceDistribution_)
        {
            Vector footContactNormalInWorldFrame = robot_state_->getSurfaceNormal(legInfo.first);//(0,0,1) normally

            Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);

            MatrixXd D_row = MatrixXd::Zero(1, n_);//1 X 3n
            D_row.block(0, legInfo.second.startIndexInVectorX_, 1, footDof_)//0,3,6,9
                = footContactNormalInBaseFrame.toImplementation().transpose();
            D_.middleRows(rowIndex, 1) = D_row.sparseView();
            d_(rowIndex) = minimalNormalGroundForce_;
            f_(rowIndex) = std::numeric_limits<double>::max();
            rowIndex++;
        }
    }
    D_ = D_ * jacobian_matrix_inverse;
    return true;
}

bool ContactJointTorqueDistribution::addJointMaxLimit()
{
    //this also with d<=Dx<=f, which d is -60, f is 60; D is the identity matrix with 3 * stance legs
    int rowIndex = D_.rows();
    int nConstraints = n_;

    Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_);
    D_.resize(rowIndex + n_, n_);
    D_.middleRows(0, D_temp.rows()) = D_temp;
    d_.conservativeResize(rowIndex + nConstraints);
    f_.conservativeResize(rowIndex + nConstraints);

    d_.segment(rowIndex,n_).setConstant(-65);
    f_.segment(rowIndex,n_).setConstant(65);
    Eigen::SparseMatrix<double, Eigen::RowMajor> D_new(n_,n_);
    D_new.setIdentity();
    D_.middleRows(D_temp.rows(),n_) = D_new;

}
bool ContactJointTorqueDistribution::addFrictionConstraints()
{
    /* For each tangent direction t, we want: -mu * n.f_i <= t.f_i <= mu * n.f_i
   * with n.f_i the normal component of contact force of leg i, t.f_i the tangential component).
   * This is equivalent to the two constraints: mu * n.f_i >= -t.f_i and mu * n.f_i >= t * f_i,
   * and equivalently mu * n.f_i + t.f_i >=0 and mu * n.f_i - t.f_i >= 0.
   * We have to define these constraints for both tangential directions (approximation of the
   * friction cone).
   */
    int nDirections = 4;
    int nConstraints = nDirections * nLegsInForceDistribution_;//!kun, 16, x and y direction,
    int rowIndex = D_.rows();
    std::cout << "in the add friction constraints" << std::endl;
    Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_); // TODO replace with conservativeResize (available in Eigen 3.2)
    D_.resize(rowIndex + nConstraints, n_);//20*12
    D_.middleRows(0, D_temp.rows()) = D_temp;
    d_.conservativeResize(rowIndex + nConstraints);
    f_.conservativeResize(rowIndex + nConstraints);

    const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();
    const RotationQuaternion orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();

    for (auto& legInfo : legInfos_)
    {
        if (legInfo.second.isPartOfForceDistribution_)
        {
            MatrixXd D_rows = MatrixXd::Zero(nDirections, n_);//4 * 3n

            Position positionWorldToFootInWorldFrame = robot_state_->getPositionWorldToFootInWorldFrame(legInfo.first);;
            Vector footContactNormalInWorldFrame;
            footContactNormalInWorldFrame = robot_state_->getSurfaceNormal(legInfo.first);//0,0,1
            Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);
            const Vector3d normalDirection = footContactNormalInBaseFrame.toImplementation();//convert contact normal direction 001 in the world frame to base frame.

            // for logging
            legInfo.second.normalDirectionOfFrictionPyramidInWorldFrame_ = Vector(footContactNormalInWorldFrame);//convert contact normal direction 001 in the world frame.

            // The choose the first tangential to lie in the XZ-plane of the base frame.
            // This is the same as the requirement as
            // 1) firstTangential perpendicular to normalDirection,
            // 2) firstTangential perpendicular to normal of XZ-plane of the base frame,
            // 3) firstTangential has unit norm.
            //      Vector3d firstTangential = normalDirection.cross(Vector3d::UnitY()).normalized();

            Vector3d vectorY = Vector3d::UnitY();//0,1,0
            Vector3d firstTangentialInBaseFrame = orientationControlToBase.rotate(vectorY);//in base frame;
            Vector3d firstTangential = normalDirection.cross(firstTangentialInBaseFrame).normalized();//footcontactnormalin baseframe *
            // logging
            legInfo.second.firstDirectionOfFrictionPyramidInWorldFrame_ = Vector(orientationWorldToBase.inverseRotate(firstTangential));

            // The second tangential is perpendicular to the normal and the first tangential.
            Vector3d secondTangential = normalDirection.cross(firstTangential).normalized();

            // logging
            legInfo.second.secondDirectionOfFrictionPyramidInWorldFrame_ = Vector(orientationWorldToBase.inverseRotate(secondTangential));

            // First tangential, positive
            D_rows.block(0, legInfo.second.startIndexInVectorX_, 1, footDof_) =
                legInfo.second.frictionCoefficient_ * normalDirection.transpose() + firstTangential.transpose();
            // First tangential, negative
            D_rows.block(1, legInfo.second.startIndexInVectorX_, 1, footDof_) =
                legInfo.second.frictionCoefficient_ * normalDirection.transpose() - firstTangential.transpose();
            // Second tangential, positive
            D_rows.block(2, legInfo.second.startIndexInVectorX_, 1, footDof_) =
                legInfo.second.frictionCoefficient_ * normalDirection.transpose() + secondTangential.transpose();
            // Second tangential, negative
            D_rows.block(3, legInfo.second.startIndexInVectorX_, 1, footDof_) =
                legInfo.second.frictionCoefficient_ * normalDirection.transpose() - secondTangential.transpose();
            D_rows = D_rows * jacobian_matrix_inverse;
            D_.middleRows(rowIndex, nDirections) = D_rows.sparseView();//nDirections = n
            d_.segment(rowIndex, nDirections) =  VectorXd::Constant(nDirections, 0.0);
            f_.segment(rowIndex, nDirections) = VectorXd::Constant(nDirections, std::numeric_limits<double>::max());

            rowIndex = rowIndex + nDirections;
        }
    }
    return true;
}

bool ContactJointTorqueDistribution::addDesiredLegLoadConstraints(const Force& virtualForce,
                                                                  const Torque& virtualTorque)
{
    /*
   * The contact force distribution is calculated
   * twice, once without the load factor equality constraint and then
   * including the equality constraint.
   * For each leg with user defined load constraints, we add equality constraints of the form
   * f_i = loadFactor * f_i_previous, with f_i the contact force of leg i and f_i_previous
   * the contact force of leg i at the optimization without user defined leg load constraints.
   */
    int nLegsWithLoadConstraintActive = 0;
    for (auto& legStatus : legInfos_)
    {
        if (legStatus.second.isLoadConstraintActive_)
            nLegsWithLoadConstraintActive++;
    }
    if (nLegsWithLoadConstraintActive == 0) return true; // No need to have equality constraints


    int nConstraints = footDof_ * nLegsWithLoadConstraintActive;//3n
    int rowIndex = C_.rows();//the number of rows of C_//0

    Eigen::SparseMatrix<double, Eigen::RowMajor> C_temp(C_);  // TODO replace with conservativeResize (available in Eigen 3.2

    C_.resize(rowIndex + nConstraints, n_);//3n*3n
    C_.middleRows(0, C_temp.rows()) = C_temp;

    C_ = A_;
    C_ = C_ * jacobian_matrix_inverse;

    c_.resize(6);
    c_.segment(0,virtualForce.toImplementation().size()) = virtualForce.toImplementation();
    c_.segment(3,virtualTorque.toImplementation().size()) = virtualTorque.toImplementation();



//    solveOptimization(); // Solving once without equality constraints

//    for (auto& legInfo : legInfos_)
//    {
//        if (legInfo.second.isLoadConstraintActive_)
//        {
//            int m = footDof_;//3
//            MatrixXd C_rows = MatrixXd::Zero(m, n_);//3 * 3n
//            C_rows.block(0, legInfo.second.startIndexInVectorX_, m, m) = MatrixXd::Identity(m, m);//0,3,6,9
//            C_.middleRows(rowIndex, m) = C_rows.sparseView();
//            Eigen::Matrix<double, 3, 1> fullForce = x_.segment(legInfo.second.startIndexInVectorX_, footDof_);
//            c_.segment(rowIndex, m) =  legInfo.second.loadFactor_ * jacobian_matrix.block(legInfo.second.startIndexInVectorX_, legInfo.second.startIndexInVectorX_,3,3).transpose()
//                                                                       * fullForce;
//            rowIndex = rowIndex + m;
//        }
//    }

    return true;
}

bool ContactJointTorqueDistribution::solveOptimization()
{

    getparameters();
    if (!ooqpei::QuadraticProblemFormulation::solve(A_, S_, b_, W_, C_, c_, D_, d_, f_, x_))
    {
        ROS_ERROR("Contact Force is Minus !!!!!!!!!!!!!!!!!!!!");
        return false;
    }

    std::cout << "successfully solved the joint torque, the joint torque is " << std::endl;
    std::cout << x_.transpose() << std::endl;

//    for (auto& legInfo : legInfos_)
//    {
//        if (legInfo.second.isPartOfForceDistribution_)
//        {
//            // The forces we computed here are actually the ground reaction forces,
//            // so the stance legs should push the ground by the opposite amount.
//            legInfo.second.desiredContactForce_ =
//                Force(-x_.segment(legInfo.second.startIndexInVectorX_, footDof_));
//            std::cout << "the desired force is " << legInfo.second.desiredContactForce_ << std::endl;
//            if(legInfo.second.desiredContactForce_(2) > 0)
//            {
//                ROS_ERROR("Contact Force is Minus !!!!!!!!!!!!!!!!!!!!");
//                std::cerr<<"Nominal Force is :"<<x_<<std::endl<<"constaints : "<<d_<<std::endl;
//            }

//        }
//    }

    return true;
}

void ContactJointTorqueDistribution::getparameters() const
{
        std::cout << "A_ is ............... " << std::endl << A_ << std::endl;
        std::cout << "b_ is " << std::endl << b_ << std::endl;
        std::cout << "s_ is " << std::endl << S_.diagonal() << std::endl;
        std::cout << "w_ is " << std::endl << W_.diagonal() << std::endl;
        std::cout << "C_ is " << std::endl << C_ << std::endl;
        std::cout << "c_ is " << std::endl << c_ << std::endl;
        std::cout << "D_ is " << std::endl << D_ << std::endl;
        std::cout << "d_ is " << std::endl << d_ << std::endl;
        std::cout << "f_ is " << std::endl << f_ << std::endl;
        std::cout << "x_ is " << std::endl << x_ << std::endl;
}

bool ContactJointTorqueDistribution::computeJointTorques()
{
    const LinearAcceleration gravitationalAccelerationInWorldFrame = LinearAcceleration(0.0,0.0,-9.8);//torso_->getProperties().getGravity();
    const LinearAcceleration gravitationalAccelerationInBaseFrame = robot_state_->getOrientationBaseToWorld().inverseRotate(gravitationalAccelerationInWorldFrame);//torso_->getMeasuredState().getOrientationWorldToBase().rotate(gravitationalAccelerationInWorldFrame);

    //  const int nDofPerLeg = 3; // TODO move to robot commons
    //  const int nDofPerContactPoint = 3; // TODO move to robot commons

    for (auto& legInfo : legInfos_)
    {

        /*
     * Torque setpoints should be updated only is leg is support leg.
     */
        if  (robot_state_->isSupportLeg(legInfo.first)) {

            if (legInfo.second.isPartOfForceDistribution_)
            {
                //        LegBase::TranslationJacobian jacobian = legInfo.first->getTranslationJacobianFromBaseToFootInBaseFrame();
                Eigen::Matrix3d jacobian = robot_state_->getTranslationJacobianFromBaseToFootInBaseFrame(legInfo.first);
                Force contactForce = legInfo.second.desiredContactForce_;//! WSHY: TODO tranform to hip frame
                Position tranformed_vector = robot_state_->getPositionFootToHipInHipFrame(legInfo.first, Position(contactForce.toImplementation()));
                //        contactForce = Force(tranformed_vector.toImplementation());
                ROS_DEBUG("leg Jacobian for %d is : \n", static_cast<int>(legInfo.first));
                //        std::cout<<jacobian<<std::endl;
                ROS_DEBUG("contact force for %d is : \n", static_cast<int>(legInfo.first));
                //        std::cout<<contactForce.toImplementation()<<std::endl;

                //        LegBase::JointTorques jointTorques = LegBase::JointTorques(jacobian.transpose() * contactForce.toImplementation());
                free_gait::JointEffortsLeg jointTorques = free_gait::JointEffortsLeg(jacobian.transpose() * contactForce.toImplementation());
                //      jointTorques += LegBase::JointTorques(torso_ Force(-torso_->getProperties().getMass() * gravitationalAccelerationInBaseFrame));
                /* gravity */
                /****************
        * TODO(Shunyao) : fix jacobian for each link
        ****************/
                //        for (auto link : *legInfo.first->getLinks()) {
                //          jointTorques -= LegBase::JointTorques( link->getTranslationJacobianBaseToCoMInBaseFrame().transpose() * Force(link->getMass() * gravitationalAccelerationInBaseFrame).toImplementation());
                //        }

                free_gait::JointPositionsLeg joint_position_leg = robot_state_->getJointPositionFeedbackForLimb(legInfo.first);
                jointTorques += robot_state_->getGravityCompensationForLimb(legInfo.first, joint_position_leg, free_gait::Force(gravitationalAccelerationInBaseFrame.toImplementation()));

                //        legInfo.first->setDesiredJointTorques(jointTorques);
                robot_state_->setJointEffortsForLimb(legInfo.first, jointTorques);
                //        ROS_INFO("Joint Torque for %d is : \n", static_cast<int>(legInfo.first));
                //        std::cout<<jointTorques<<std::endl;
            }
            else
            {
                /*
         * True if load factor is zero.
         */
                //        legInfo.first->setDesiredJointTorques(free_gait::JointEffortsLeg::Zero());
                robot_state_->setJointEffortsForLimb(legInfo.first, free_gait::JointEffortsLeg::Zero());
            }

        }

    }
    //  ROS_INFO("Computed joint Torque once : \n");
    //  std::cout<<robot_state_->getAllJointEfforts()<<std::endl;
    return true;
}

bool ContactJointTorqueDistribution::resetOptimization()
{
    isForceDistributionComputed_ = false;

    D_.resize(0, 0);
    d_.resize(0);
    f_.resize(0);
    C_.resize(0, 0);
    c_.resize(0);

    for (auto& legInfo : legInfos_)
    {
        if(!is_minForceDiff_)
            legInfo.second.desiredContactForce_.setZero();
    }

    return true;
}

//bool ContactForceDistribution::getForceForLeg(
//    const Legs& leg, robotModel::VectorCF& force)
//{
//  if (!checkIfForceDistributionComputed()) return false;
//
//  if (legInfos_[leg].isInStance_)
//  {
//    // The forces we computed here are actually the ground reaction forces,
//    // so the stance legs should push the ground by the opposite amount.
//    force = -x_.segment(legInfos_[leg].startIndexInVectorX_, footDof_);
//    return true;
//  }
//
//  return false;
//}

bool ContactJointTorqueDistribution::getNetForceAndTorqueOnBase(
    Force& netForce, Torque& netTorque)
{
    if (!checkIfForceDistributionComputed()) return false;

    Eigen::Matrix<double, nElementsVirtualForceTorqueVector_, 1> stackedNetForceAndTorque;
    stackedNetForceAndTorque = A_ *jacobian_matrix_inverse *  x_;
    netForce = Force(stackedNetForceAndTorque.head(3));
    netTorque = Torque(stackedNetForceAndTorque.tail(3));

    return true;
}

bool ContactJointTorqueDistribution::updateLoggerData()
{
    if (!isLogging_) return false;
    return true;
}

double ContactJointTorqueDistribution::getGroundForceWeight() const {
    return groundForceWeight_;
}

double ContactJointTorqueDistribution::getMinimalNormalGroundForce() const {
    return minimalNormalGroundForce_;
}
double ContactJointTorqueDistribution::getVirtualForceWeight(int index) const {
    return virtualForceWeights_(index);
}

const Vector& ContactJointTorqueDistribution::getFirstDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum leg) const {
    return legInfos_.at(leg).firstDirectionOfFrictionPyramidInWorldFrame_;
}
const Vector& ContactJointTorqueDistribution::getSecondDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum leg) const {
    return legInfos_.at(leg).secondDirectionOfFrictionPyramidInWorldFrame_;
}
const Vector& ContactJointTorqueDistribution::getNormalDirectionOfFrictionPyramidInWorldFrame(free_gait::LimbEnum leg) const {
    return legInfos_.at(leg).normalDirectionOfFrictionPyramidInWorldFrame_;
}

double ContactJointTorqueDistribution::getFrictionCoefficient(free_gait::LimbEnum leg) const {
    return legInfos_.at(leg).frictionCoefficient_;
}

//double ContactForceDistribution::getFrictionCoefficient(const free_gait::LimbEnum leg) const {
//  //return legInfos_.at(const_cast<free_gait::LimbEnum>(leg)).frictionCoefficient_;
//  return legInfos_.at(leg).frictionCoefficient_;

//}

double ContactJointTorqueDistribution::getFrictionCoefficient(int index) const {
    const free_gait::LimbEnum leg = static_cast<free_gait::LimbEnum>(index);
    //  return legInfos_.at(const_cast<free_gait::LimbEnum*>(leg)).frictionCoefficient_;
    return legInfos_.at(leg).frictionCoefficient_;
}


const ContactJointTorqueDistribution::LegInfo& ContactJointTorqueDistribution::getLegInfo(free_gait::LimbEnum leg) const {
    return legInfos_.at(leg);
}
/**
 * @brief ContactForceDistribution::setToInterpolated, what's the useness?
 * @param contactForceDistribution1
 * @param contactForceDistribution2
 * @param t
 * @return
 */
bool ContactJointTorqueDistribution::setToInterpolated(const ContactForceDistributionBase& contactForceDistribution1, const ContactForceDistributionBase& contactForceDistribution2, double t) {
    const ContactJointTorqueDistribution& distribution1 = static_cast<const ContactJointTorqueDistribution&>(contactForceDistribution1);
    const ContactJointTorqueDistribution& distribution2 = static_cast<const ContactJointTorqueDistribution&>(contactForceDistribution2);

    if(!distribution1.checkIfParametersLoaded()) {
        return false;
    }
    if(!distribution2.checkIfParametersLoaded()) {
        return false;
    }


    for (auto leg : limbs_) {
        //      int index = static_cast<int>(leg);
        legInfos_.at(leg).frictionCoefficient_ = linearlyInterpolate(distribution1.getFrictionCoefficient(leg) , distribution2.getFrictionCoefficient(leg), 0.0, 1.0, t);
    }

    this->groundForceWeight_ = linearlyInterpolate(distribution1.getGroundForceWeight(), distribution2.getGroundForceWeight(), 0.0, 1.0, t);
    this->minimalNormalGroundForce_ = linearlyInterpolate(distribution1.getMinimalNormalGroundForce(), distribution2.getMinimalNormalGroundForce(), 0.0, 1.0, t);

    for (int i=0; i<(int)this->virtualForceWeights_.size(); i++) {
        this->virtualForceWeights_(i) = linearlyInterpolate(distribution1.getVirtualForceWeight(i), distribution2.getVirtualForceWeight(i), 0.0, 1.0, t);
    }
    return true;
}

Eigen::VectorXd ContactJointTorqueDistribution::getTheOptimizationResults()
{
    return x_;
}

Eigen::SparseMatrix<double, Eigen::RowMajor> ContactJointTorqueDistribution::getTheJacobianMatrix()
{
    return jacobian_matrix;
}

Eigen::SparseMatrix<double, Eigen::RowMajor> ContactJointTorqueDistribution::getTheJacobianMatrixInv()
{
    return jacobian_matrix_inverse;
}


bool ContactJointTorqueDistribution::loadParameters(const TiXmlHandle& handle)
{
    isParametersLoaded_ = false;

    TiXmlElement* element = handle.FirstChild("ContactForceDistribution").ToElement();
    if (!element) {
        printf("Could not find ContactForceDistribution\n");
        return false;
    }
    TiXmlHandle forceDistributionHandle(handle.FirstChild("ContactForceDistribution"));

    // Weights
    element = forceDistributionHandle.FirstChild("Weights").Element();
    if (!element) {
        printf("Could not find ContactForceDistribution:Weights\n");
        return false;
    }
    TiXmlHandle weightsHandle(handle.FirstChild("ContactForceDistribution").FirstChild("Weights"));

    // Virtual force weights
    element = weightsHandle.FirstChild("Force").Element();
    if (!element) {
        printf("Could not find ContactForceDistribution:Weights:Force!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("heading", &virtualForceWeights_(0))!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Force:heading!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("lateral", &virtualForceWeights_(1))!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Force:lateral!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("vertical", &virtualForceWeights_(2))!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Force:vertical!\n");
        return false;
    }

    // Virtual torque weights
    element = weightsHandle.FirstChild("Torque").Element();
    if (!element) {
        printf("Could not find ContactForceDistribution:Weights:Torque!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("roll", &virtualForceWeights_(3))!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Torque:roll!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("pitch", &virtualForceWeights_(4))!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Torque:pitch!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("yaw", &virtualForceWeights_(5))!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Torque:yaw!\n");
        return false;
    }

    // Regularizer
    element = weightsHandle.FirstChild("Regularizer").Element();
    if (!element) {
        printf("Could not find ContactForceDistribution:Weights:Regularizer!\n");
        return false;
    }
    if (element->QueryDoubleAttribute("value", &groundForceWeight_)!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Weights:Regularizer:value!\n");
        return false;
    }

    // Constraints
    element = forceDistributionHandle.FirstChild("Constraints").Element();
    if (!element) {
        printf("Could not find ContactForceDistribution:Constraints\n");
        return false;
    }
    double frictionCoefficient = 0.6;
    if (element->QueryDoubleAttribute("frictionCoefficient", &frictionCoefficient)!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Constraints:frictionCoefficient!\n");
        return false;
    }
    for (auto& legInfo : legInfos_) {
        legInfo.second.frictionCoefficient_ = frictionCoefficient;
    }
    if (element->QueryDoubleAttribute("minimalNormalForce", &minimalNormalGroundForce_)!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:Constraints:minimalNormalForce!\n");
        return false;
    }

    // Load factor
    element = forceDistributionHandle.FirstChild("LoadFactor").Element();
    if (!element) {
        printf("Could not find ContactForceDistribution:Constraints\n");
        return false;
    }
    double loadFactor = 1.0;
    if (element->QueryDoubleAttribute("loadFactor", &loadFactor)!=TIXML_SUCCESS) {
        printf("Could not find ContactForceDistribution:LoadFactor:loadFactor!\n");
        return false;
    }
    /****************
  * TODO(Shunyao) : figure out the use of loadFactor and fix it
  ****************/
    //  for (auto& legInfo : legInfos_) {
    //    legInfo.first->setDesiredLoadFactor(loadFactor);
    //  }


    isParametersLoaded_ = true;
    return true;
}

bool ContactJointTorqueDistribution::loadParameters()
{
    isParametersLoaded_ = false;
    // Force
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/force/heading")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/force/heading", virtualForceWeights_(0));
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/force/heading'.");
        return false;
    }
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/force/lateral")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/force/lateral", virtualForceWeights_(1));
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/force/lateral'.");
        return false;
    }
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/force/vertical")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/force/vertical", virtualForceWeights_(2));
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/force/vertical'.");
        return false;
    }
    // Torque
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/torque/roll")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/torque/roll", virtualForceWeights_(3));
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/torque/roll'.");
        return false;
    }
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/torque/pitch")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/torque/pitch", virtualForceWeights_(4));
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/torque/pitch'.");
        return false;
    }
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/torque/yaw")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/torque/yaw", virtualForceWeights_(5));
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/torque/yaw'.");
        return false;
    }
    // Regularizer
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/regularizer/value")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/regularizer/value", groundForceWeight_);
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/regularizer/value'.");
        return false;
    }

    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/force_diff/value")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/force_diff/value", minForceDiffWeight_);
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/force_diff/value'.");
        return false;
    }

    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/weights/force_diff/is_use")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/weights/force_diff/is_use", is_minForceDiff_);
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/weights/force_diff/is_use'.");
        return false;
    }

    // Constraints
    double frictionCoefficient = 0.6;
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/constraints/friction_coefficient")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/constraints/friction_coefficient", frictionCoefficient);
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/constraints/friction_coefficient'.");
        return false;
    }
    for (auto& legInfo : legInfos_) {
        legInfo.second.frictionCoefficient_ = frictionCoefficient;
    }
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/constraints/minimal_normal_force")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/constraints/minimal_normal_force", minimalNormalGroundForce_);
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/constraints/minimal_normal_force'.");
        return false;
    }

    double load_factor = 1;
    if (node_handle_.hasParam("/balance_controller/contact_force_distribution/constraints/load_factor")) {
        node_handle_.getParam("/balance_controller/contact_force_distribution/constraints/load_factor", load_factor);
    } else {
        ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/contact_force_distribution/constraints/load_factor'.");
        return false;
    }

    for (auto& legInfo : legInfos_) {
        legInfo.second.loadFactor_ = load_factor;
    }

    isParametersLoaded_ = true;
    return true;
}

//const LegGroup* ContactForceDistribution::getLegs() const {
//  return legs_.get();
//}

} /* namespace loco */
