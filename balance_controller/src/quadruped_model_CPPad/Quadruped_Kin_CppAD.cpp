/**
This file is the quadruped kinematics combine the CPPAD version, which can use the code the make some optimal solve.
The single leg optimization is in the ifopt_test_execuable.cpp in balance_controller test folder;;
*/
#include "quadruped_model_CppAD/Quadruped_Kin_CppAD.h"

#include "iostream"
#include "Eigen/LU"
namespace quadruped_model {
Quad_Kin_CppAD::Quad_Kin_CppAD(std::shared_ptr<free_gait::State> robot_state)
    :footDof_(3){
    std::cout << "construct the quad_kin_cppad" << std::endl;
    angles_.setOnes();
    torques_.setOnes();
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    nLegsInForceDistributuion_ = 0;

    for (auto leg : limbs_) {
        legInfos_[leg] = LEGINFO();
    }
    robot_state_.reset(new free_gait::State);
    robot_state_ = robot_state;
}

Quad_Kin_CppAD::~Quad_Kin_CppAD()
{
    std::cout << "release the Quad_Kin_CppAD" << std::endl;
}

void Quad_Kin_CppAD::Angles_Torques_Initial(const ADvector& x)
{
//    std::cout << "initial angles and torques" << std::endl;
    JointPositions all_joint_positions;
    for (unsigned int i = 0; i < 12; i++) {
        angles_(i,0) = x[i];
        torques_(i,0) = x[i + 12];
        all_joint_positions(i) =CppAD::Value(x[i]);
    }
    robot_state_->setJointPositions(all_joint_positions);
}

template<typename T, int rows, int cols>
void Quad_Kin_CppAD::Matrix_Printf(const Eigen::Matrix<T, rows, cols>& matrix)
{
    for (unsigned int i = 0; i < matrix.rows(); i++) {
        for (unsigned int j = 0; j < matrix.cols(); j++) {
            std::cout << matrix(i,j) << " ";
        }
        std::cout << std::endl;
    }
}

const Position_cppad Quad_Kin_CppAD::Get_foot_position_In_Baseframe(const LimbEnum& limb)
{
    Eigen::Matrix<CppAD::AD<double>, 4, 4> foot_pose;
    iit::simpledog::HomogeneousTransforms motion_trans;

    switch (limb) {
    case LimbEnum::LF_LEG:
        foot_pose = motion_trans.fr_base_X_LF_FOOT(angles_);
        foot_position_[LimbEnum::LF_LEG].vector() = foot_pose.block(0,3,3,1);
        break;
    case LimbEnum::RF_LEG:
        foot_pose = motion_trans.fr_base_X_RF_FOOT(angles_);
        foot_position_[LimbEnum::RF_LEG].vector() = foot_pose.block(0,3,3,1);
        break;
    case LimbEnum::RH_LEG:
        foot_pose = motion_trans.fr_base_X_RH_FOOT(angles_);
        foot_position_[LimbEnum::RH_LEG].vector() = foot_pose.block(0,3,3,1);
    case LimbEnum::LH_LEG:
        foot_pose = motion_trans.fr_base_X_LH_FOOT(angles_);
        foot_position_[LimbEnum::LH_LEG].vector() = foot_pose.block(0,3,3,1);
        break;
    }
    return foot_position_[limb];
}


bool Quad_Kin_CppAD::PrepareLegLoading()
{
//    std::cout << "PrepareLegLoading" << std::endl;
    nLegsInForceDistributuion_ = 0;
    for (auto& leginfo : legInfos_) {
        if(robot_state_->isSupportLeg(leginfo.first))
        {
            leginfo.second.isPartOfForceDistribution_ = true;
            leginfo.second.isLoadConstraintActive_ = false;
            leginfo.second.indexInStanceLegList_ = nLegsInForceDistributuion_;//0,1,2,3
            leginfo.second.startIndexInVectorX_ = leginfo.second.indexInStanceLegList_ * footDof_;
            nLegsInForceDistributuion_ ++ ;
            leginfo.second.isLoadConstraintActive_ = true;
        }else {
            leginfo.second.isPartOfForceDistribution_ = false;
            leginfo.second.isLoadConstraintActive_ = false;
        }
    }
    return true;
}

void Quad_Kin_CppAD::PrepareOptimization()
{
//    std::cout << "PrepareOptimization" << std::endl;
    n_ = footDof_ * nLegsInForceDistributuion_;//3 * nlegs support;
    A_.resize(6,n_);
    A_.setZero();
    Eigen::Matrix<CppAD::AD<double>,3,3> I_matrix;
    I_matrix.setIdentity();
    A_.middleRows(0, footDof_) = I_matrix.replicate(1, nLegsInForceDistributuion_);
//    std::cout << "success get the A_" << std::endl;

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_bottomMatrix(3,n_);
    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> foot_skew(3, 3);
    Position_cppad foot_posi;
    for (auto& legInfo : legInfos_) {
        if (legInfo.second.isPartOfForceDistribution_)
        {
            foot_posi = Get_foot_position_In_Baseframe(legInfo.first);
            foot_skew(0,0) = 0;
            foot_skew(0,1) = -foot_posi(2,0);
            foot_skew(0,2) = foot_posi(1,0);
            foot_skew(1,0) = foot_posi(2,0);
            foot_skew(1,1) = 0;
            foot_skew(1,2) = -foot_posi(0,0);
            foot_skew(2,0) = -foot_posi(1,0);
            foot_skew(2,1) = foot_posi(0,0);
            foot_skew(2,2) = 0;
        }
        A_bottomMatrix.block(0,legInfo.second.startIndexInVectorX_, 3,3) = foot_skew;
    }
    A_.middleRows(footDof_,A_bottomMatrix.rows()) = A_bottomMatrix;

    //calculate the jacobian.inverse
    Store_the_jacobians();
    Jacobians_.resize(3 * nLegsInForceDistributuion_, 3 * nLegsInForceDistributuion_);
    Jacobians_.setZero();
    for (auto& legInfo : legInfos_) {
        if(legInfo.second.isPartOfForceDistribution_)
        {
            Jacobians_.block(legInfo.second.startIndexInVectorX_, legInfo.second.startIndexInVectorX_, 3, 3) =
                foot_jacobians_[legInfo.first];
        }
    }
//    std::cout << "success get the jacobians" << std::endl;
}

void Quad_Kin_CppAD::Store_the_jacobians()
{
//    std::cout << "Store_the_jacobians" << std::endl;
    iit::simpledog::Jacobians jac;
    Eigen::Matrix<CppAD::AD<double>, 3, 3> jac33;
    Eigen::Matrix<CppAD::AD<double>, 6, 3> jac63;

    jac63 = jac.fr_base_J_LF_FOOT(angles_);
    jac33 = jac63.block(3,0,3,3).transpose().inverse();
    foot_jacobians_[LimbEnum::LF_LEG] = jac33;

    jac63 = jac.fr_base_J_RF_FOOT(angles_);
    jac33 = jac63.block(3,0,3,3).transpose().inverse();
    foot_jacobians_[LimbEnum::RF_LEG] = jac33;

    jac63 = jac.fr_base_J_RH_FOOT(angles_);
    jac33 = jac63.block(3,0,3,3).transpose().inverse();
    foot_jacobians_[LimbEnum::RH_LEG] = jac33;

    jac63 = jac.fr_base_J_LH_FOOT(angles_);
    jac33 = jac63.block(3,0,3,3).transpose().inverse();
    foot_jacobians_[LimbEnum::LH_LEG] = jac33;
}

Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> Quad_Kin_CppAD::GetFootJacobian()
{
//    std::cout << "get the jacobian_" << std::endl;
    return Jacobians_;
}

Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> Quad_Kin_CppAD::GetAMatrix()
{
//    std::cout << "get the matrix A" << std::endl;
    return A_;
}

void Quad_Kin_CppAD::SetBaseInWorld(const Pose_cppad &base_pose)
{
    base_pose_to_world_ = base_pose;
}

const Position_cppad Quad_Kin_CppAD::GetFootPositionInWorldframe(const LimbEnum& limb)
{
    Position_cppad foot_in_base, base_in_world;
    foot_in_base = Get_foot_position_In_Baseframe(limb);
    base_in_world = base_pose_to_world_.getPosition();
    return base_pose_to_world_.getRotation().rotate(foot_in_base) + base_in_world;
}

void Quad_Kin_CppAD::EigenMatrixPrintf(const Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic>& EigenMatrix)
{
    Eigen::IOFormat OctaveFmt(4, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::MatrixXd temp_matrix;
    temp_matrix.resize(EigenMatrix.rows(),EigenMatrix.cols());
    for (unsigned int i = 0; i < EigenMatrix.rows(); i++) {
        for (int var = 0; var < EigenMatrix.cols(); ++var) {
            temp_matrix(i,var) = CppAD::Value(EigenMatrix(i,var));
        }
    }
    std::cout << temp_matrix.format(OctaveFmt) << std::endl;
}

void Quad_Kin_CppAD::CppadPositionPrintf(const Position_cppad &position)
{
    Eigen::IOFormat OctaveFmt(4, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::MatrixXd temp_matrix;
    temp_matrix.resize(1,3);
    temp_matrix(0,0) = CppAD::Value(position.x());
    temp_matrix(0,1) = CppAD::Value(position.y());
    temp_matrix(0,2) = CppAD::Value(position.z());
    std::cout << temp_matrix.format(OctaveFmt) << std::endl;
}

void Quad_Kin_CppAD::CppadRotationPrintf(const RotationQuaternion_cppad &rotation)
{
    Eigen::IOFormat OctaveFmt(4, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::MatrixXd temp_matrix;
    temp_matrix.resize(1,4);
    temp_matrix(0,1) = CppAD::Value(rotation.x());
    temp_matrix(0,2) = CppAD::Value(rotation.y());
    temp_matrix(0,3) = CppAD::Value(rotation.z());
    temp_matrix(0,0) = CppAD::Value(rotation.w());
    std::cout << temp_matrix.format(OctaveFmt) << std::endl;
}

void Quad_Kin_CppAD::CppadPosePrintf(const Pose_cppad &pose)
{
    CppadPositionPrintf(pose.getPosition());
    CppadRotationPrintf(pose.getRotation());
}



}//namespace
