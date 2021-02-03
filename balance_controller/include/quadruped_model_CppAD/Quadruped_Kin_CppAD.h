
#ifndef QUADRUPED_KIN_CPPAD_H
#define QUADRUPED_KIN_CPPAD_H

#include "iit/robots/quadruped_model/jacobians.h"
#include "iit/robots/quadruped_model/transforms.h"
#include "ros/ros.h"
#include "free_gait_core/executor/State.hpp"
#include "memory"//ptr
#include "unordered_map"
#include "utility"

#include "kindr/Core"
#include <iostream>
namespace quadruped_model
{
typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
typedef Eigen::Matrix<CppAD::AD<double>,3,1> Force;
typedef kindr::HomTransformQuat<CppAD::AD<double>> Pose_cppad;
typedef kindr::RotationQuaternion<CppAD::AD<double>> RotationQuaternion_cppad;
typedef kindr::Position<CppAD::AD<double>,3> Position_cppad;
class Quad_Kin_CppAD
{
public:
    struct LEGINFO
    {
        bool isPartOfForceDistribution_;
        bool isLoadConstraintActive_;
        int indexInStanceLegList_;
        int startIndexInVectorX_;
        Force desiredContactForce_;
        double frictionCoefficient_;
        double loadFactor_;
    };
    Quad_Kin_CppAD(std::shared_ptr<free_gait::State> robot_state);
    ~Quad_Kin_CppAD();
    /**
     * @brief Matrix_Printf
     * This file is used to printf the Eigen::Matrix<CppAD::AD<double>>
     */
    template<typename T, int rows, int cols>
    void Matrix_Printf(const Eigen::Matrix<T, rows, cols>& matrix);

    /**
     * @brief Angles_Torques_Initial
     * TO initialize the angles and torques
     */

    void Angles_Torques_Initial(const ADvector& x);


    /**
     * @brief Get_foot_position w.r.t base frame;
     * @param leg_num
     * @return leg num corresponding position
     */
    const Position_cppad Get_foot_position_In_Baseframe(const LimbEnum& limb);

    /**
     * @brief StoreFootPositionInWorldframe
     */
    const Position_cppad GetFootPositionInWorldframe(const LimbEnum& limb);

    /**
     * @brief SetBaseInWorld
     * get the transformation to express the base in the world frame;
     */
    void SetBaseInWorld(const Pose_cppad& base_pose);

    /**
     * @brief PrepareaOptimization
     * calculate all the jacobian.inverse, matrix A in CppAD verrsion, full it into matrix;
     */    
    void PrepareOptimization();

    /**
     * @brief PrepareLegLoading
     * convert the robot_state parameters to the matrix, like the contactforcedistribution file
     * @return
     */
    bool PrepareLegLoading();

    /**
     * @brief Store_the_jacobians
     * store the jacobians based on the leg;
     */
    void Store_the_jacobians();

    /**
     * @brief GetFootJacobian
     * get the big jacobian matrix Jac_;
     * @return
     */
    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> GetFootJacobian();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> GetFootDoubleJacobian();

    /**
     * @brief GetFootJacobian
     * @return Get the big matrix A;
     */
    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> GetAMatrix();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> GetADoubleMatrix();

    /**
     *printf the eigen matrix;
     */
    void EigenMatrixPrintf(const Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic>& EigenMatrix);
    void CppadPositionPrintf(const Position_cppad& position);
    void CppadRotationPrintf(const RotationQuaternion_cppad& rotation);
    void CppadPosePrintf(const Pose_cppad& pose);

public:
    std::map<free_gait::LimbEnum, LEGINFO> legInfos_;
    std::map<free_gait::LimbEnum, Position_cppad> foot_position_;

    typedef  Eigen::Matrix<CppAD::AD<double>, 3, 3> jac_33_;
    std::map<free_gait::LimbEnum, jac_33_> foot_jacobians_;

private:
    iit::simpledog::JointState angles_;
    iit::simpledog::JointState torques_;
    iit::simpledog::Jacobians jac_;//jacobian;

    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> Jacobians_;
    std::vector<free_gait::LimbEnum> limbs_;
    int footDof_;
    std::shared_ptr<free_gait::State> robot_state_;
    int nLegsInForceDistributuion_;
    int n_;// the matrix size or the matrix A size;
    Eigen::Matrix<CppAD::AD<double>, Eigen::Dynamic, Eigen::Dynamic> A_;

    Pose_cppad base_pose_to_world_;




};
}//namespace
#endif // QUADRUPEDKINEMATICS_H
