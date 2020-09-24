/*
 *  kinematicsTest.cpp
 *  Descriotion:
 *
 *  Created on: date, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "quadruped_model/quadrupedkinematics.h"
#include "rbdl/Model.h"
#include "rbdl/addons/urdfreader/urdfreader.h"
#include "curves/ScalarCurveConfig.hpp"
#include "curves/PolynomialSplineContainer.hpp"
#include "curves/polynomial_splines_containers.hpp"
#include "curves/PolynomialSplineScalarCurve.hpp"
#include "curves/PolynomialSplineVectorSpaceCurve.hpp"

#include "iostream"
#include "fstream"
#include "sstream"

//the third joint rolling 360 degree when go upstairs;
using namespace std;
using namespace quadruped_model;

typedef typename curves::Time Time;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType valuetype;

QuadrupedKinematics QK;

void SaveAsFile(const std::string& file_name, const std::vector<JointPositions>& joint_position_vector)
{
    std::ofstream invFile;
    std::cout << "Saving the data" << std::endl;
    invFile.open(file_name);
    if(invFile.fail())
    {
        std::cerr << "The file cannot be opened!";
    }

    double time_ = 0;
    for (unsigned int i = 0; i < joint_position_vector.size(); i++) {
        invFile <<time_ << " " << joint_position_vector[i] << " ";
        invFile << "\r\n";
        time_ = time_ + 0.0025;
    }
    invFile.close();

    std::cout << "success store the file " << std::endl;
}

void SaveAsFile_with_footheight(const std::string &file_name, const std::vector<JointPositions>& joint_position_vector, const std::vector<valuetype>& lf_foot_height,
                const std::vector<valuetype>& rf_foot_height, const std::vector<valuetype>& rh_foot_height,
                const std::vector<valuetype>& lh_foot_height)
{
    std::ofstream invFile;
    std::cout << "Saving the data" << std::endl;
    invFile.open(file_name);
    if(invFile.fail())
    {
        std::cerr << "The file cannot be opened!";
    }

    double time_ = 0;
    for (unsigned int i = 0; i < joint_position_vector.size(); i++) {
        invFile <<time_ << " " << joint_position_vector[i] << " " << lf_foot_height[i].transpose() << " " << rf_foot_height[i].transpose() << " " << rh_foot_height[i].transpose() << " "<< lh_foot_height[i].transpose() << " ";
        invFile << "\r\n";
        time_ = time_ + 0.0025;
    }
    invFile.close();
    std::cout << "success store the file " << std::endl;
}

double step_dis = 0.30;
double forward_d = 0.08;
double adjust_t = 2.4;
double period_t = 5.8;

double delta_t = 0.0025;
valuetype generate_lf_motion_1(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    //height 2 is the height of the stairs

    step =1;
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 11;


    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    //step 1: move back;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves by rolling 360 degree;

    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start));

    times_of_lf[j].push_back(0.2 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start + 0.05));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;

    start_pose.getPosition() << x_start - step_dis / 2 + forward_d, y_start, z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::LF_LEG, start_joints, start_joints, "IN_LEFT");

    final_pose.getPosition() << x_start + step_dis / 2 + forward_d, y_start, z_start + height_2 + 0.05;
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::LF_LEG, final_joints, final_joints, "IN_LEFT");

    j = 8;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

//        std::cout << "start joint is " << start_joints << std::endl;

    times_of_lf[j].push_back(0.3 * period_t);
    lf_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 9;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    times_of_lf[j].push_back(0.3 * period_t);
    lf_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

//        std::cout << "final joint is " << final_joints << std::endl;

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 10;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start + height_2 + 0.05));

    times_of_lf[j].push_back(0.2 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);


    //step 3: com moves foward;
    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start, z_start + height_2));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start, z_start + height_2));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5: rf leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start, z_start + height_2));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: come moves forward;

    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start, z_start + height_2));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: lh leg moves;

    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start, z_start + height_2));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 2 - forward_d, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            if(step == 1 || step == 2)
            {
                if(dt > adjust_t && dt <= (0.2 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.2 * period_t + adjust_t) && dt <= (0.5 * period_t + adjust_t))
                {
                    dt_ = dt - (0.2 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (0.5 * period_t + adjust_t) && dt <= (0.8 * period_t + adjust_t))
                {
                    dt_ = dt - (0.5 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (0.8 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.8 * period_t + adjust_t);
                    lf_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }
            }else {// in plain
                if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
                {
                    dt_ = dt - (0.25 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.75 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 1 || step == 2)
        {
            if(dt > (0.2 * period_t + adjust_t) && dt <= ( 0.8 * period_t + adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LF_LEG, lf_leg_pose);
            }else {
                lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
            }
        }else {
            lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        }

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }
    std::cout << "finish lf step 1" << std::endl;
    return pos_final;
}
valuetype generate_rf_motion_1(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    double rf_x_start = height_x;
    double rf_y_start = -height_y;
    double rf_z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_num;
    if(step == 1 || step == 2)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }
    rf_leg_trajectory.resize(step_num);
    rf_leg_position.resize(step_num);
    times_of_rf.resize(step_num);

    unsigned long j;

    //step 0 : com moves to back;
    j = 0;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 - forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 + forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: com moves to forward
    j = 2;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 + forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 - forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 - forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: com moves to back
    j = 4;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: rf leg moves

    j = 5;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(0.2 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start, rf_z_start + 0.05));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;

    start_pose.getPosition() << rf_x_start - step_dis / 2 + forward_d, rf_y_start, rf_z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::RF_LEG, start_joints, start_joints, "OUT_LEFT");

    final_pose.getPosition() << rf_x_start + step_dis / 6 + forward_d, rf_y_start, rf_z_start + height_2 + 0.05;// not move too long.
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::RF_LEG, final_joints, final_joints, "OUT_LEFT");

    j = 8;
    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_rf[j].push_back(0.3 * period_t);
    rf_leg_position[j].push_back(valuetype(final_joints.x(), -M_PI_2, M_PI));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 9;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(final_joints.x(), -M_PI_2, M_PI));

    times_of_rf[j].push_back(0.3 * period_t);
    rf_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 10;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start, rf_z_start + height_2 + 0.05));

    times_of_rf[j].push_back(0.2 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);
    //step 6: com moves to forward;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 - forward_d, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: lh leg moves;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 - forward_d, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 - forward_d, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    valuetype pos_final;
    pos_final << rf_x_start - step_dis / 6 - forward_d, rf_y_start, rf_z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            if(step == 1 || step == 2)
            {
                if(dt > (2 * period_t + 3 * adjust_t) && dt <=(2.2 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
                }else if(dt > (2.2 * period_t + 3 * adjust_t) && dt <=(2.5 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.2 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (2.5 * period_t + 3 * adjust_t) && dt <=(2.8 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.5 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (2.8 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.8 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (2 * period_t + 3 * adjust_t) && dt <= (2.25 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
                }else if(dt > (2.25 * period_t + 3 * adjust_t) && dt <(2.75 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.25 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }if(dt > (2.75 * period_t + 3 * adjust_t) && dt <(3 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.75 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 1 || step == 2)
        {
            if(dt > (2.2 * period_t + 3 * adjust_t) && dt <= (2.8 * period_t + 3 * adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::RF_LEG, rf_leg_pose);
            }else {
                rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");
            }
        }else{
            rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");
        }
        rf_joint_positions.push_back(joints);
        rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    std::cout << "finish rf step 1" << std::endl;
    return pos_final;

}
valuetype generate_lh_motion_1(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = -height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_num;
    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_lh.resize(step_num);
    lh_leg_position.resize(step_num);
    lh_leg_trajectory.resize(step_num);

    unsigned long j = 0;

    //step 1: com moves back;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 1  lf leg moves

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: com moves back
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: rf leg moves
    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: com moves forward
    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: lh leg moves

    j = 7;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 8;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.5 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 9;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + step_dis / 2 - forward_d, y_start, z_start;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.2 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.5 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.2 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.5 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.5 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (3.8 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.8 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.25 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.25 * period_t + 4 * adjust_t) && dt <=(3.75 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.25 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.75 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.75 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if((dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t)))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LH_LEG, lh_leg_pose);
            }else{
                lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
            }
        }else{
            lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        }
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(valuetype(lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rh_motion_1(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = -height_x;
    y_start = -height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_num;

    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 0: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: rh leg moves
    if( step == 1 || step == 2)
    {
        j = 3;

        times_of_rh[j].push_back(0);
        rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start));

        times_of_rh[j].push_back(0.25 * period_t);
        rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start + height_1));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

        j = 8;

        times_of_rh[j].push_back(0);
        rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start + height_1));

        times_of_rh[j].push_back(0.5 * period_t);
        rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_1));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

        j = 9;

        times_of_rh[j].push_back(0);
        rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_1));

        times_of_rh[j].push_back(0.25 * period_t);
        rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

        z_start = z_start - height_2;
    }else{

        j = 3;

        times_of_rh[j].push_back(0);
        rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start));

        times_of_rh[j].push_back(0.2 * period_t);
        rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start, z_start + 0.05));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

        std::cout << "the position is " << x_start - step_dis / 2 - forward_d << std::endl;

        j = 8;


        Pose start_pose, final_pose;
        JointPositionsLimb start_joints, final_joints;
        start_pose.getPosition() << x_start - step_dis / 2 - forward_d, y_start, z_start + 0.05;
        QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::RH_LEG, start_joints, start_joints, "IN_LEFT");


        final_pose.getPosition() << x_start + step_dis / 2 - forward_d, y_start, z_start + height_2 + 0.05;
        QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::RH_LEG, final_joints, final_joints, "IN_LEFT");

        times_of_rh[j].push_back(0.0);
        rh_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

        times_of_rh[j].push_back(0.3 * period_t);
        rh_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

        valuetype test;
        rh_leg_trajectory[j].evaluate(test, 0);

        JointPositionsLimb joint_test;
        joint_test << test.x(), test.y(), test.z();
        Pose pose_test;
        QK.FowardKinematicsSolve(joint_test,LimbEnum::RH_LEG,pose_test);
        std::cout << "pose_test is " << pose_test.getPosition() << std::endl;

        j = 9;

        times_of_rh[j].push_back(0.0);
        rh_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

        times_of_rh[j].push_back(0.3 * period_t);
        rh_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

        j = 10;

        times_of_rh[j].push_back(0);
        rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_2 + 0.05));

        times_of_rh[j].push_back(0.2 * period_t);
        rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_2));

        rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);


    }

    //step 4: com moves back;
    j = 4;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 5: rf leg moves;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 6 - forward_d, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.5 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.2 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.5 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.5 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (1.8 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.8 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.25 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.75 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {            
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::RH_LEG, rh_leg_pose);
            }else{
                rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
                }
        }else{
            rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        }
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}


valuetype generate_lf_motion_2(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    //height 2 is the height of the stairs
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;
    step = 2;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 11;

    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    //step 1: move back;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves by rolling 360 degree;
    double walking_dis = 0.4;
    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_lf[j].push_back(0.2 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start + 0.05));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;

    start_pose.getPosition() << x_start + 2 * forward_d, y_start, z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::LF_LEG, start_joints, start_joints, "IN_LEFT");

//    std::cout << " the start pos is " << start_joints << std::endl;


    final_pose.getPosition() << x_start + 2 * forward_d + walking_dis, y_start, z_start + height_2 + 0.05;
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::LF_LEG, final_joints, final_joints, "IN_LEFT");

//    std::cout << " the start pos is " << final_joints << std::endl;


    j = 8;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_lf[j].push_back(0.3 * period_t);
    lf_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 9;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    times_of_lf[j].push_back(0.3 * period_t);
    lf_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

//    std::cout << "final joint_1 is " << final_joints << std::endl;

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 10;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis, y_start, z_start + height_2 + 0.05));

    times_of_lf[j].push_back(0.2 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 3: com moves foward;
    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis, y_start, z_start + height_2));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis, y_start, z_start + height_2));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis - 0.1, y_start, z_start + height_2));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis - 0.1, y_start, z_start + height_2));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis - 0.1, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5: rf leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis - 0.1, y_start, z_start + height_2));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis - 0.2, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: come moves forward;

    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + walking_dis - 0.2, y_start, z_start + height_2));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis - 0.2, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: lh leg moves;

    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis - 0.2, y_start, z_start + height_2));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + walking_dis - 0.3, y_start, z_start + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

//    final_pose.getPosition() << x_start + 0 * forward_d + walking_dis - 0.3, y_start, z_start + height_2;
//    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::LF_LEG, final_joints, final_joints, "OUT_LEFT");

    valuetype pos_final;
    pos_final << x_start + walking_dis - 0.1 - 0.1 - 0.1, y_start, z_start + height_2;


    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            if(step == 1 || step == 2)
            {
                if(dt > adjust_t && dt <= (0.2 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.2 * period_t + adjust_t) && dt <= (0.5 * period_t + adjust_t))
                {
                    dt_ = dt - (0.2 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (0.5 * period_t + adjust_t) && dt <= (0.8 * period_t + adjust_t))
                {
                    dt_ = dt - (0.5 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (0.8 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.8 * period_t + adjust_t);
                    lf_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }
            }else {// in plain
                if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
                {
                    dt_ = dt - (0.25 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.75 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(dt <= (adjust_t + 0.2 * period_t))
        {
            lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        }else if(dt > (0.2 * period_t + adjust_t) && dt <= ( 0.8 * period_t + adjust_t))
        {
            joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.FowardKinematicsSolve(joints,LimbEnum::LF_LEG, lf_leg_pose);
        }else {
            lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        }

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rf_motion_2(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    double rf_x_start = height_x;
    double rf_y_start = height_y;
    double rf_z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_num;
    if(step == 1 || step == 2)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }
    rf_leg_trajectory.resize(step_num);
    rf_leg_position.resize(step_num);
    times_of_rf.resize(step_num);

    unsigned long j;

    //step 0 : com moves to back;
    j = 0;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: com moves to forward
    j = 2;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.1, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.1, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: com moves to back
    j = 4;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: rf leg moves

    j = 5;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(0.2 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start + 0.05));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;

    double walking_dis;
    walking_dis = 0.3;

    start_pose.getPosition() << rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::RF_LEG, start_joints, start_joints, "OUT_LEFT");

    final_pose.getPosition() << rf_x_start + 2 * forward_d - 0.2 + walking_dis, rf_y_start, rf_z_start + height_2 + 0.05;// not move too long.
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::RF_LEG, final_joints, final_joints, "OUT_LEFT");

    j = 8;
    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_rf[j].push_back(0.3 * period_t);
    rf_leg_position[j].push_back(valuetype(final_joints.x(), -M_PI_2, M_PI));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 9;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(final_joints.x(), -M_PI_2, M_PI));

    times_of_rf[j].push_back(0.3 * period_t);
    rf_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 10;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2 + walking_dis, rf_y_start, rf_z_start + height_2 + 0.05));

    times_of_rf[j].push_back(0.2 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2 + walking_dis, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);


    //step 6: com moves to forward;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2 + walking_dis, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2 + walking_dis, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: lh leg moves;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2 + walking_dis, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2 + walking_dis - 0.1, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

//    Pose final_pos;
//    final_pos.getPosition() << rf_x_start + 2 * forward_d + 0, rf_y_start, rf_z_start + height_2;
//    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::RF_LEG, final_joints, final_joints, "IN_LEFT");
//    std::cout << "final_pos " << final_joints << std::endl;
    valuetype pos_final;
    pos_final << rf_x_start + 0 * forward_d - 0.2 + walking_dis - 0.1, rf_y_start, rf_z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            if(step == 1 || step == 2)
            {
                if(dt > (2 * period_t + 3 * adjust_t) && dt <=(2.2 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
                }else if(dt > (2.2 * period_t + 3 * adjust_t) && dt <=(2.5 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.2 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (2.5 * period_t + 3 * adjust_t) && dt <=(2.8 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.5 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (2.8 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
                {
                    dt_ = dt - (2.8 * period_t + 3 * adjust_t);
                    rf_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(dt <= (2.2 * period_t + 3 * adjust_t))
        {
            rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");

            rf_joint_positions.push_back(joints);
            rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));
        }else if(dt > (2.2 * period_t + 3 * adjust_t) && dt <= (2.8 * period_t + 3 * adjust_t))
        {
            joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.FowardKinematicsSolve(joints,LimbEnum::RF_LEG, rf_leg_pose);

            rf_joint_positions.push_back(joints);
            rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));
        }else {
            rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");

            rf_joint_positions.push_back(joints);
            rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));
        }

        dt = dt + delta_t;
    }

    return pos_final;

}
valuetype generate_lh_motion_2(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_num;
    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_lh.resize(step_num);
    lh_leg_position.resize(step_num);
    lh_leg_trajectory.resize(step_num);

    unsigned long j = 0;

    //step 1: com moves back;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 1  lf leg moves

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.2, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: com moves back
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.2, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.2, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: rf leg moves
    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.2, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.3, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: com moves forward
    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.3, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: lh leg moves

    j = 7;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 8;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.5 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.0, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 9;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.0, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.0, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + 0 * forward_d - 0.0, y_start, z_start;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.2 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.5 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.2 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.5 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.5 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (3.8 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.8 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.25 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.25 * period_t + 4 * adjust_t) && dt <=(3.75 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.25 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.75 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.75 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if((dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t)))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LH_LEG, lh_leg_pose);
            }else{
                lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
            }
        }else{
            lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        }
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(valuetype(lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rh_motion_2(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_num;

    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 0: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: rh leg moves
    j = 3;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 8;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start + height_1));

    times_of_rh[j].push_back(0.5 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 9;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_1));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 4: com moves back;
    j = 4;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 5: rf leg moves;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.0, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + 0 * forward_d + 0.0, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.5 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.2 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.5 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.5 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (1.8 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.8 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.25 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.75 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::RH_LEG, rh_leg_pose);
            }else{
                rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
            }
        }else{
            rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        }
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;

}

valuetype generate_lf_motion_3(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    //height 2 is the height of the stairs
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 10;

    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    //step 1: move back;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves by rolling 360 degree;
    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 8;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start + height_1));

    times_of_lf[j].push_back(0.5 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 9;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_1));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    z_start = z_start - height_2;

    //step 3: com moves foward;
    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d + 0.2, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d + 0.2, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d + 0.1, y_start, z_start));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d + 0.1, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5: rf leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: come moves forward;

    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d + 0, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: lh leg moves;

    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d + 0, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - 0 * forward_d - 0.1, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - 0 * forward_d - 0.1, y_start, z_start;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            if(step == 1 || step == 2)
            {
                if(dt > adjust_t && dt <= (0.2 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.2 * period_t + adjust_t) && dt <= (0.5 * period_t + adjust_t))
                {
                    dt_ = dt - (0.2 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (0.5 * period_t + adjust_t) && dt <= (0.8 * period_t + adjust_t))
                {
                    dt_ = dt - (0.5 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (0.8 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.8 * period_t + adjust_t);
                    lf_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }
            }else {// in plain
                if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
                {
                    dt_ = dt - (0.25 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.75 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 1 || step == 2)
        {
            if(dt > (0.2 * period_t + adjust_t) && dt <= ( 0.8 * period_t + adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LF_LEG, lf_leg_pose);
            }else {
                lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
            }
        }else {
            lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        }
        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rf_motion_3(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    double rf_x_start = height_x;
    double rf_y_start = height_y;
    double rf_z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_num;
    if(step == 1 || step == 2)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }
    rf_leg_trajectory.resize(step_num);
    rf_leg_position.resize(step_num);
    times_of_rf.resize(step_num);

    unsigned long j;

    //step 0 : com moves to back;
    j = 0;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: com moves to forward
    j = 2;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.1, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.1, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: com moves to back
    j = 4;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: rf leg moves

    j = 5;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 8;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start + height_1));

    times_of_rf[j].push_back(0.5 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 9;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start + height_1));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    rf_z_start = rf_z_start - height_2;


    //step 6: com moves to forward;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d + 0.1, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: lh leg moves;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d + 0.1, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d + 0.0, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    valuetype pos_final;
    pos_final << rf_x_start + 0 * forward_d + 0.0, rf_y_start, rf_z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {

            if(dt > (2 * period_t + 3 * adjust_t) && dt <= (2.25 * period_t + 3 * adjust_t))
            {
                dt_ = dt - (2 * period_t + 3 * adjust_t);
                rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
            }else if(dt > (2.25 * period_t + 3 * adjust_t) && dt <(2.75 * period_t + 3 * adjust_t))
            {
                dt_ = dt - (2.25 * period_t + 3 * adjust_t);
                rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
            }if(dt > (2.75 * period_t + 3 * adjust_t) && dt <(3 * period_t + 3 * adjust_t))
            {
                dt_ = dt - (2.75 * period_t + 3 * adjust_t);
                rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
            }

        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }


        rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");
        rf_joint_positions.push_back(joints);
        rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;

}
valuetype generate_lh_motion_3(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_num;
    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_lh.resize(step_num);
    lh_leg_position.resize(step_num);
    lh_leg_trajectory.resize(step_num);

    unsigned long j = 0;

    //step 1: com moves back;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 1  lf leg moves

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.2, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: com moves back
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.2, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.2, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: rf leg moves
    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.2, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.3, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: com moves forward
    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.3, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: lh leg moves

    j = 7;

    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start));

    times_of_lh[j].push_back(0.2 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start + 0.05));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);


    j = 8;
    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;
    start_pose.getPosition() << x_start + 0 * forward_d - 0.3, y_start, z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::LH_LEG, start_joints, start_joints, "OUT_LEFT");

    final_pose.getPosition() << x_start + 0 * forward_d - 0, y_start, z_start + height_2 + 0.05;
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::LH_LEG, final_joints, final_joints, "OUT_LEFT");

    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_lh[j].push_back(0.3 * period_t);
    lh_leg_position[j].push_back(valuetype(start_joints.x(), -M_PI_2, M_PI));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 9;

    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(start_joints.x(), -M_PI_2, M_PI));

    times_of_lh[j].push_back(0.3 * period_t);
    lh_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 10;

    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0, y_start, z_start + height_2 + 0.05));

    times_of_lh[j].push_back(0.2 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0, y_start, z_start + height_2));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + 0 * forward_d - 0, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.2 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.5 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.2 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.5 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.5 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (3.8 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.8 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.25 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.25 * period_t + 4 * adjust_t) && dt <=(3.75 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.25 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.75 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.75 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if((dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t)))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LH_LEG, lh_leg_pose);
            }else{
                lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
            }
        }else{
            lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        }
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(valuetype(lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rh_motion_3(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_num;

    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 0: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: rh leg moves

    j = 3;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    times_of_rh[j].push_back(0.2 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start + 0.05));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

//        std::cout << "the position is " << x_start - step_dis / 2 - forward_d << std::endl;

    j = 8;


    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;
    start_pose.getPosition() << x_start + 0 * forward_d - 0.1, y_start, z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::RH_LEG, start_joints, start_joints, "IN_LEFT");


    final_pose.getPosition() << x_start + 0 * forward_d + 0.2, y_start, z_start + height_2 + 0.05;
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::RH_LEG, final_joints, final_joints, "IN_LEFT");

    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_rh[j].push_back(0.3 * period_t);
    rh_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype test;
    rh_leg_trajectory[j].evaluate(test, 0);

    JointPositionsLimb joint_test;
    joint_test << test.x(), test.y(), test.z();
    Pose pose_test;
    QK.FowardKinematicsSolve(joint_test,LimbEnum::RH_LEG,pose_test);
    std::cout << "pose_test is " << pose_test.getPosition() << std::endl;

    j = 9;

    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    times_of_rh[j].push_back(0.3 * period_t);
    rh_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 10;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2 + 0.05));

    times_of_rh[j].push_back(0.2 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 4: com moves back;
    j = 4;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 5: rf leg moves;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.0, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + 0 * forward_d + 0.0, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.5 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.2 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.5 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.5 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (1.8 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.8 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.25 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.75 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::RH_LEG, rh_leg_pose);
            }else{
                rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
            }
        }else{
            rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        }
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;

}

valuetype generate_lf_motion_4(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    //height 2 is the height of the stairs
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 10;

    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    //step 1: move back;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves by rolling 360 degree;
    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 8;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start + height_1));

    times_of_lf[j].push_back(0.5 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 9;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_1));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    z_start = z_start - height_2;

    //step 3: com moves foward;
    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5: rf leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: come moves forward;

    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0, y_start, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: lh leg moves;

    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + 0 * forward_d - 0.1, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            if(step == 1 || step == 2)
            {
                if(dt > adjust_t && dt <= (0.2 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.2 * period_t + adjust_t) && dt <= (0.5 * period_t + adjust_t))
                {
                    dt_ = dt - (0.2 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (0.5 * period_t + adjust_t) && dt <= (0.8 * period_t + adjust_t))
                {
                    dt_ = dt - (0.5 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (0.8 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.8 * period_t + adjust_t);
                    lf_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }
            }else {// in plain
                if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
                {
                    dt_ = dt - adjust_t;
                    lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
                }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
                {
                    dt_ = dt - (0.25 * period_t + adjust_t);
                    lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
                {
                    dt_ = dt - (0.75 * period_t + adjust_t);
                    lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 1 || step == 2)
        {
            if(dt > (0.2 * period_t + adjust_t) && dt <= ( 0.8 * period_t + adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LF_LEG, lf_leg_pose);
            }else {
                lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
            }
        }else {
            lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        }

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }
    return pos_final;
}
valuetype generate_rf_motion_4(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    double rf_x_start = height_x;
    double rf_y_start = height_y;
    double rf_z_start = height_z;

    double step_dis = 0.30;

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_num;
    if(step == 1 || step == 2)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }
    rf_leg_trajectory.resize(step_num);
    rf_leg_position.resize(step_num);
    times_of_rf.resize(step_num);

    unsigned long j;

    //step 0 : com moves to back;
    j = 0;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: com moves to forward
    j = 2;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.1, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.1, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: com moves to back
    j = 4;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d - 0.2, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: rf leg moves

    j = 5;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 8;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d - 0.2, rf_y_start, rf_z_start + height_1));

    times_of_rf[j].push_back(0.5 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 9;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start + height_1));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    rf_z_start = rf_z_start - height_2;



    //step 6: com moves to forward;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 2 * forward_d + 0.1, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d + 0.1, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: lh leg moves;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d + 0.1, rf_y_start, rf_z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + 0 * forward_d + 0.0, rf_y_start, rf_z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    valuetype pos_final;
    pos_final << rf_x_start + 0 * forward_d + 0.0, rf_y_start, rf_z_start + height_2;


    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rf_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rf_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {

            if(dt > (2 * period_t + 3 * adjust_t) && dt <= (2.25 * period_t + 3 * adjust_t))
            {
                dt_ = dt - (2 * period_t + 3 * adjust_t);
                rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
            }else if(dt > (2.25 * period_t + 3 * adjust_t) && dt <(2.75 * period_t + 3 * adjust_t))
            {
                dt_ = dt - (2.25 * period_t + 3 * adjust_t);
                rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
            }if(dt > (2.75 * period_t + 3 * adjust_t) && dt <(3 * period_t + 3 * adjust_t))
            {
                dt_ = dt - (2.75 * period_t + 3 * adjust_t);
                rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
            }

        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }


        rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");

        rf_joint_positions.push_back(joints);
        rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;

}
valuetype generate_lh_motion_4(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_num;
    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_lh.resize(step_num);
    lh_leg_position.resize(step_num);
    lh_leg_trajectory.resize(step_num);

    unsigned long j = 0;

    //step 1: com moves back;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    std::cout << "z_start is "<< z_start << std::endl;

    //step 1  lf leg moves

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: rh leg moves
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.2, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: com moves back
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.2, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.2, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: rf leg moves
    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.2, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.3, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: com moves forward
    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.3, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: lh leg moves

    j = 7;

    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start));

    times_of_lh[j].push_back(0.2 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.3, y_start, z_start + 0.05));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);


    j = 8;
    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;
    start_pose.getPosition() << x_start + 0 * forward_d - 0.3, y_start, z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::LH_LEG, start_joints, start_joints, "OUT_LEFT");

    final_pose.getPosition() <<x_start + 0 * forward_d - 0.0, y_start, z_start + height_2 + 0.05;
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::LH_LEG, final_joints, final_joints, "OUT_LEFT");

    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_lh[j].push_back(0.3 * period_t);
    lh_leg_position[j].push_back(valuetype(start_joints.x(), -M_PI_2, M_PI));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 9;

    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(start_joints.x(), -M_PI_2, M_PI));

    times_of_lh[j].push_back(0.3 * period_t);
    lh_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 10;

    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.0, y_start, z_start + height_2 + 0.05));

    times_of_lh[j].push_back(0.2 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.0, y_start, z_start + height_2));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);


    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.2 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.5 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.2 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.5 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.5 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (3.8 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.8 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.25 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[7].evaluate(evaluate_value, dt_);
                }else if(dt > (3.25 * period_t + 4 * adjust_t) && dt <=(3.75 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.25 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (3.75 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
                {
                    dt_ = dt - (3.75 * period_t + 4 * adjust_t);
                    lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if((dt > (3.2 * period_t + 4 * adjust_t) && dt <=(3.8 * period_t + 4 * adjust_t)))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::LH_LEG, lh_leg_pose);
            }else{
                lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
            }
        }else{
            lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        }
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(valuetype(lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }
}
valuetype generate_rh_motion_4(int step, double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_num;

    if(step == 3 || step == 4)
    {
        step_num = 11;
    }else {
        step_num = 10;
    }

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 0: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 1: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: com moves forward
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d - 0.1, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: rh leg moves

    j = 3;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start));

    times_of_rh[j].push_back(0.2 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d - 0.1, y_start, z_start + 0.05));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 8;


    Pose start_pose, final_pose;
    JointPositionsLimb start_joints, final_joints;
    start_pose.getPosition() << x_start + 0 * forward_d - 0.1, y_start, z_start + 0.05;
    QK.InverseKinematicsSolve(start_pose.getPosition(), LimbEnum::RH_LEG, start_joints, start_joints, "IN_LEFT");


    final_pose.getPosition() << x_start + 0 * forward_d + 0.2, y_start, z_start + height_2 + 0.05;
    QK.InverseKinematicsSolve(final_pose.getPosition(), LimbEnum::RH_LEG, final_joints, final_joints, "IN_LEFT");

    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(start_joints.x(), start_joints.y(), start_joints.z()));

    times_of_rh[j].push_back(0.3 * period_t);
    rh_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 9;

    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(start_joints.x(), M_PI_2, -M_PI));

    times_of_rh[j].push_back(0.3 * period_t);
    rh_leg_position[j].push_back(valuetype(final_joints.x(), final_joints.y(), final_joints.z()));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 10;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2 + 0.05));

    times_of_rh[j].push_back(0.2 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 4: com moves back;
    j = 4;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.2, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 5: rf leg moves;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.2, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 2 * forward_d + 0.1, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.1, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + 0 * forward_d + 0.0, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + 0 * forward_d + 0.0, y_start, z_start + height_2;

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose rh_leg_pose;
    JointPositionsLimb joints;

    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rh_leg_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rh_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            if(step == 3 || step == 4)
            {
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.5 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.2 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.5 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.5 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }else if(dt > (1.8 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.8 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[10].evaluate(evaluate_value, dt_);
                }

            }else{
                if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (period_t + 2 * adjust_t);
                    rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
                }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.25 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
                }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
                {
                    dt_ = dt - (1.75 * period_t + 2 * adjust_t);
                    rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
                }
            }
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        if(step == 3 || step == 4)
        {
            if(dt > (1.2 * period_t + 2 * adjust_t) && dt <=(1.8 * period_t + 2 * adjust_t))
            {
                joints << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.FowardKinematicsSolve(joints,LimbEnum::RH_LEG, rh_leg_pose);
            }else{
                rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
                QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
            }
        }else{
            rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
            QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        }
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;

}

valuetype lf_leg_stand_up(double& stand_dis, valuetype& lf_leg_pos, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = lf_leg_pos.x();
    double y_start = lf_leg_pos.y();
    double z_start = lf_leg_pos.z();

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 1;

    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start, y_start, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start, y_start, z_start + stand_dis));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype evaluate_value;
    Pose lf_leg_pose;
    JointPositionsLimb joints;

    double dt = 0;
    while(dt < period_t)
    {
        lf_leg_trajectory[0].evaluate(evaluate_value, dt);

        lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    valuetype final_pos;
    final_pos << x_start, y_start, z_start + stand_dis;
    return final_pos;
}
valuetype rf_leg_stand_up(double& stand_dis, valuetype& rf_leg_pos, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_num;
    step_num = 1;

    rf_leg_trajectory.resize(step_num);
    rf_leg_position.resize(step_num);
    times_of_rf.resize(step_num);

    unsigned long j;

    //step 0 : com moves to back;
    j = 0;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_leg_pos.x(), rf_leg_pos.y(), rf_leg_pos.z()));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_leg_pos.x(), rf_leg_pos.y(), rf_leg_pos.z() + stand_dis));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    double dt = 0;

    valuetype evaluate_value;
    Pose rf_leg_pose;
    JointPositionsLimb joints;

    while(dt < period_t)
    {
        rf_leg_trajectory[0].evaluate(evaluate_value, dt);

        rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");

        rf_joint_positions.push_back(joints);
        rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    valuetype final_pos;
    final_pos << rf_leg_pos.x(), rf_leg_pos.y(), rf_leg_pos.z() + stand_dis;

    return final_pos;

}
valuetype lh_leg_stand_up(double& stand_dis, valuetype& lh_leg_pos, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_num;
    step_num = 1;

    times_of_lh.resize(step_num);
    lh_leg_position.resize(step_num);
    lh_leg_trajectory.resize(step_num);

    unsigned long j = 0;

    //step 1: com moves back;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(lh_leg_pos.x(), lh_leg_pos.y(), lh_leg_pos.z()));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(lh_leg_pos.x(), lh_leg_pos.y(), lh_leg_pos.z() + stand_dis));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    double dt = 0;
    double dt_ = 0;

    valuetype evaluate_value;
    Pose lh_leg_pose;
    JointPositionsLimb joints;

    while(dt < period_t)
    {
        lh_leg_trajectory[0].evaluate(evaluate_value, dt);

        lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");

        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(valuetype(lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    valuetype final_pos;
    final_pos << lh_leg_pos.x(), lh_leg_pos.y(), lh_leg_pos.z() + stand_dis;
    return final_pos;
}
valuetype rh_leg_stand_up(double& stand_dis, valuetype& lh_leg_pos, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_num;
    step_num = 1;

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 0: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(lh_leg_pos.x(), lh_leg_pos.y(), lh_leg_pos.z()));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(lh_leg_pos.x(), lh_leg_pos.y(), lh_leg_pos.z() + stand_dis));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype evaluate_value;
    Pose rh_leg_pose;
    JointPositionsLimb joints;
    double dt;
    dt = 0;

    while(dt <= period_t)
    {
        rh_leg_trajectory[0].evaluate(evaluate_value, dt);

        rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");

        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    valuetype final_pos;
    final_pos << lh_leg_pos.x(), lh_leg_pos.y(), lh_leg_pos.z() + stand_dis;
    return final_pos;
}

int main(int argc, char **argv)
{
    int step;


    step = 1;
    double height_1 = 0.15;
    double body_height = 0.05;
    double height_x = 0.382653;
    double height_y = 0.305;
    double height_z = -0.48;

    std::vector<JointPositionsLimb> rh_joint_positions, rf_joint_positions, lh_joint_positions, lf_joint_positions;
    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;

    //step 1: lf,rf leg moves to the stairs, so the height-1 = 0; height_2 = stairs_height
    // lh, rh leg moves in plain, so the height 1 = 0.15, height_2 = 0;
    double height_2 = 0.08;
    valuetype lf_pos_1 = generate_lf_motion_1(step, height_1, height_2, height_x, height_y, height_z, lf_joint_positions, lf_foot_height);
    valuetype rf_pos_1 = generate_rf_motion_1(step, height_1, height_2, height_x, height_y, height_z, rf_joint_positions, rf_foot_height);
    valuetype rh_pos_1 = generate_rh_motion_1(step, height_1, 0, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    valuetype lh_pos_1 = generate_lh_motion_1(step, height_1, 0, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);

    //the final state of step 1 is lf, rh height is z_start + height_2;
    //rh, lh leg in the plain;

    //in the step 2; lf, rf now on the first stairs, so the start position is z_start + height_2, the final position is the z_start + height_2 + height_2;
    //rh, lh leg in the plain;
    step = 2;
    height_2 = 0.13;

    valuetype lf_pos_2 = generate_lf_motion_2(step, height_1, height_2, lf_pos_1.x(), lf_pos_1.y(), lf_pos_1.z(), lf_joint_positions, lf_foot_height);
    valuetype rf_pos_2 = generate_rf_motion_2(step, height_1, height_2, rf_pos_1.x(), rf_pos_1.y(), rf_pos_1.z(), rf_joint_positions, rf_foot_height);
    valuetype rh_pos_2 = generate_rh_motion_2(step, height_1, 0, rh_pos_1.x(), rh_pos_1.y(), rh_pos_1.z(), rh_joint_positions, rh_foot_height);
    valuetype lh_pos_2 = generate_lh_motion_2(step, height_1, 0, lh_pos_1.x(), lh_pos_1.y(), lh_pos_1.z(), lh_joint_positions, lh_foot_height);

    //in the step 3: lf, rf now on the second stairs, so the stasrt position is z_start + height_2 + height_2
    //rh, lh leg on the plain; and they will go upstairs;
    step = 3;
    height_2 = 0.08;

    valuetype lf_pos_3 = generate_lf_motion_3(step, height_1, 0, lf_pos_2.x(), lf_pos_2.y(), lf_pos_2.z(), lf_joint_positions, lf_foot_height);
    valuetype rf_pos_3 = generate_rf_motion_3(step, height_1, 0, rf_pos_2.x(), rf_pos_2.y(), rf_pos_2.z(), rf_joint_positions, rf_foot_height);
    valuetype rh_pos_3 = generate_rh_motion_3(step, height_1, height_2, rh_pos_2.x(), rh_pos_2.y(), rh_pos_2.z(), rh_joint_positions, rh_foot_height);
    valuetype lh_pos_3 = generate_lh_motion_3(step, height_1, height_2, lh_pos_2.x(), lh_pos_2.y(), lh_pos_2.z(), lh_joint_positions, lh_foot_height);

    //all legs stand up
    double stand_dis = -0.08;
    valuetype lf_stand_pos = lf_leg_stand_up(stand_dis,lf_pos_3, lf_joint_positions, lf_foot_height);
    valuetype rf_stand_pos = rf_leg_stand_up(stand_dis,rf_pos_3, rf_joint_positions, rf_foot_height);
    valuetype lh_stand_pos = lh_leg_stand_up(stand_dis,lh_pos_3, lh_joint_positions, lh_foot_height);
    valuetype rh_stand_pos = rh_leg_stand_up(stand_dis,rh_pos_3, rh_joint_positions, rh_foot_height);


    //in the step 4: lf, rf now on the second stairs, so the stasrt position is z_start + height_2
    //rh, lh leg on the first plain; and they will go upstairs;

    step = 4;
    height_2 = 0.13;

    valuetype lf_pos_4 = generate_lf_motion_4(step, height_1, 0, lf_stand_pos.x(), lf_stand_pos.y(), lf_stand_pos.z(), lf_joint_positions, lf_foot_height);
    valuetype rf_pos_4 = generate_rf_motion_4(step, height_1, 0, rf_stand_pos.x(), rf_stand_pos.y(), rf_stand_pos.z(), rf_joint_positions, rf_foot_height);
    valuetype rh_pos_4 = generate_rh_motion_4(step, height_1, height_2, rh_stand_pos.x(), rh_stand_pos.y(), rh_stand_pos.z(), rh_joint_positions, rh_foot_height);
    valuetype lh_pos_4 = generate_lh_motion_4(step, height_1, height_2, lh_stand_pos.x(), lh_stand_pos.y(), lh_stand_pos.z(), lh_joint_positions, lh_foot_height);

    quadruped_model::JointPositions joint_positions;
    std::vector<quadruped_model::JointPositions> joint_positions_total;

    for(unsigned int i = 0; i < lf_joint_positions.size(); i++)
    {
        joint_positions(0) = lf_joint_positions[i](0);
        joint_positions(1) = lf_joint_positions[i](1);
        joint_positions(2) = lf_joint_positions[i](2);

        joint_positions(6) = rh_joint_positions[i](0);
        joint_positions(7) = rh_joint_positions[i](1);
        joint_positions(8) = rh_joint_positions[i](2);

        joint_positions(3) = rf_joint_positions[i](0);
        joint_positions(4) = rf_joint_positions[i](1);
        joint_positions(5) = rf_joint_positions[i](2);

        joint_positions(9) = lh_joint_positions[i](0);
        joint_positions(10) = lh_joint_positions[i](1);
        joint_positions(11) = lh_joint_positions[i](2);

        joint_positions_total.push_back(joint_positions);

    }
    std::cout << "size is " << joint_positions_total.size() << std::endl;
    std::string file_name = "rolling_to_the_stairs.txt";
    SaveAsFile(file_name, joint_positions_total);
    file_name = "rolling_to_the_stairs_read.txt";
    SaveAsFile_with_footheight(file_name, joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);
    return 1;
}
