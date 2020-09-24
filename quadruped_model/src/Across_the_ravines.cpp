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


using namespace std;
using namespace quadruped_model;
QuadrupedKinematics QK;

typedef typename curves::Time Time;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType valuetype;
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

double period_t = 5.8;
double adjust_t = 2.4;

double delta_t = 0.0025;

double height_x = 0.382653;
double height_y = 0.305;
double height_z = -0.50;

double forward_d = 0.08;
double y_direction = 0.05;
double adjust_height = 0.03;

double step_dis = 0.3;

valuetype generate_lf_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 10;


    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    //step 1: com move back and right;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves;
    double small_dis = 0.05;

    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lf[j].push_back(0.5 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d + small_dis, y_start + y_direction, z_start - adjust_height + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d + small_dis, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d + small_dis, y_start + y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);


    //step 3: com moves forward and left;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d + small_dis, y_start + y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d + small_dis, y_start - y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d + small_dis, y_start - y_direction, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d + small_dis, y_start - y_direction, z_start));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back and keep in left;
    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d + small_dis, y_start - y_direction, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d + small_dis, y_start - y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: rf leg moves
    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d + small_dis, y_start - y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d + small_dis, y_start - y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: come moves forward and to right;

    j = 8;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d + small_dis, y_start - y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d + small_dis, y_start + y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 8: lh leg moves;

    j = 9;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d + small_dis, y_start + y_direction, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - 0.1 - forward_d + small_dis, y_start + y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 2 - forward_d - 0.1 + small_dis, y_start + y_direction, z_start;

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
        }else if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
        {
            dt_ = dt - adjust_t;
            lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
        {
            dt_ = dt - (0.25 * period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
        {
            dt_ = dt - (0.75 * period_t + adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }
    std::cout << "finish lf step 1" << std::endl;
    return pos_final;
}
valuetype generate_rf_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    double rf_x_start = height_x;
    double rf_y_start = -height_y;
    double rf_z_start = height_z;

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_num;
    step_num = 10;
    rf_leg_trajectory.resize(step_num);
    rf_leg_position.resize(step_num);
    times_of_rf.resize(step_num);

    unsigned long j;

    //step 1 : com moves to back and right
    j = 0;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 - forward_d, rf_y_start + y_direction, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start + y_direction, rf_z_start - adjust_height));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: lf leg moves

    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 + forward_d, rf_y_start + y_direction, rf_z_start - adjust_height));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 + forward_d, rf_y_start + y_direction, rf_z_start - adjust_height));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: com moves to forward and left
    j = 2;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 + forward_d, rf_y_start + y_direction, rf_z_start - adjust_height));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 - forward_d, rf_y_start - y_direction, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: rh leg moves
    j = 3;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 6 - forward_d, rf_y_start - y_direction, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 - forward_d, rf_y_start - y_direction, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: com moves to back and keep in left
    j = 4;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 - forward_d, rf_y_start - y_direction, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 6: rf leg moves

    j = 5;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 6;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height + height_1));

    times_of_rf[j].push_back(0.5 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height + height_1));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: com moves forward and right;
    j = 8;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 2 + forward_d, rf_y_start - y_direction, rf_z_start - adjust_height));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 2 - forward_d, rf_y_start + y_direction, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 8: lh leg moves;
    j = 9;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 2 - forward_d, rf_y_start + y_direction, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + step_dis / 6 - forward_d, rf_y_start + y_direction, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    valuetype pos_final;
    pos_final << rf_x_start + step_dis / 6 - forward_d, rf_y_start + y_direction, rf_z_start;

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
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(2.25 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2.25 * period_t + 3 * adjust_t) && dt <=(2.75 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2.25 * period_t + 3 * adjust_t);
            rf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2.75 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2.75 * period_t + 3 * adjust_t);
            rf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        rf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rf_leg_pose.getPosition(), LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");

        rf_joint_positions.push_back(joints);
        rf_foot_position.push_back(valuetype(rf_leg_pose.getPosition().x(), rf_leg_pose.getPosition().y(), rf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    std::cout << "finish rf step 1" << std::endl;
    return pos_final;

}
valuetype generate_lh_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
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
    step_num = 10;

    times_of_lh.resize(step_num);
    lh_leg_position.resize(step_num);
    lh_leg_trajectory.resize(step_num);

    unsigned long j = 0;

    //step 1: com moves back;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2:  lf leg moves

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start + y_direction, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: com moves forward and left
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start + y_direction, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: rh leg moves
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: com moves back and keep in left
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: rf leg moves
    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start - y_direction, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 7: com moves forward and to right;
    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start - y_direction, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: lh leg moves

    j = 7;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 8;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lh[j].push_back(0.5 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 9;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start + step_dis / 2 - forward_d, y_start + y_direction, z_start - adjust_height;

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
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3.25 * period_t + 4 * adjust_t))
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
        }else {
            std::cout << "nothing" << std::endl;
        }
        lh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");

        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(valuetype(lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rh_motion_1_2(double height_1, double height_2, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
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
    step_num = 10;

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 1: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start + y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start + y_direction, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: com moves forward and left
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 4: rh leg moves

    j = 3;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 4;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    times_of_rh[j].push_back(0.5 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 5;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);


    //step 5: com moves back and keep in left;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start - y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: rf leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start - y_direction, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward and move to right;
    j = 8;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 9;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height;

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
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.25 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.75 * period_t + 2 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }
        rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");

        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}

valuetype generate_lf_motion_2(double height_1, double height_2, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 10;


    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    double small_dis = 0.05;
    //step 1: com move back and right;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d - 0.1 + small_dis, y_start + y_direction, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d - 0.1 + small_dis, y_start + y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves;

    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d - 0.1 + small_dis, y_start + y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d - 0.1 + small_dis, y_start + y_direction, z_start - adjust_height + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d - 0.1 + small_dis, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lf[j].push_back(0.5 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);


    //step 3: com moves forward and left;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back and keep in left;
    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: rf leg moves
    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: come moves forward and to right;

    j = 8;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 8: lh leg moves;

    j = 9;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start;

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
        }else if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
        {
            dt_ = dt - adjust_t;
            lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
        {
            dt_ = dt - (0.25 * period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
        {
            dt_ = dt - (0.75 * period_t + adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }
    std::cout << "finish lf step 1" << std::endl;
    return pos_final;
}
valuetype generate_lf_motion_3_4(double height_1, double height_2, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_num = 10;


    times_of_lf.resize(step_num);
    lf_leg_position.resize(step_num);
    lf_leg_trajectory.resize(step_num);

    unsigned long j;

    //step 1: com move back and right;
    j = 0;
    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 2: lf leg moves;

    j = 1;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 2;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lf[j].push_back(0.5 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    j = 3;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height + height_1));

    times_of_lf[j].push_back(0.25 * period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);


    //step 3: com moves forward and left;
    j = 4;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 4:rh leg moves
    j = 5;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start));
    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 5:come moves back and keep in left;
    j = 6;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 6: rf leg moves
    j = 7;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 7: come moves forward and to right;

    j = 8;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_lf[j].push_back(adjust_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    //step 8: lh leg moves;

    j = 9;

    times_of_lf[j].push_back(0.0);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start));

    times_of_lf[j].push_back(period_t);
    lf_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 2 - forward_d, y_start + y_direction, z_start;

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
        }else if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
        {
            dt_ = dt - adjust_t;
            lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
        }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
        {
            dt_ = dt - (0.25 * period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
        }else if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
        {
            dt_ = dt - (0.75 * period_t + adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }

        lf_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(lf_leg_pose.getPosition(), LimbEnum::LF_LEG, joints, joints, "IN_LEFT");

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(valuetype(lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }
    std::cout << "finish lf step 1" << std::endl;
    return pos_final;
}

valuetype generate_rh_motion_3(double height_1, double height_2, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
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
    step_num = 10;

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 1: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start + y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start + y_direction, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: com moves forward and left
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d, y_start + y_direction, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 4: rh leg moves

    j = 3;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 4;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    times_of_rh[j].push_back(0.5 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 5;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);


    //step 5: com moves back and keep in left;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: rf leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward and move to right;
    j = 8;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d, y_start - y_direction, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 9;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d - 0.1, y_start + y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 6 - forward_d - 0.1, y_start + y_direction, z_start - adjust_height;

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
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.25 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.75 * period_t + 2 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }
        rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");

        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}
valuetype generate_rh_motion_4(double height_1, double height_2, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
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
    step_num = 10;

    times_of_rh.resize(step_num);
    rh_leg_position.resize(step_num);
    rh_leg_trajectory.resize(step_num);

    unsigned long j = 0;
    //step 1: com moves back;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d - 0.1, y_start + y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d - 0.1, y_start + y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 2: lf leg moves

    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 + forward_d - 0.1, y_start + y_direction, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d - 0.1, y_start + y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 3: com moves forward and left
    j = 2;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 + forward_d - 0.1, y_start + y_direction, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d - 0.1, y_start - y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 4: rh leg moves

    j = 3;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d - 0.1, y_start - y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d - 0.1, y_start - y_direction, z_start - adjust_height + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 4;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 2 - forward_d - 0.1, y_start - y_direction, z_start - adjust_height + height_1));

    times_of_rh[j].push_back(0.5 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 5;

    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height + height_1));

    times_of_rh[j].push_back(0.25 * period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);


    //step 5: com moves back and keep in left;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 - forward_d, y_start - y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start - y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: rf leg moves;
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 2 + forward_d, y_start - y_direction, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 6: com moves forward and move to right;
    j = 8;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 + forward_d, y_start - y_direction, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    ////step 7: lh leg moves;
    j = 9;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    valuetype pos_final;
    pos_final << x_start - step_dis / 6 - forward_d, y_start + y_direction, z_start - adjust_height;

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
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(1.25 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rh_leg_trajectory[3].evaluate(evaluate_value, dt_);
        }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.25 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
        }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.75 * period_t + 2 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[7].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[8].evaluate(evaluate_value, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rh_leg_trajectory[9].evaluate(evaluate_value, dt_);
        }else {
            std::cout << "nothing" << std::endl;
        }
        rh_leg_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();
        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(), LimbEnum::RH_LEG, joints, joints, "IN_LEFT");

        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(valuetype(rh_leg_pose.getPosition().x(), rh_leg_pose.getPosition().y(), rh_leg_pose.getPosition().z()));

        dt = dt + delta_t;
    }

    return pos_final;
}

int main(int argc, char **argv)
{
    double height_1 = 0.2;
    double height_2 = 0;
    std::vector<JointPositionsLimb> rh_joint_positions, rf_joint_positions, lh_joint_positions, lf_joint_positions;
    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;

    valuetype lf_pos_3 = generate_lf_motion(height_1, 0, lf_joint_positions, lf_foot_height);
    valuetype rf_pos_3 = generate_rf_motion(height_1, 0, rf_joint_positions, rf_foot_height);
    valuetype rh_pos_3 = generate_rh_motion_1_2(height_1, height_2, rh_joint_positions, rh_foot_height);
    valuetype lh_pos_3 = generate_lh_motion(height_1, height_2, lh_joint_positions, lh_foot_height);

    lf_pos_3 = generate_lf_motion_2(height_1, 0, lf_joint_positions, lf_foot_height);
    rf_pos_3 = generate_rf_motion(height_1, 0, rf_joint_positions, rf_foot_height);
    rh_pos_3 = generate_rh_motion_1_2(height_1, height_2, rh_joint_positions, rh_foot_height);
    lh_pos_3 = generate_lh_motion(height_1, height_2, lh_joint_positions, lh_foot_height);

    lf_pos_3 = generate_lf_motion_3_4(height_1, 0, lf_joint_positions, lf_foot_height);
    rf_pos_3 = generate_rf_motion(height_1, 0, rf_joint_positions, rf_foot_height);
    rh_pos_3 = generate_rh_motion_3(height_1, height_2, rh_joint_positions, rh_foot_height);
    lh_pos_3 = generate_lh_motion(height_1, height_2, lh_joint_positions, lh_foot_height);

    lf_pos_3 = generate_lf_motion_3_4(height_1, 0, lf_joint_positions, lf_foot_height);
    rf_pos_3 = generate_rf_motion(height_1, 0, rf_joint_positions, rf_foot_height);
    rh_pos_3 = generate_rh_motion_4(height_1, height_2, rh_joint_positions, rh_foot_height);
    lh_pos_3 = generate_lh_motion(height_1, height_2, lh_joint_positions, lh_foot_height);


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
    std::string file_name = "across_the_ravines.txt";
    SaveAsFile(file_name, joint_positions_total);
    file_name = "across_the_ravines_read.txt";
    SaveAsFile_with_footheight(file_name, joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);
    return 1;
}
