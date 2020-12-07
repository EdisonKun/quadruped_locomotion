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

//this file use the x configure and with wheel after 2 steps
//to avoid the front two legs interact with the stairs;


using namespace std;
using namespace quadruped_model;

Eigen::MatrixXd jacobian;
QuadrupedKinematics QK;

double Step_dis = 0.20;//step distance
double period_t = 6;//period 6
double adjust_t = period_t;//pose adjust time;

double knee_height = 0.00;//knee down height
double down_height = 0.00;//swing down height


double forward_d = 0.08;//adjust the COM

typedef typename curves::Time Time;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType valuetype;

double dt = 0;
double dt_ = 0;
double delta_t = 0.0025;

void SaveAsFile(const std::string &file_name, const std::vector<JointPositions>& joint_position_vector, const std::vector<valuetype>& lf_foot_height,
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
        time_ = time_ + delta_t;
    }
    invFile.close();
    std::cout << "success store the file " << std::endl;
}
void SaveAsFile_no_footheight(const std::string &file_name, const std::vector<JointPositions>& joint_position_vector)
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
        invFile <<time_ << " " << joint_position_vector[i];
        invFile << "\r\n";
        time_ = time_ + delta_t;
    }
    invFile.close();
    std::cout << "success store the file " << std::endl;
}


void generate_lf_motion(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double lf_x_start = height_x;
    double lf_y_start = height_y;
    double lf_z_start = height_z;

    std::cout << lf_z_start << std::endl;
    /****planning lf leg******/
    //step 1: RF leg up and RH leg down, LF and LH keep the prepare angle unchanged;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_total = 5;
    lf_leg_trajectory.resize(step_total);
    times_of_lf.resize(step_total);
    lf_leg_position.resize(step_total);

    // step 0, COM to the back;
    times_of_lf[0].push_back(0.0);
    lf_leg_position[0].push_back(valuetype(lf_x_start - Step_dis / 2 - forward_d, lf_y_start, lf_z_start));

    times_of_lf[0].push_back(adjust_t);
    lf_leg_position[0].push_back(valuetype(lf_x_start - Step_dis / 2 + forward_d, lf_y_start, lf_z_start));

    lf_leg_trajectory[0].fitCurve(times_of_lf[0], lf_leg_position[0]);

    //step 1:LF leg does not move;(RF leg moves)
    times_of_lf[1].push_back(0.0);
    lf_leg_position[1].push_back(valuetype(lf_x_start - Step_dis / 2 + forward_d, lf_y_start, lf_z_start));

    times_of_lf[1].push_back(period_t);
    lf_leg_position[1].push_back(valuetype(lf_x_start - Step_dis / 2 + forward_d, lf_y_start, lf_z_start));

    lf_leg_trajectory[1].fitCurve(times_of_lf[1], lf_leg_position[1]);

    //step 2:LF LEG does not move;(body moves forward)
    times_of_lf[2].push_back(0.0);
    lf_leg_position[2].push_back(valuetype(lf_x_start - Step_dis / 2 + forward_d, lf_y_start, lf_z_start));

    times_of_lf[2].push_back(period_t);
    lf_leg_position[2].push_back(valuetype(lf_x_start - Step_dis / 2 - forward_d, lf_y_start, lf_z_start));

    lf_leg_trajectory[2].fitCurve(times_of_lf[2], lf_leg_position[2]);

    //step 3:LF LEG does not move;(LH leg out_left -> in_left)

    times_of_lf[3].push_back(0.0);
    lf_leg_position[3].push_back(valuetype(lf_x_start - Step_dis / 2 - forward_d, lf_y_start, lf_z_start));

    times_of_lf[3].push_back(period_t);
    lf_leg_position[3].push_back(valuetype(lf_x_start - Step_dis / 2 - forward_d, lf_y_start, lf_z_start));

    lf_leg_trajectory[3].fitCurve(times_of_lf[3], lf_leg_position[3]);

    //step 4: LF leg does not move;(RH leg in_left -> out_left)

    times_of_lf[4].push_back(0.0);
    lf_leg_position[4].push_back(valuetype(lf_x_start - Step_dis / 2 - forward_d , lf_y_start, lf_z_start));

    times_of_lf[4].push_back(adjust_t);
    lf_leg_position[4].push_back(valuetype(lf_x_start - Step_dis / 2 - forward_d , lf_y_start, lf_z_start));

    lf_leg_trajectory[4].fitCurve(times_of_lf[4], lf_leg_position[4]);

    JointPositionsLimb joints;
    valuetype lf_leg_valuetype;
    Pose lf_leg_pose;

    dt = 0;
    dt_ = 0;
    /***** evaluate the leg position ******/

    while(dt <= 5 * period_t)
    {
        if(dt <= adjust_t)
        {
            lf_leg_trajectory[0].evaluate(lf_leg_valuetype, dt);
        }else if( dt > period_t && dt <= 2 * period_t)
        {
            dt_ = dt - adjust_t;
            lf_leg_trajectory[1].evaluate(lf_leg_valuetype, dt_);
        }else if( dt > 2 * period_t && dt <= 3 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 2 * period_t;
            lf_leg_trajectory[2].evaluate(lf_leg_valuetype, dt_);
        }else if( dt > 3 * period_t && dt <= 4 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 3 * period_t;
            lf_leg_trajectory[3].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > 4 * period_t && dt <= 5 * period_t)//adjust_t + period_t~ period_t + 2 * adjust_t
        {
            dt_ = dt - 4 * period_t;
            lf_leg_trajectory[4].evaluate(lf_leg_valuetype, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        lf_leg_pose.getPosition().x() = lf_leg_valuetype.x();
        lf_leg_pose.getPosition().y() = lf_leg_valuetype.y();
        lf_leg_pose.getPosition().z() = lf_leg_valuetype.z();

        QK.InverseKinematicsSolve(lf_leg_pose.getPosition(),LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(lf_leg_valuetype);/*
        std::cout << lf_joint_positions.size() << std::endl;*/
    }
    std::cout << "finish the lf_leg planning" << std::endl;
}
void generate_rh_motion_1(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    /*******planning the RH leg *******/
    double rh_x_start = -height_x;
    double rh_y_start = -height_y;
    double rh_z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_total = 4;
    times_of_rh.resize(step_total);
    rh_leg_position.resize(step_total);
    rh_leg_trajectory.resize(step_total);

    unsigned long j;
    //step 0: COM BACK;
    times_of_rh[0].push_back(0);
    rh_leg_position[0].push_back(valuetype(rh_x_start - Step_dis / 6 - forward_d, rh_y_start, rh_z_start));

    times_of_rh[0].push_back(adjust_t);
    rh_leg_position[0].push_back(valuetype(rh_x_start - Step_dis / 6 + forward_d, rh_y_start, rh_z_start));

    rh_leg_trajectory[0].fitCurve(times_of_rh[0], rh_leg_position[0]);

    //step 1: rh does not move(rf moves);
    times_of_rh[1].push_back(0);
    rh_leg_position[1].push_back(valuetype(rh_x_start - Step_dis / 6 + forward_d, rh_y_start, rh_z_start));

    times_of_rh[1].push_back(period_t);
    rh_leg_position[1].push_back(valuetype(rh_x_start - Step_dis / 6 + forward_d, rh_y_start, rh_z_start));

    rh_leg_trajectory[1].fitCurve(times_of_rh[1], rh_leg_position[1]);//5s

    //step 2: rh does not move(body moves forward)
    times_of_rh[2].push_back(0);
    rh_leg_position[2].push_back(valuetype(rh_x_start - Step_dis / 6 + forward_d, rh_y_start, rh_z_start));

    times_of_rh[2].push_back(adjust_t);
    rh_leg_position[2].push_back(valuetype(rh_x_start - Step_dis / 6 - forward_d, rh_y_start, rh_z_start));

    rh_leg_trajectory[2].fitCurve(times_of_rh[2], rh_leg_position[2]);//6.2s

    //step 3: rh does not move(body moves forward)
    times_of_rh[3].push_back(0);
    rh_leg_position[3].push_back(valuetype(rh_x_start - Step_dis / 6 - forward_d, rh_y_start, rh_z_start));

    times_of_rh[3].push_back(adjust_t);
    rh_leg_position[3].push_back(valuetype(rh_x_start - Step_dis / 6 - forward_d, rh_y_start, rh_z_start));

    rh_leg_trajectory[3].fitCurve(times_of_rh[3], rh_leg_position[3]);//6.2s

    //step 4: rh moves;
    //step 3-1: lh moves.
    //get the initial joint angle;
    Pose rh_leg_pose_initial;
    valuetype rh_leg_joint_initial;
    JointPositionsLimb joints_initial;
    rh_leg_pose_initial.getPosition().x() = rh_x_start - Step_dis / 6 - forward_d;
    rh_leg_pose_initial.getPosition().y() = rh_y_start;
    rh_leg_pose_initial.getPosition().z() = rh_z_start;
    QK.InverseKinematicsSolve(rh_leg_pose_initial.getPosition(),LimbEnum::RH_LEG, joints_initial, joints_initial, "IN_LEFT");

    Pose rh_leg_pose_final;
    valuetype rh_leg_joint_final;
    JointPositionsLimb joints_final;
    rh_leg_pose_final.getPosition().x() = rh_x_start - Step_dis / 2 - forward_d;
    rh_leg_pose_final.getPosition().y() = rh_y_start;
    rh_leg_pose_final.getPosition().z() = rh_z_start;

    QK.InverseKinematicsSolve(rh_leg_pose_final.getPosition(), LimbEnum::RH_LEG, joints_final, joints_final, "OUT_LEFT");

    std::vector<valuetype> joint_interpolation;
    std::vector<Time> joint_time_inter;
    curves::PolynomialSplineQuinticVector3Curve joint_trajectory;

    valuetype joint_middle;
    joint_middle.x() = joints_initial.x();
    joint_middle.y() = joints_initial.y();
    joint_middle.z() = joints_initial.z();

    joint_time_inter.push_back(0.0);
    joint_interpolation.push_back(joint_middle);

//    joints_initial << 0, 0 ,(joints_final.z() - 2 * M_PI + joints_initial.z()) / 2;
//    joint_middle.x() = joints_initial.x();
//    joint_middle.y() = joints_initial.y();
//    joint_middle.z() = joints_initial.z();

//    joint_time_inter.push_back(period_t / 2.0);
//    joint_interpolation.push_back(joint_middle);

    joints_initial << joints_final.x(), joints_final.y(), joints_final.z() - 2 * M_PI;
    joint_middle.x() = joints_initial.x();
    joint_middle.y() = joints_initial.y();
    joint_middle.z() = joints_initial.z();

    joint_time_inter.push_back(period_t);
    joint_interpolation.push_back(joint_middle);

    joint_trajectory.fitCurve(joint_time_inter, joint_interpolation);


    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 5 * period_t)
    {
        if(dt <= period_t)
        {
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt);
        }else if(dt > period_t && dt <= 2 * period_t)
        {
            dt_ = dt - period_t;
            rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }
        else if(dt > 2 * period_t && dt <= 3 * period_t)
        {
            dt_ = dt - 2 * period_t;
            rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }
        else if( dt > 3 * period_t && dt <= 4 * period_t)
        {
            dt_ = dt - 3 * period_t;
            rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
        }else if (dt > 4 * period_t && dt <= 5 * period_t)
        {
            if(dt == 5 * period_t)
            {
                std::cout << "leohou -2" << std::endl;
            }
        }else{
            std::cout << "nothing" << std::endl;
        }

        rh_leg_pose.getPosition().x() = rh_leg_joint.x();
        rh_leg_pose.getPosition().y() = rh_leg_joint.y();
        rh_leg_pose.getPosition().z() = rh_leg_joint.z();

        if(dt <= 4 * period_t)
        {
            QK.InverseKinematicsSolve(rh_leg_pose.getPosition(),LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        }else if(dt > 4 * period_t && dt <= 5 * period_t)
        {
            dt_ = dt - 4 * period_t;
            joint_trajectory.evaluate(rh_leg_joint, dt_);
            joints.x() = rh_leg_joint.x();
            joints.y() = rh_leg_joint.y();
            joints.z() = rh_leg_joint.z();

            QK.FowardKinematicsSolve(joints, LimbEnum::RH_LEG, rh_leg_pose);

            rh_leg_joint.x() = rh_leg_pose.getPosition().x();
            rh_leg_joint.y() = rh_leg_pose.getPosition().y();
            rh_leg_joint.z() = rh_leg_pose.getPosition().z();
        }
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(rh_leg_joint);
        dt = dt + delta_t;
    }
    std::cout << "finish the RH_leg planning" << std::endl;
}
void generate_rf_motion_1(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    //    std::cout << "rf_joint_position size is " <<rf_joint_positions.size() << std::endl;
    double rf_x_start = height_x;
    double rf_y_start = -height_y;
    double rf_z_start = height_z;
    /*******planning the RF leg *******/

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;

    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_total = 7;
    rf_leg_trajectory.resize(step_total);
    rf_leg_position.resize(step_total);
    times_of_rf.resize(step_total);

    unsigned long j;

    //step 0: COM moves to back
    j = 0;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + Step_dis / 6 - forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + Step_dis / 6 + forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 1-1: rf leg moves forward
    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + Step_dis / 6 + forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start + Step_dis / 6 + forward_d, rf_y_start, rf_z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 1-2: RF moves swing;
    j = 2;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start + Step_dis / 6 + forward_d, rf_y_start, rf_z_start + height_1));

    times_of_rf[j].push_back(0.5 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 + forward_d, rf_y_start, rf_z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);


    //step 1-3
    j = 3;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 + forward_d, rf_y_start, rf_z_start + height_1));

    times_of_rf[j].push_back(0.25 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 + forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//

    //step 2 body moves forward

    j = 4;
    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 + forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//12s

    //step 3: rf leg does not move(lh leg moves);
    j = 5;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: rf does not move (rh leg moves);
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(rf_x_start - Step_dis / 2 - forward_d, rf_y_start, rf_z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    /***** evaluate the leg position ******/
    Pose rf_leg_pose;
    valuetype rf_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 5 * period_t )
    {
        if(dt <= period_t)
        {
            rf_leg_trajectory[0].evaluate(rf_leg_joint, dt);
        }else if(dt > period_t && dt <=1.25 * period_t)
        {
            dt_ = dt - period_t;
            rf_leg_trajectory[1].evaluate(rf_leg_joint, dt_);
        }else if( dt > 1.25 * period_t && dt <= 1.75 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 1.25 * period_t;
            rf_leg_trajectory[2].evaluate(rf_leg_joint, dt_);
        }else if( dt > 1.75 * period_t && dt <= 2 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 1.75 * period_t;
            rf_leg_trajectory[3].evaluate(rf_leg_joint, dt_);
        }else if( dt > 2 * period_t && dt <= 3 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 2 * period_t;
            rf_leg_trajectory[4].evaluate(rf_leg_joint, dt_);
        }else if(dt > 3 * period_t  && dt <= 4 * period_t)
        {
            dt_ = dt - (2 * period_t + adjust_t);
            rf_leg_trajectory[5].evaluate(rf_leg_joint, dt_);
        }else if(dt > 4 * period_t && dt <=5 * period_t)
        {
            dt_ = dt - 4 * period_t;
            rf_leg_trajectory[6].evaluate(rf_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        //        std::cout << "time is " << dt;
        dt = dt + delta_t;
        rf_leg_pose.getPosition().x() = rf_leg_joint.x();
        rf_leg_pose.getPosition().y() = rf_leg_joint.y();
        rf_leg_pose.getPosition().z() = rf_leg_joint.z();

        QK.InverseKinematicsSolve(rf_leg_pose.getPosition(),LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");
        rf_joint_positions.push_back(joints);
        rf_foot_position.push_back(rf_leg_joint);
    }

    std::cout << "finish the RF_leg planning" << std::endl;
}
void generate_lh_motion_1(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    /*******planning the LH leg *******/

    double lh_x_start = -height_x;
    double lh_y_start = height_y;
    double lh_z_start = height_z;
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_total = 4;

    times_of_lh.resize(step_total);
    lh_leg_position.resize(step_total);
    lh_leg_trajectory.resize(step_total);

    unsigned long j;

    //step 0: COM to back;
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(lh_x_start + Step_dis / 2 - forward_d, lh_y_start, lh_z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(lh_x_start + Step_dis / 2 + forward_d, lh_y_start, lh_z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 1: LH does not move (rf moves);
    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(lh_x_start + Step_dis / 2 + forward_d, lh_y_start, lh_z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(lh_x_start + Step_dis / 2 + forward_d, lh_y_start, lh_z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: lh does not move (body moves forward)
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(lh_x_start + Step_dis / 2 + forward_d, lh_y_start, lh_z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(lh_x_start + Step_dis / 2 - forward_d, lh_y_start, lh_z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3-1: lh moves.
    //get the initial joint angle;
    Pose lh_leg_pose_initial;
    valuetype lh_leg_joint_initial;
    JointPositionsLimb joints_initial;
    lh_leg_pose_initial.getPosition().x() = lh_x_start + Step_dis / 2 - forward_d;
    lh_leg_pose_initial.getPosition().y() = lh_y_start;
    lh_leg_pose_initial.getPosition().z() = lh_z_start;
    QK.InverseKinematicsSolve(lh_leg_pose_initial.getPosition(),LimbEnum::LH_LEG, joints_initial, joints_initial, "OUT_LEFT");

    Pose lh_leg_pose_final;
    valuetype lh_leg_joint_final;
    JointPositionsLimb joints_final;
    lh_leg_pose_final.getPosition().x() = lh_x_start - Step_dis / 2 - forward_d;
    lh_leg_pose_final.getPosition().y() = lh_y_start;
    lh_leg_pose_final.getPosition().z() = lh_z_start;

    QK.InverseKinematicsSolve(lh_leg_pose_final.getPosition(), LimbEnum::LH_LEG, joints_final, joints_final, "IN_LEFT");

    std::vector<valuetype> joint_interpolation;
    std::vector<Time> joint_time_inter;
    curves::PolynomialSplineQuinticVector3Curve joint_trajectory;

    valuetype joint_middle;
    joint_middle.x() = joints_initial.x();
    joint_middle.y() = joints_initial.y();
    joint_middle.z() = joints_initial.z();

    joint_time_inter.push_back(0.0);
    joint_interpolation.push_back(joint_middle);

    joints_initial << 0, 0 ,(joints_final.z() + 2 * M_PI + joints_initial.z()) / 2;
    joint_middle.x() = joints_initial.x();
    joint_middle.y() = joints_initial.y();
    joint_middle.z() = joints_initial.z();

    joint_time_inter.push_back(period_t / 2.0);
    joint_interpolation.push_back(joint_middle);

    joints_initial << joints_final.x(), joints_final.y(), joints_final.z() + 2 * M_PI;
    joint_middle.x() = joints_initial.x();
    joint_middle.y() = joints_initial.y();
    joint_middle.z() = joints_initial.z();

    joint_time_inter.push_back(period_t);
    joint_interpolation.push_back(joint_middle);

    joint_trajectory.fitCurve(joint_time_inter, joint_interpolation);

    //step 4: lh does not move(rh moves)
    j = 3;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(lh_x_start - Step_dis / 2 - forward_d, lh_y_start, lh_z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(lh_x_start - Step_dis / 2 - forward_d, lh_y_start, lh_z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 5 * period_t)
    {
        if(dt <= period_t)//step 0
        {
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt);
        }else if( dt > adjust_t && dt <= 2 * period_t) //step 1
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if(dt > 2 * period_t && dt <= 3 * period_t)//step 2
        {
            dt_ = dt - 2 * period_t;
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        }else if(dt > 3 * period_t && dt <= 4 * period_t) //step
        {
            if (dt == 4 * period_t)
            {
                std::cout << " leihou" <<std::endl;
            }
        }else if( dt > 4 * period_t && dt <= 5 * period_t) //step 4
        {
            dt_ = dt - 4 * period_t;
            lh_leg_trajectory[3].evaluate(lh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }

        lh_leg_pose.getPosition().x() = lh_leg_joint.x();
        lh_leg_pose.getPosition().y() = lh_leg_joint.y();
        lh_leg_pose.getPosition().z() = lh_leg_joint.z();

        if(dt <= 3 * period_t)
        {
            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        }else if (dt > 3 * period_t && dt <= 4 * period_t) {
            dt_ = dt - 3 * period_t;
            joint_trajectory.evaluate(lh_leg_joint, dt_);
            joints.x() = lh_leg_joint.x();
            joints.y() = lh_leg_joint.y();
            joints.z() = lh_leg_joint.z();
            QK.FowardKinematicsSolve(joints, LimbEnum::LH_LEG, lh_leg_pose);

            lh_leg_joint.x() = lh_leg_pose.getPosition().x();
            lh_leg_joint.y() = lh_leg_pose.getPosition().y();
            lh_leg_joint.z() = lh_leg_pose.getPosition().z();
        }else if(dt > 4 * period_t && dt <= 5 * period_t)
        {
            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(), LimbEnum::LH_LEG, joints, joints, "IN_LEFT");
            joints.z() = joints.z() + 2 * M_PI;
        }
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(lh_leg_joint);
        dt = dt + delta_t;
    }

    std::cout << "finish the LH_leg planning" << std::endl;
}


int main(int argc, char **argv)
{

    //step 1, LF->0.13, RF->0.13, LH->0, RH->0
    double height_1 = 0.10;//up stairs maximum height
    double height_2 = 0.05;//stairs height;
    double height_3 = 0.10;//plain walk height;

    double height_x = 0.382653;
    double height_y = 0.305;
    double height_z = -0.48;

    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;



    std::cout << "let us start" << std::endl;
    std::vector<JointPositionsLimb> lf_joint_positions, rh_joint_positions, rf_joint_positions, lh_joint_positions;

    generate_lf_motion(height_1, height_2, height_x, height_y, height_z, lf_joint_positions, lf_foot_height);
    generate_rf_motion_1(height_1, height_2, height_x, height_y, height_z, rf_joint_positions, rf_foot_height);
    generate_rh_motion_1(height_3, -height_2, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    generate_lh_motion_1(height_3, -height_2, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);

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
    std::cout << "lf_joint_position 4 size is " << lf_joint_positions.size() << std::endl;
    std::cout << "rf_joint_position 4 size is " << rf_joint_positions.size() << std::endl;
    std::cout << "rh_joint_position 4 size is " << rh_joint_positions.size() << std::endl;
    std::cout << "lh_joint_position 4 size is " << lh_joint_positions.size() << std::endl;

//    SaveAsFile_no_footheight("second_go_upstairs.txt",joint_positions_total);
    SaveAsFile("walk_state_to_upstairs.txt",joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);

}
