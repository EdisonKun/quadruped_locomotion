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
double period_t = 4;//period 6
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


unsigned long j;//curves number;
void lf_leg_moves()
{

}
void generate_lf_motion(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;
    /****planning lf leg******/
    //step 1: RF leg up and RH leg down, LF and LH keep the prepare angle unchanged;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;
    lf_leg_trajectory.resize(9);
    times_of_lf.resize(9);
    lf_leg_position.resize(9);

    //COM to the back;
    times_of_lf[0].push_back(0.0);
    lf_leg_position[0].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lf[0].push_back(adjust_t);
    lf_leg_position[0].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    lf_leg_trajectory[0].fitCurve(times_of_lf[0], lf_leg_position[0]);//1.2s

      //step 2:LF leg moves, RH moves with -0.35, the others unchanged;
    times_of_lf[1].push_back(0.0);
    lf_leg_position[1].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lf[1].push_back(period_t * 4 / 16);
    lf_leg_position[1].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + height_1));

    lf_leg_trajectory[1].fitCurve(times_of_lf[1], lf_leg_position[1]);//0.95s

         //step 2-1
    times_of_lf[2].push_back(0.0);
    lf_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_lf[2].push_back(period_t * 8 / 16);
    lf_leg_position[2].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    lf_leg_trajectory[2].fitCurve(times_of_lf[2], lf_leg_position[2]);//1.9s

         //step 2-2

    times_of_lf[3].push_back(0.0);
    lf_leg_position[3].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_lf[3].push_back(period_t * 4 / 16);
    lf_leg_position[3].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[3].fitCurve(times_of_lf[3], lf_leg_position[3]);//5s

    //step 3: LF leg does not move;

    times_of_lf[4].push_back(0.0);
    lf_leg_position[4].push_back(valuetype(x_start + Step_dis / 2 + forward_d , y_start, z_start + height_2));

    times_of_lf[4].push_back(adjust_t);
    lf_leg_position[4].push_back(valuetype(x_start + Step_dis / 2 + forward_d , y_start, z_start + height_2));

    lf_leg_trajectory[4].fitCurve(times_of_lf[4], lf_leg_position[4]);

    //step 4: COM to the front;
    times_of_lf[5].push_back(period_t * 0);
    lf_leg_position[5].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_lf[5].push_back(period_t * 1);
    lf_leg_position[5].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[5].fitCurve(times_of_lf[5], lf_leg_position[5]);

    //step 5: LF does not move;
    times_of_lf[6].push_back(0.0);
    lf_leg_position[6].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_lf[6].push_back(adjust_t);
    lf_leg_position[6].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[6].fitCurve(times_of_lf[6], lf_leg_position[6]);

    //step 6: LF does not move;
    times_of_lf[7].push_back(period_t * 0);
    lf_leg_position[7].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start  + height_2));

    times_of_lf[7].push_back(period_t * 1);
    lf_leg_position[7].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start  + height_2));

    lf_leg_trajectory[7].fitCurve(times_of_lf[7], lf_leg_position[7]);

    //step 7: all leg moves;
    j = 8;
    times_of_lf[j].push_back(period_t * 0);
    lf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start  + height_2));

    times_of_lf[j].push_back(period_t * 1);
    lf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start  + height_2));

    lf_leg_trajectory[j].fitCurve(times_of_lf[j], lf_leg_position[j]);

    JointPositionsLimb joints;
    valuetype lf_leg_valuetype;
    Pose lf_leg_pose;

    dt = 0;
    dt_ = 0;
    /***** evaluate the leg position ******/

    while(dt <= 7 * period_t)
    {
        if(dt <= adjust_t)//0-adjust_t
        {
            lf_leg_trajectory[0].evaluate(lf_leg_valuetype, dt);
        }else if( dt > adjust_t && dt <= (4.0 / 16 * period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - adjust_t;
            lf_leg_trajectory[1].evaluate(lf_leg_valuetype, dt_);
        }else if( dt > (4.0 / 16 * period_t + adjust_t) && dt <= (12.0 / 16 * period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (4.0 / 16 * period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(lf_leg_valuetype, dt_);
        }else if( dt > (12.0 / 16 * period_t + adjust_t) && dt <= (period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (12.0 / 16 * period_t + adjust_t);
            lf_leg_trajectory[3].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
            dt_ = dt - period_t - adjust_t;
            lf_leg_trajectory[4].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//adjust_t + period_t * 2~ 2 * period_t + 2 * adjust_t
        {
            dt_ = dt - period_t - 2 * adjust_t;
            lf_leg_trajectory[5].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - period_t * 2 - 2 * adjust_t;
            lf_leg_trajectory[6].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - 2 * period_t - 3 * adjust_t;
            lf_leg_trajectory[7].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(4 * period_t + 3 * adjust_t))
        {
            dt_ = dt - 3 * period_t - 3 * adjust_t;
            lf_leg_trajectory[8].evaluate(lf_leg_valuetype, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
//        std::cout << "dt is " << dt << " ";
//        std::cout << "time is " << dt << std::endl;
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
    double x_start = -height_x;
    double y_start = -height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;
    times_of_rh.resize(9);
    rh_leg_position.resize(9);
    rh_leg_trajectory.resize(9);
    //step 1: COM BACK;
    times_of_rh[0].push_back(0);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rh[0].push_back(adjust_t);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    rh_leg_trajectory[0].fitCurve(times_of_rh[0], rh_leg_position[0]);

    //step 2: LF moves;
    times_of_rh[1].push_back(0);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rh[1].push_back(period_t);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    rh_leg_trajectory[1].fitCurve(times_of_rh[1], rh_leg_position[1]);//5s

    //step 3: rf moves;
    times_of_rh[2].push_back(0);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rh[2].push_back(adjust_t);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    rh_leg_trajectory[2].fitCurve(times_of_rh[2], rh_leg_position[2]);//6.2s

    //step 4: COM forward;
    times_of_rh[3].push_back(0);
    rh_leg_position[3].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rh[3].push_back(adjust_t);
    rh_leg_position[3].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    rh_leg_trajectory[3].fitCurve(times_of_rh[3], rh_leg_position[3]);//6.2s

    //step 5: LH moves;
    times_of_rh[4].push_back(0);
    rh_leg_position[4].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rh[4].push_back(adjust_t);
    rh_leg_position[4].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    rh_leg_trajectory[4].fitCurve(times_of_rh[4], rh_leg_position[4]);//6.2s-----


    //step 6: rh moves;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 6;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 8.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//1.9s

    j = 7;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s

    j = 8;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s


    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 7 * period_t)
    {
//        std::cout <<setprecision(2) << fixed << "time is " << dt;
        if(dt <= adjust_t)//1
        {
//            std::cout <<"step 1" << std::endl;
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t))//2
        {
//            std::cout <<"step 2" << std::endl;
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }
        else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//3
        {
//            std::cout <<"step 3" << std::endl;
            dt_ = dt - period_t - adjust_t;
            rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }
        else if( dt >(period_t + 2 * adjust_t) && dt <= (2 * period_t + 2 * adjust_t)) //4
        {
//            std::cout <<"step 4";
            dt_ = dt - (period_t + 2 * adjust_t);
            rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
//            std::cout << rh_leg_joint.x() << std::endl;
        }
        else if( dt >(2 * period_t + 2 * adjust_t) && dt <= (2 * period_t  + 3 * adjust_t)) //5
        {
//                        std::cout <<"step 5";
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(rh_leg_joint, dt_);
        }

        else if( dt > (2 * period_t + 3 * adjust_t) && dt <= (2.25 * period_t + 3 * adjust_t)) //6-1
        {
//            std::cout <<"step 6";
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[5].evaluate(rh_leg_joint, dt_);
//            std::cout << rh_leg_joint.x() << std::endl;
        }
        else if( dt > (2.25 * period_t + 3 * adjust_t) && dt <= (2.75 * period_t + 3 * adjust_t)) //6-2
        {
//                        std::cout <<"step 7";
            dt_ = dt - (2.25 * period_t + 3 * adjust_t);
            rh_leg_trajectory[6].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }
        else if( dt > (2.75 * period_t + 3 * adjust_t) && dt <= (3 * period_t + 3 * adjust_t)) //6-3
        {
//                        std::cout <<"step 8";
            dt_ = dt - (2.75 * period_t + 3 * adjust_t);
            rh_leg_trajectory[7].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }else if( dt > (3 * period_t + 3 * adjust_t) && dt <= (4 * period_t + 3 * adjust_t)) //6-3
        {
//                        std::cout <<"step 9";
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[8].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        rh_leg_pose.getPosition().x() = rh_leg_joint.x();
        rh_leg_pose.getPosition().y() = rh_leg_joint.y();
        rh_leg_pose.getPosition().z() = rh_leg_joint.z();

        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(),LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(rh_leg_joint);
    }
    std::cout << "finish the RH_leg planning" << std::endl;
}
void generate_rf_motion_1(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
//    std::cout << "rf_joint_position size is " <<rf_joint_positions.size() << std::endl;
    double x_start = height_x;
    double y_start = -height_y;
    double z_start = height_z;
    /*******planning the RF leg *******/

    std::vector<std::vector<Time>> times_of_rf;
    times_of_rf.resize(9);
    std::vector<std::vector<valuetype>>rf_leg_position;
    rf_leg_position.resize(9);
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;
    rf_leg_trajectory.resize(9);

    //step 1: COM moves to back
    j = 0;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: lf moves;
    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: RF moves swing;
    j = 2;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t * 4 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);


    //step 3-1
    j = 3;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start + height_1));

    times_of_rf[j].push_back(period_t * 8 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//

    //step 3-2

    j = 4;
    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_rf[j].push_back(period_t * 4 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//12s

    //step 4: COM moves forward;
    j = 5;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: LH moves;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 6: RH moves;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: RH moves;
    j = 8;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    /***** evaluate the leg position ******/
    Pose rf_leg_pose;
    valuetype rf_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 7 * period_t )
    {
        if(dt <= adjust_t)//0-adjust_t
        {
//            std::cout << "step 1" << std::endl;
            rf_leg_trajectory[0].evaluate(rf_leg_joint, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
//            std::cout << "step 2" << std::endl;
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(rf_leg_joint, dt_);
        }else if( dt > (period_t + adjust_t) && dt <= (1.25* period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout << "step 6" << std::endl;
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(rf_leg_joint, dt_);
        }else if( dt > (1.25* period_t + adjust_t) && dt <= (1.75 * period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout << "step 7" << std::endl;
            dt_ = dt - (1.25* period_t + adjust_t);
            rf_leg_trajectory[3].evaluate(rf_leg_joint, dt_);
        }else if( dt > (1.75 * period_t + adjust_t) && dt <= (2 * period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout << "step 8" << std::endl;
            dt_ = dt - (1.75 * period_t + adjust_t);
            rf_leg_trajectory[4].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
//            std::cout << "step 9" << std::endl;
            dt_ = dt - (2 * period_t + adjust_t);
            rf_leg_trajectory[5].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(3 * period_t + 2 * adjust_t))
        {
//            std::cout << "step 10" << std::endl;
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[6].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 2 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            //            std::cout << "step 10" << std::endl;
            dt_ = dt - (3 * period_t + 2 * adjust_t);
            rf_leg_trajectory[7].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(4 * period_t + 3 * adjust_t))
        {
            //            std::cout << "step 10" << std::endl;
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[8].evaluate(rf_leg_joint, dt_);
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

    double x_start = -height_x;
    double y_start = height_y;
    double z_start = height_z;
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    times_of_lh.resize(7);
    lh_leg_position.resize(7);
    lh_leg_trajectory.resize(7);

    //step 1: COM to back;
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start - down_height));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: lf moves;
    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: rf moves;
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: COM to forward.
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: lh moves;
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);    

    //step 6: rh moves
    j = 5;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    //step 7: rh moves
    j = 6;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 7 * period_t)
    {
        if(dt <= adjust_t)//step 1
        {
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt);
        }else if( dt > adjust_t && dt <= (period_t + adjust_t)) //step 2
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//step 3
        {
            dt_ = dt - period_t - adjust_t;
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//step 4
        {
            dt_ = dt - period_t - 2 * adjust_t;
            lh_leg_trajectory[3].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2 * period_t + 2 * adjust_t) && dt <= (2 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lh_leg_trajectory[4].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2 * period_t + 3 * adjust_t) && dt <= (3 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lh_leg_trajectory[5].evaluate(lh_leg_joint, dt_);
        }else if( dt > (3 * period_t + 3 * adjust_t) && dt <= (4 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lh_leg_trajectory[6].evaluate(lh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        lh_leg_pose.getPosition().x() = lh_leg_joint.x();
        lh_leg_pose.getPosition().y() = lh_leg_joint.y();
        lh_leg_pose.getPosition().z() = lh_leg_joint.z();

        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(lh_leg_joint);
    }

    std::cout << "finish the LH_leg planning" << std::endl;
}

void generate_rh_motion_2(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    /*******planning the RH leg *******/
    double x_start = -height_x;
    double y_start = -height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;
    times_of_rh.resize(9);
    rh_leg_position.resize(9);
    rh_leg_trajectory.resize(9);
    //step 1: COM BACK;
    times_of_rh[0].push_back(0);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[0].push_back(adjust_t);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rh_leg_trajectory[0].fitCurve(times_of_rh[0], rh_leg_position[0]);

    //step 2: LF moves;
    times_of_rh[1].push_back(0);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rh[1].push_back(period_t);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rh_leg_trajectory[1].fitCurve(times_of_rh[1], rh_leg_position[1]);//5s

    //step 3: rf moves;
    times_of_rh[2].push_back(0);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rh[2].push_back(adjust_t);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rh_leg_trajectory[2].fitCurve(times_of_rh[2], rh_leg_position[2]);//6.2s

    //step 4: COM forward;
    times_of_rh[3].push_back(0);
    rh_leg_position[3].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rh[3].push_back(adjust_t);
    rh_leg_position[3].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[3].fitCurve(times_of_rh[3], rh_leg_position[3]);//6.2s

    //step 5: LH moves;
    times_of_rh[4].push_back(0);
    rh_leg_position[4].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[4].push_back(adjust_t);
    rh_leg_position[4].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[4].fitCurve(times_of_rh[4], rh_leg_position[4]);//6.2s-----


    //step 6: rh moves;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 6;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 8.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//1.9s

    j = 7;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s

    j = 8;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s


    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 7 * period_t)
    {
        //        std::cout <<setprecision(2) << fixed << "time is " << dt;
        if(dt <= adjust_t)//1
        {
            //            std::cout <<"step 1" << std::endl;
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t))//2
        {
            //            std::cout <<"step 2" << std::endl;
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }
        else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//3
        {
            //            std::cout <<"step 3" << std::endl;
            dt_ = dt - period_t - adjust_t;
            rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }
        else if( dt >(period_t + 2 * adjust_t) && dt <= (2 * period_t + 2 * adjust_t)) //4
        {
            //            std::cout <<"step 4";
            dt_ = dt - (period_t + 2 * adjust_t);
            rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }
        else if( dt >(2 * period_t + 2 * adjust_t) && dt <= (2 * period_t  + 3 * adjust_t)) //5
        {
            //                        std::cout <<"step 5";
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(rh_leg_joint, dt_);
        }

        else if( dt > (2 * period_t + 3 * adjust_t) && dt <= (2.25 * period_t + 3 * adjust_t)) //6-1
        {
            //            std::cout <<"step 6";
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rh_leg_trajectory[5].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }
        else if( dt > (2.25 * period_t + 3 * adjust_t) && dt <= (2.75 * period_t + 3 * adjust_t)) //6-2
        {
            //                        std::cout <<"step 7";
            dt_ = dt - (2.25 * period_t + 3 * adjust_t);
            rh_leg_trajectory[6].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }
        else if( dt > (2.75 * period_t + 3 * adjust_t) && dt <= (3 * period_t + 3 * adjust_t)) //6-3
        {
            //                        std::cout <<"step 8";
            dt_ = dt - (2.75 * period_t + 3 * adjust_t);
            rh_leg_trajectory[7].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }else if( dt > (3 * period_t + 3 * adjust_t) && dt <= (4 * period_t + 3 * adjust_t)) //6-3
        {
            //                        std::cout <<"step 9";
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rh_leg_trajectory[8].evaluate(rh_leg_joint, dt_);
            //            std::cout << rh_leg_joint.x() << std::endl;
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        rh_leg_pose.getPosition().x() = rh_leg_joint.x();
        rh_leg_pose.getPosition().y() = rh_leg_joint.y();
        rh_leg_pose.getPosition().z() = rh_leg_joint.z();

        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(),LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(rh_leg_joint);
    }
    std::cout << "finish the RH_leg planning" << std::endl;
}
void generate_rf_motion_2(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    //    std::cout << "rf_joint_position size is " <<rf_joint_positions.size() << std::endl;
    double x_start = height_x;
    double y_start = -height_y;
    double z_start = height_z;
    /*******planning the RF leg *******/

    std::vector<std::vector<Time>> times_of_rf;
    times_of_rf.resize(9);
    std::vector<std::vector<valuetype>>rf_leg_position;
    rf_leg_position.resize(9);
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;
    rf_leg_trajectory.resize(9);

    //step 1: COM moves to back
    j = 0;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: lf moves;
    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: RF moves swing;
    j = 2;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t * 4 / 16);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);


    //step 3-1
    j = 3;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_rf[j].push_back(period_t * 8 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//

    //step 3-2

    j = 4;
    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_rf[j].push_back(period_t * 4 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//12s

    //step 4: COM moves forward;
    j = 5;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: LH moves;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 6: RH moves;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 7: RH moves;
    j = 8;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    /***** evaluate the leg position ******/
    Pose rf_leg_pose;
    valuetype rf_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 7 * period_t )
    {
        if(dt <= adjust_t)//0-adjust_t
        {
            //            std::cout << "step 1" << std::endl;
            rf_leg_trajectory[0].evaluate(rf_leg_joint, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
            //            std::cout << "step 2" << std::endl;
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(rf_leg_joint, dt_);
        }else if( dt > (period_t + adjust_t) && dt <= (1.25* period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            //            std::cout << "step 6" << std::endl;
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(rf_leg_joint, dt_);
        }else if( dt > (1.25* period_t + adjust_t) && dt <= (1.75 * period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            //            std::cout << "step 7" << std::endl;
            dt_ = dt - (1.25* period_t + adjust_t);
            rf_leg_trajectory[3].evaluate(rf_leg_joint, dt_);
        }else if( dt > (1.75 * period_t + adjust_t) && dt <= (2 * period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            //            std::cout << "step 8" << std::endl;
            dt_ = dt - (1.75 * period_t + adjust_t);
            rf_leg_trajectory[4].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            //            std::cout << "step 9" << std::endl;
            dt_ = dt - (2 * period_t + adjust_t);
            rf_leg_trajectory[5].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(3 * period_t + 2 * adjust_t))
        {
            //            std::cout << "step 10" << std::endl;
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[6].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 2 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            //            std::cout << "step 10" << std::endl;
            dt_ = dt - (3 * period_t + 2 * adjust_t);
            rf_leg_trajectory[7].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(4 * period_t + 3 * adjust_t))
        {
            //            std::cout << "step 10" << std::endl;
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[8].evaluate(rf_leg_joint, dt_);
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
void generate_lh_motion_2(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    /*******planning the LH leg *******/

    double x_start = -height_x;
    double y_start = height_y;
    double z_start = height_z;
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    times_of_lh.resize(9);
    lh_leg_position.resize(9);
    lh_leg_trajectory.resize(9);

    //step 1: COM to back;
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start - down_height));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: lf moves;
    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: rf moves;
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: COM to forward.
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: lh moves;
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.5 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: rh moves
    j = 7;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    //step 7: all leg moves
    j = 8;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 7 * period_t)
    {
        if(dt <= adjust_t)//step 1
        {
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt);
        }else if( dt > adjust_t && dt <= (period_t + adjust_t)) //step 2
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//step 3
        {
            dt_ = dt - period_t - adjust_t;
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//step 4
        {
            dt_ = dt - period_t - 2 * adjust_t;
            lh_leg_trajectory[3].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2 * period_t + 2 * adjust_t) && dt <= (2.25 * period_t + 2 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lh_leg_trajectory[4].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2.25 * period_t + 2 * adjust_t) && dt <= (2.75 * period_t + 2 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2.25 * period_t + 2 * adjust_t);
            lh_leg_trajectory[5].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2.75 * period_t + 2 * adjust_t) && dt <= (3 * period_t + 2 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2.75 * period_t + 2 * adjust_t);
            lh_leg_trajectory[6].evaluate(lh_leg_joint, dt_);
        } else if( dt > (2 * period_t + 3 * adjust_t) && dt <= (3 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lh_leg_trajectory[7].evaluate(lh_leg_joint, dt_);
        }else if( dt > (3 * period_t + 3 * adjust_t) && dt <= (4 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lh_leg_trajectory[8].evaluate(lh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        lh_leg_pose.getPosition().x() = lh_leg_joint.x();
        lh_leg_pose.getPosition().y() = lh_leg_joint.y();
        lh_leg_pose.getPosition().z() = lh_leg_joint.z();

        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(lh_leg_joint);
    }

    std::cout << "finish the LH_leg planning" << std::endl;
}

// the back legs are high than the front legs
void generate_rf_motion_3(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
//    std::cout << "rf_joint_position size is " <<rf_joint_positions.size() << std::endl;
    double x_start = height_x;
    double y_start = -height_y;
    double z_start = height_z;
    /*******planning the RF leg *******/

    std::vector<std::vector<Time>> times_of_rf;
    times_of_rf.resize(1);
    std::vector<std::vector<valuetype>>rf_leg_position;
    rf_leg_position.resize(1);
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;
    rf_leg_trajectory.resize(1);

    //step 1: COM moves to back
    j = 0;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(3 * period_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);


    /***** evaluate the leg position ******/
    Pose rf_leg_pose;
    valuetype rf_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 3 * period_t )
    {
        if(dt <= 3 * adjust_t)//0-adjust_t
        {
            rf_leg_trajectory[0].evaluate(rf_leg_joint, dt);
        }else{
            std::cout << "nothing" << std::endl;
        }
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
void generate_lf_motion_3(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;
    /****planning lf leg******/
    //step 1: RF leg up and RH leg down, LF and LH keep the prepare angle unchanged;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;
    lf_leg_trajectory.resize(1);
    times_of_lf.resize(1);
    lf_leg_position.resize(1);

    //COM to the back;
    times_of_lf[0].push_back(0.0);
    lf_leg_position[0].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_lf[0].push_back(3 * adjust_t);
    lf_leg_position[0].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[0].fitCurve(times_of_lf[0], lf_leg_position[0]);//1.2s

    JointPositionsLimb joints;
    valuetype lf_leg_valuetype;
    Pose lf_leg_pose;

    dt = 0;
    dt_ = 0;
    /***** evaluate the leg position ******/

    while(dt <= 3 * period_t)
    {
        if(dt <= 3 * adjust_t)//0-adjust_t
        {
            lf_leg_trajectory[0].evaluate(lf_leg_valuetype, dt);
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

void generate_lh_motion_3(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    /*******planning the LH leg *******/

    double x_start = -height_x;
    double y_start = height_y;
    double z_start = height_z;
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    times_of_lh.resize(5);
    lh_leg_position.resize(5);
    lh_leg_trajectory.resize(5);


    //step 5: lh moves;
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.5 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: rh moves
    j = 3;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    //step 7: all leg moves
    j = 4;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 3 * period_t)
    {
        if(dt <= (0.25 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt;
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt_);
        }else if( dt > (0.25 * period_t) && dt <= (0.75 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (0.25 * period_t);
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if( dt > (0.75 * period_t) && dt <= (period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (0.75 * period_t);
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        } else if( dt > (period_t) && dt <= (2 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (period_t);
            lh_leg_trajectory[3].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2 * period_t) && dt <= (3 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2 * period_t);
            lh_leg_trajectory[4].evaluate(lh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        lh_leg_pose.getPosition().x() = lh_leg_joint.x();
        lh_leg_pose.getPosition().y() = lh_leg_joint.y();
        lh_leg_pose.getPosition().z() = lh_leg_joint.z();

        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(lh_leg_joint);
    }

    std::cout << "finish the LH_leg planning" << std::endl;
}
void generate_rh_motion_3(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    /*******planning the RH leg *******/
    double x_start = -height_x;
    double y_start = -height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;
    times_of_rh.resize(5);
    rh_leg_position.resize(5);
    rh_leg_trajectory.resize(5);

    //step 5: LH moves;
    j = 0;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);


    //step 6: rh moves;
    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 2;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 8.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//1.9s

    j = 3;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s

    j = 4;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s


    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 3 * period_t)
    {
        if(dt <= (period_t)) //5
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt_);
        }else if( dt > (period_t) && dt <= (1.25 * period_t)) //6-1
        {
            dt_ = dt - (period_t);
            rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }else if( dt > (1.25 * period_t) && dt <= (1.75 * period_t)) //6-2
        {
            dt_ = dt - (1.25 * period_t);
            rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }else if( dt > (1.75 * period_t) && dt <= (2 * period_t)) //6-3
        {
            dt_ = dt - (1.75 * period_t);
            rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
        }else if( dt > (2 * period_t) && dt <= (3 * period_t)) //6-3
        {
            dt_ = dt - (2 * period_t);
            rh_leg_trajectory[4].evaluate(rh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        rh_leg_pose.getPosition().x() = rh_leg_joint.x();
        rh_leg_pose.getPosition().y() = rh_leg_joint.y();
        rh_leg_pose.getPosition().z() = rh_leg_joint.z();

        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(),LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(rh_leg_joint);
    }
    std::cout << "finish the RH_leg planning" << std::endl;
}

void generate_lh_motion_4(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    /*******planning the LH leg *******/

    double x_start = -height_x;
    double y_start = height_y;
    double z_start = height_z;
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    times_of_lh.resize(5);
    lh_leg_position.resize(5);
    lh_leg_trajectory.resize(5);


    //step 5: lh moves;
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.5 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(0.25 * period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: rh moves
    j = 3;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    //step 7: all leg moves
    j = 4;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 3 * period_t)
    {
        if(dt <= (0.25 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt;
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt_);
        }else if( dt > (0.25 * period_t) && dt <= (0.75 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (0.25 * period_t);
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if( dt > (0.75 * period_t) && dt <= (period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (0.75 * period_t);
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        } else if( dt > (period_t) && dt <= (2 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (period_t);
            lh_leg_trajectory[3].evaluate(lh_leg_joint, dt_);
        }else if( dt > (2 * period_t) && dt <= (3 * period_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (2 * period_t);
            lh_leg_trajectory[4].evaluate(lh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        lh_leg_pose.getPosition().x() = lh_leg_joint.x();
        lh_leg_pose.getPosition().y() = lh_leg_joint.y();
        lh_leg_pose.getPosition().z() = lh_leg_joint.z();

        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        lh_joint_positions.push_back(joints);
        lh_foot_position.push_back(lh_leg_joint);
    }

    std::cout << "finish the LH_leg planning" << std::endl;
}
void generate_rh_motion_4(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    /*******planning the RH leg *******/
    double x_start = -height_x;
    double y_start = -height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;
    times_of_rh.resize(5);
    rh_leg_position.resize(5);
    rh_leg_trajectory.resize(5);

    //step 5: LH moves;
    j = 0;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);


    //step 6: rh moves;
    j = 1;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    j = 2;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 8.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//1.9s

    j = 3;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s

    j = 4;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start ));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s


    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 3 * period_t)
    {
        if(dt <= (period_t)) //5
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt_);
        }else if( dt > (period_t) && dt <= (1.25 * period_t)) //6-1
        {
            dt_ = dt - (period_t);
            rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }else if( dt > (1.25 * period_t) && dt <= (1.75 * period_t)) //6-2
        {
            dt_ = dt - (1.25 * period_t);
            rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }else if( dt > (1.75 * period_t) && dt <= (2 * period_t)) //6-3
        {
            dt_ = dt - (1.75 * period_t);
            rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
        }else if( dt > (2 * period_t) && dt <= (3 * period_t)) //6-3
        {
            dt_ = dt - (2 * period_t);
            rh_leg_trajectory[4].evaluate(rh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;
        rh_leg_pose.getPosition().x() = rh_leg_joint.x();
        rh_leg_pose.getPosition().y() = rh_leg_joint.y();
        rh_leg_pose.getPosition().z() = rh_leg_joint.z();

        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(),LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        rh_joint_positions.push_back(joints);
        rh_foot_position.push_back(rh_leg_joint);
    }
    std::cout << "finish the RH_leg planning" << std::endl;
}


int main(int argc, char **argv)
{

    //step 1, LF->0.13, RF->0.13, LH->0, RH->0
    double height_1 = 0.18;//up stairs maximum height
    double height_2 = 0.13;//plain
    double height_3 = 0.10;

    double height_x = 0.382653;
    double height_y = 0.305;
    double height_z = -0.48;

    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;



    std::cout << "let us start" << std::endl;
    std::vector<JointPositionsLimb> lf_joint_positions, rh_joint_positions, rf_joint_positions, lh_joint_positions;

    generate_lf_motion(height_1, height_2, height_x, height_y, height_z, lf_joint_positions, lf_foot_height);
    generate_rf_motion_1(height_1, height_2, height_x, height_y, height_z, rf_joint_positions, rf_foot_height);
    generate_rh_motion_1(height_3, 0, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    generate_lh_motion_1(height_3, 0, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);

    //step 2
    double height_z_to_front = height_z + height_2;
    generate_lf_motion(height_1, height_2, height_x, height_y, height_z_to_front, lf_joint_positions, lf_foot_height);
    generate_rf_motion_2(height_1, height_2, height_x, height_y, height_z_to_front, rf_joint_positions, rf_foot_height);
    generate_rh_motion_2(height_3, 0, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    generate_lh_motion_2(height_3, 0, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);

    //step 3
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

    SaveAsFile_no_footheight("go_upstairs.txt",joint_positions_total);

    lf_joint_positions.clear();
    rf_joint_positions.clear();
    lh_joint_positions.clear();
    rh_joint_positions.clear();
    lf_foot_height.clear();
    rf_foot_height.clear();
    lh_foot_height.clear();
    rh_foot_height.clear();

    //step 3
    height_z_to_front = height_z + height_2;//height_2 = 0.13; height_1 = 0.18;
    generate_lf_motion_3(height_1, height_2, height_x, height_y, height_z_to_front, lf_joint_positions, lf_foot_height);
    generate_rf_motion_3(height_1, height_2, height_x, height_y, height_z_to_front, rf_joint_positions, rf_foot_height);
    generate_lh_motion_3(height_1, height_2, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);
    generate_rh_motion_3(height_1, height_2, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);

    //step 4
    double height_z_to_back = height_z + height_2;
    generate_lf_motion_3(height_1, height_2, height_x, height_y, height_z_to_front, lf_joint_positions, lf_foot_height);
    generate_rf_motion_3(height_1, height_2, height_x, height_y, height_z_to_front, rf_joint_positions, rf_foot_height);
    generate_lh_motion_4(height_1, height_2, height_x,height_y, height_z_to_back, lh_joint_positions, lh_foot_height);
    generate_rh_motion_4(height_1, height_2, height_x, height_y, height_z_to_back, rh_joint_positions, rh_foot_height);

    joint_positions_total.clear();
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

    SaveAsFile_no_footheight("second_go_upstairs.txt",joint_positions_total);

//    quadruped_model::JointPositionsLimb joint_position_temp;
//    Pose cartisian_pose;
//    joint_position_temp(0) = joint_positions(0);
//    joint_position_temp(1) = joint_positions(1);
//    joint_position_temp(2) = joint_positions(2);

//    QK.FowardKinematicsSolve(joint_position_temp, quadruped_model::LimbEnum::LF_LEG, cartisian_pose);
//    std::cout << cartisian_pose.getPosition() << std::endl;
//    std::cout << lf_foot_height.back() << std::endl;




    /******Now we store the joint angles to the txt file *******/
//    for(unsigned int i = 0; i < lf_joint_positions.size(); i++)
//    {
//        joint_positions(0) = lf_joint_positions[i](0);
//        joint_positions(1) = lf_joint_positions[i](1);
//        joint_positions(2) = lf_joint_positions[i](2);

//        joint_positions(6) = rh_joint_positions[i](0);
//        joint_positions(7) = rh_joint_positions[i](1);
//        joint_positions(8) = rh_joint_positions[i](2);

//        joint_positions(3) = rf_joint_positions[i](0);
//        joint_positions(4) = rf_joint_positions[i](1);
//        joint_positions(5) = rf_joint_positions[i](2);

//        joint_positions(9) = lh_joint_positions[i](0);
//        joint_positions(10) = lh_joint_positions[i](1);
//        joint_positions(11) = lh_joint_positions[i](2);

//        joint_positions_total.push_back(joint_positions);

//    }
//    std::cout << "size is " << joint_positions_total.size() << std::endl;

//    SaveAsFile("go_upstairs_2.txt", joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);
    /*****end store the joint file*****/


//    std::ifstream readfile;
//    readfile.open("calculated.txt");
//    quadruped_model::JointPositions joint_position_file;
//    std::vector<quadruped_model::JointPositions> joint_position_collection;
//    double time;
//    for(int i = 0; !readfile.eof(); i++)
//      {
//         readfile >> time >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
//                                                       >> joint_position_file(3)>> joint_position_file(4)
//                                                          >> joint_position_file(5)>> joint_position_file(6)
//                                                             >> joint_position_file(7)>> joint_position_file(8)
//                                                                >> joint_position_file(9)>> joint_position_file(10)
//                                                                   >> joint_position_file(11);
//         joint_position_collection.push_back(joint_position_file);
////         std::cout << time <<" " << joint_position_file << std::endl;
//      }
//    readfile.close();
//    joint_position_collection.pop_back();


}
