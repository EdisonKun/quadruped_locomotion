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

//read joint angle from the kneel down and move to the 0.2 0.45 0
void generate_lf_motion(quadruped_model::JointPositionsLimb& lf_start_position, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;

    unsigned long step_total = 4;
    lf_leg_trajectory.resize(step_total);
    times_of_lf.resize(step_total);
    lf_leg_position.resize(step_total);

    JointPositionsLimb lf_joint;
    Pose fd_lf_pose, fd_lh_pose;
    fd_lf_pose.getPosition() << 0.2, 0.305, 0.155;
    QK.InverseKinematicsSolve(fd_lf_pose.getPosition(), LimbEnum::LF_LEG, lf_joint, lf_joint, "IN_RIGHT");
    std::cout << "lf_joint is " << lf_joint << std::endl;

    // step 0;
    times_of_lf[0].push_back(0.0);
    lf_leg_position[0].push_back(valuetype(lf_start_position.x(), lf_start_position.y(), lf_start_position.z()));

    times_of_lf[0].push_back(2 * period_t);
    lf_leg_position[0].push_back(valuetype(lf_joint.x(), lf_joint.y(), lf_joint.z() - 2* M_PI));

    lf_leg_trajectory[0].fitCurve(times_of_lf[0], lf_leg_position[0]);

    //step 3:
    std::cout << "lf_joint is " << lf_joint << std::endl;
    lf_joint.z() = lf_joint.z() - 2 * M_PI;
//    lf_joint << 0, 1.1, 2.2;
    QK.FowardKinematicsSolve(lf_joint,LimbEnum::LF_LEG,fd_lf_pose);
    std::cout << fd_lf_pose.getPosition() << std::endl;

    times_of_lf[1].push_back(0.0);
    lf_leg_position[1].push_back(valuetype(fd_lf_pose.getPosition().x(), fd_lf_pose.getPosition().y(), fd_lf_pose.getPosition().z()));

    times_of_lf[1].push_back(period_t);
    lf_leg_position[1].push_back(valuetype(fd_lf_pose.getPosition().x() - 0.08, fd_lf_pose.getPosition().y(), fd_lf_pose.getPosition().z()));

    lf_leg_trajectory[1].fitCurve(times_of_lf[1], lf_leg_position[1]);

    times_of_lf[2].push_back(0.0);
    lf_leg_position[2].push_back(valuetype(fd_lf_pose.getPosition().x() - 0.08, fd_lf_pose.getPosition().y(), fd_lf_pose.getPosition().z()));

    times_of_lf[2].push_back(period_t);
    lf_leg_position[2].push_back(valuetype(fd_lf_pose.getPosition().x() - 0.08, fd_lf_pose.getPosition().y(), fd_lf_pose.getPosition().z() + 0.05));

    lf_leg_trajectory[2].fitCurve(times_of_lf[2], lf_leg_position[2]);

    JointPositionsLimb joints;
    valuetype lf_leg_valuetype;
    Pose lf_leg_pose;

    dt = 0;
    dt_ = 0;
    /***** evaluate the leg position ******/

    while(dt <= 4 * period_t)
    {
        if(dt <= 2 * adjust_t)
        {
            lf_leg_trajectory[0].evaluate(lf_leg_valuetype, dt);
        }else if( dt > 2 * period_t && dt <= 3 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 2 * period_t;
            lf_leg_trajectory[1].evaluate(lf_leg_valuetype, dt_);
        }else if( dt > 3 * period_t && dt <= 4 * period_t) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - 3 * period_t;
            lf_leg_trajectory[2].evaluate(lf_leg_valuetype, dt_);
        }else{
            std::cout << "nothing" << std::endl;        }

        if(dt <= 2 * period_t)
        {
            joints << lf_leg_valuetype;
            QK.FowardKinematicsSolve(joints, LimbEnum::LF_LEG, lf_leg_pose);
            lf_leg_valuetype << lf_leg_pose.getPosition().x(), lf_leg_pose.getPosition().y(), lf_leg_pose.getPosition().z();
        }else {
            lf_leg_pose.getPosition().x() = lf_leg_valuetype.x();
            lf_leg_pose.getPosition().y() = lf_leg_valuetype.y();
            lf_leg_pose.getPosition().z() = lf_leg_valuetype.z();

            QK.InverseKinematicsSolve(lf_leg_pose.getPosition(),LimbEnum::LF_LEG, joints, joints, "IN_RIGHT");
            joints.z() = joints.z() - 2 * M_PI;
        }

        lf_joint_positions.push_back(joints);
        lf_foot_position.push_back(lf_leg_valuetype);
        dt = dt + delta_t;
    }
    std::cout << "finish the lf_leg planning" << std::endl;
}
// read the joint angles of the kneel down;
void generate_rh_motion_1(quadruped_model::JointPositionsLimb& rh_start_position, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    /*******planning the RH leg *******/


    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

    unsigned long step_total = 1;
    times_of_rh.resize(step_total);
    rh_leg_position.resize(step_total);
    rh_leg_trajectory.resize(step_total);

    unsigned long j;
    //step 0: COM BACK;
    times_of_rh[0].push_back(0);
    rh_leg_position[0].push_back(valuetype(rh_start_position.x(), rh_start_position.y(), rh_start_position.z()));

    times_of_rh[0].push_back(4 * period_t);
    rh_leg_position[0].push_back(valuetype(rh_start_position.x(), rh_start_position.y(), rh_start_position.z()));

    rh_leg_trajectory[0].fitCurve(times_of_rh[0], rh_leg_position[0]);

    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 4 * period_t)
    {
        if(dt <= 4 * period_t)
        {
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt);
        }else{
            std::cout << "nothing" << std::endl;
        }

        joints << rh_leg_joint;

        QK.FowardKinematicsSolve(joints, LimbEnum::RH_LEG, rh_leg_pose);
        rh_joint_positions.push_back(joints);

        rh_leg_joint.x() = rh_leg_pose.getPosition().x();
        rh_leg_joint.y() = rh_leg_pose.getPosition().y();
        rh_leg_joint.z() = rh_leg_pose.getPosition().z();

        rh_foot_position.push_back(rh_leg_joint);
        dt = dt + delta_t;
    }
    std::cout << "finish the RH_leg planning" << std::endl;
}
// read the joint angles of the kneel down;
void generate_rf_motion_1(quadruped_model::JointPositionsLimb& rf_start_position, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    //    std::cout << "rf_joint_position size is " <<rf_joint_positions.size() << std::endl;

    /*******planning the RF leg *******/

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;

    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;

    unsigned long step_total = 1;
    rf_leg_trajectory.resize(step_total);
    rf_leg_position.resize(step_total);
    times_of_rf.resize(step_total);

    unsigned long j;

    //step 0: COM moves to back
    j = 0;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(rf_start_position.x(), rf_start_position.y(), rf_start_position.z()));

    times_of_rf[j].push_back(4 * period_t);
    rf_leg_position[j].push_back(valuetype(rf_start_position.x(), rf_start_position.y(), rf_start_position.z()));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    /***** evaluate the leg position ******/
    Pose rf_leg_pose;
    valuetype rf_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 4 * period_t )
    {
        if(dt <= 4 * period_t)
        {
            rf_leg_trajectory[0].evaluate(rf_leg_joint, dt);
        }else{
            std::cout << "nothing" << std::endl;
        }
        dt = dt + delta_t;

        joints << rf_leg_joint;

        QK.FowardKinematicsSolve(joints, LimbEnum::RF_LEG, rf_leg_pose);

        rf_joint_positions.push_back(joints);

        rf_leg_joint.x() = rf_leg_pose.getPosition().x();
        rf_leg_joint.y() = rf_leg_pose.getPosition().y();
        rf_leg_joint.z() = rf_leg_pose.getPosition().z();

        rf_foot_position.push_back(rf_leg_joint);
    }

    std::cout << "finish the RF_leg planning" << std::endl;
}
void generate_lh_motion_1(quadruped_model::JointPositionsLimb& lh_start_position, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    /*******planning the LH leg *******/

    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    unsigned long step_total = 4;

    times_of_lh.resize(step_total);
    lh_leg_position.resize(step_total);
    lh_leg_trajectory.resize(step_total);

    JointPositionsLimb lh_joint;
    Pose fd_lh_pose;
    fd_lh_pose.getPosition() << -0.2, 0.305, 0.155;
    QK.InverseKinematicsSolve(fd_lh_pose.getPosition(), LimbEnum::LH_LEG, lh_joint, lh_joint, "OUT_RIGHT");
    std::cout << "lh_joint is" << lh_joint << std::endl;

    unsigned long j;

    //step 0:
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(lh_start_position.x(), lh_start_position.y(), lh_start_position.z()));

    times_of_lh[j].push_back(2 * adjust_t);
    lh_leg_position[j].push_back(valuetype(lh_joint.x(), -lh_joint.y(), lh_joint.z() + 2 * M_PI));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(fd_lh_pose.getPosition().x(), fd_lh_pose.getPosition().y(), fd_lh_pose.getPosition().z()));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(fd_lh_pose.getPosition().x() + 0.08, fd_lh_pose.getPosition().y(), fd_lh_pose.getPosition().z()));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(fd_lh_pose.getPosition().x() + 0.08, fd_lh_pose.getPosition().y(), fd_lh_pose.getPosition().z()));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(fd_lh_pose.getPosition().x() + 0.08, fd_lh_pose.getPosition().y(), fd_lh_pose.getPosition().z() + 0.05));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;
    joints << 0, -1.11, -2.2;
    QK.FowardKinematicsSolve(joints, LimbEnum::LH_LEG, lh_leg_pose);
    std::cout << "lh_leg_position is " << lh_leg_pose.getPosition() << std::endl;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 4 * period_t)
    {
        if(dt <= 2 * period_t)//step 0
        {
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt);
        }else if(dt > 2 * period_t && dt <= 3 * period_t)//step 2
        {
            dt_ = dt - 2 * period_t;
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if(dt > 3 * period_t && dt <= 4 * period_t)//step 2
        {
            dt_ = dt - 3 * period_t;
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        }else{
            std::cout << "nothing" << std::endl;
        }

        if(dt <= 2 * period_t)
        {
            joints << lh_leg_joint;
            QK.FowardKinematicsSolve(joints, LimbEnum::LH_LEG, lh_leg_pose);
            lh_leg_joint << lh_leg_pose.getPosition().x(), lh_leg_pose.getPosition().y(), lh_leg_pose.getPosition().z();
        }else {
            lh_leg_pose.getPosition().x() = lh_leg_joint.x();
            lh_leg_pose.getPosition().y() = lh_leg_joint.y();
            lh_leg_pose.getPosition().z() = lh_leg_joint.z();

            QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_RIGHT");
            joints.y() = -joints.y();
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
    std::ifstream readfile;
    readfile.open("/home/kun/catkin_ws_dependency/15_walk_and_kneel_down.txt");
    assert(readfile.is_open());
    quadruped_model::JointPositions joint_position_file;
    std::vector<quadruped_model::JointPositions> joint_positions_read;
    double time;
    for(int i = 0; !readfile.eof(); i++)
    {
        readfile >> time >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
            >> joint_position_file(3)>> joint_position_file(4)
            >> joint_position_file(5)>> joint_position_file(6)
            >> joint_position_file(7)>> joint_position_file(8)
            >> joint_position_file(9)>> joint_position_file(10)
            >> joint_position_file(11);
        joint_positions_read.push_back(joint_position_file);
    }
    readfile.close();
    joint_positions_read.pop_back();


    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;
    std::vector<JointPositionsLimb> lf_joint_positions, rh_joint_positions, rf_joint_positions, lh_joint_positions;

    quadruped_model::JointPositionsLimb lf_joint_start, rf_joint_start, rh_joint_start, lh_joint_start;

    lf_joint_start << joint_positions_read.back()(0), joint_positions_read.back()(1), joint_positions_read.back()(2);
    rf_joint_start << joint_positions_read.back()(3), joint_positions_read.back()(4), joint_positions_read.back()(5);
    rh_joint_start << joint_positions_read.back()(6), joint_positions_read.back()(7), joint_positions_read.back()(8);
    lh_joint_start << joint_positions_read.back()(9), joint_positions_read.back()(10), joint_positions_read.back()(11);

    generate_lf_motion(lf_joint_start, lf_joint_positions, lf_foot_height);
    generate_rf_motion_1(rf_joint_start, rf_joint_positions, rf_foot_height);
    generate_rh_motion_1(rh_joint_start, rh_joint_positions, rh_foot_height);
    generate_lh_motion_1(lh_joint_start, lh_joint_positions, lh_foot_height);

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

    SaveAsFile_no_footheight("catch_a_ball.txt",joint_positions_total);
    SaveAsFile("catch_a_ball_read.txt",joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);

}
