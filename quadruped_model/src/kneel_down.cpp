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

int main(int argc, char **argv)
{

    valuetype lf_start, rf_start, rh_start, lh_start;

    std::ifstream readfile;
    double time_read;
    readfile.open("/home/kun/catkin_ws_dependency/rolling_to_the_stairs_read.txt");
    quadruped_model::JointPositions joint_position_file;
    std::vector<quadruped_model::JointPositions> joint_position_collection;
    for(int i = 0; !readfile.eof(); i++)
    {
        readfile >> time_read >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
            >> joint_position_file(3)>> joint_position_file(4)
            >> joint_position_file(5)>> joint_position_file(6)
            >> joint_position_file(7)>> joint_position_file(8)
            >> joint_position_file(9)>> joint_position_file(10)
            >> joint_position_file(11) >> lf_start.x() >> lf_start.y() >> lf_start.z()>> rf_start.x() >> rf_start.y() >> rf_start.z() >> rh_start.x() >> rh_start.y() >>rh_start.z()
            >>lh_start.x() >> lh_start.y() >> lh_start.z();
    }
    readfile.close();

    std::cout << lf_start << rf_start << lh_start << rh_start << std::endl;


    curves::PolynomialSplineQuinticVector3Curve lf_swing_planning, rf_swing_planning,lh_swing_planning,rh_swing_planning;

    valuetype lf_end ,rf_end, lh_end, rh_end;
    lf_end << lf_start.x(), 0.305, lf_start.z();
    rf_end << rf_start.x(), -0.305, rf_start.z();
    lh_end << lh_start.x(), 0.305, lh_start.z();
    rh_end << rh_start.x(), -0.305, rh_start.z();

    std::vector<Time> time;
    std::vector<valuetype> lf_foot_pos, rf_foot_pos, rh_foot_pos, lh_foot_pos;
    Pose lf_pose, rf_pose, lh_pose, rh_pose;
    JointPositionsLimb lf_joints, rf_joints, lh_joints, rh_joints;
    std::vector<JointPositionsLimb> rh_joint_positions, rf_joint_positions, lh_joint_positions, lf_joint_positions;
    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;

    double time_t = 3.0;

    time.push_back(0.0);
    lf_foot_pos.push_back(lf_start);

    time.push_back(time_t);
    lf_foot_pos.push_back(lf_end);

    lf_swing_planning.fitCurve(time, lf_foot_pos);

    //rf
    rf_foot_pos.push_back(rf_start);
    rf_foot_pos.push_back(rf_end);
    rf_swing_planning.fitCurve(time, rf_foot_pos);

    //rh

    rh_foot_pos.push_back(rh_start);
    rh_foot_pos.push_back(rh_end);
    rh_swing_planning.fitCurve(time, rh_foot_pos);

    //lh
    lh_foot_pos.push_back(lh_start);
    lh_foot_pos.push_back(lh_end);

    lh_swing_planning.fitCurve(time, lh_foot_pos);

    double dt = 0;
    valuetype evaluate_value;

    while(dt < 3)
    {
        //lf
        lf_swing_planning.evaluate(evaluate_value, dt);
        lf_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();

        QK.InverseKinematicsSolve(lf_pose.getPosition(), LimbEnum::LF_LEG, lf_joints, lf_joints, "IN_LEFT");
        lf_joint_positions.push_back(lf_joints);
        lf_foot_height.push_back(evaluate_value);

        //rf

        rf_swing_planning.evaluate(evaluate_value, dt);
        rf_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();

        QK.InverseKinematicsSolve(rf_pose.getPosition(), LimbEnum::RF_LEG, rf_joints, rf_joints, "OUT_LEFT");
        rf_joint_positions.push_back(rf_joints);
        rf_foot_height.push_back(evaluate_value);

        //lh
        lh_swing_planning.evaluate(evaluate_value, dt);
        lh_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();

        QK.InverseKinematicsSolve(lh_pose.getPosition(), LimbEnum::LH_LEG, lh_joints, lh_joints, "OUT_LEFT");
        lh_joint_positions.push_back(lh_joints);
        lh_foot_height.push_back(evaluate_value);

        //rh
        rh_swing_planning.evaluate(evaluate_value, dt);
        rh_pose.getPosition() << evaluate_value.x(), evaluate_value.y(), evaluate_value.z();

        QK.InverseKinematicsSolve(rh_pose.getPosition(), LimbEnum::RH_LEG, rh_joints, rh_joints, "IN_LEFT");
        rh_joint_positions.push_back(rh_joints);
        rh_foot_height.push_back(evaluate_value);

        dt = dt + 0.0025;
    }

    std::vector<valuetype> z_height;
    curves::PolynomialSplineQuinticVector3Curve height_planning;

    z_height.push_back(valuetype(0, 0, -0.42));
    z_height.push_back(valuetype(0, 0, -0.1));

    height_planning.fitCurve(time, z_height);
    dt = 0;

    while(dt < 3)
    {
        height_planning.evaluate(evaluate_value, dt);

        //lf
        lf_end.z() = evaluate_value.z();
        lf_pose.getPosition() << lf_end.x(), lf_end.y(), lf_end.z();

        QK.InverseKinematicsSolve(lf_pose.getPosition(), LimbEnum::LF_LEG, lf_joints, lf_joints, "IN_LEFT");
        lf_joint_positions.push_back(lf_joints);
        lf_foot_height.push_back(lf_end);

        //rf

        rf_end.z() = evaluate_value.z();
        rf_pose.getPosition() << rf_end.x(), rf_end.y(), rf_end.z();

        QK.InverseKinematicsSolve(rf_pose.getPosition(), LimbEnum::RF_LEG, rf_joints, rf_joints, "OUT_LEFT");
        rf_joint_positions.push_back(rf_joints);
        rf_foot_height.push_back(rf_end);

        //lh
        lh_end.z() = evaluate_value.z();
        lh_pose.getPosition() << lh_end.x(), lh_end.y(), lh_end.z();

        QK.InverseKinematicsSolve(lh_pose.getPosition(), LimbEnum::LH_LEG, lh_joints, lh_joints, "OUT_LEFT");
        lh_joint_positions.push_back(lh_joints);
        lh_foot_height.push_back(lh_end);

        //rh
        rh_end.z() = evaluate_value.z();
        rh_pose.getPosition() << rh_end.x(), rh_end.y(), rh_end.z();

        QK.InverseKinematicsSolve(rh_pose.getPosition(), LimbEnum::RH_LEG, rh_joints, rh_joints, "IN_LEFT");
        rh_joint_positions.push_back(rh_joints);
        rh_foot_height.push_back(rh_end);

        dt = dt + 0.0025;
    }

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
    std::string file_name = "kneel_down.txt";
    SaveAsFile(file_name, joint_positions_total);
    file_name = "kneel_down_read.txt";
    SaveAsFile_with_footheight(file_name, joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);
    return 1;
}
