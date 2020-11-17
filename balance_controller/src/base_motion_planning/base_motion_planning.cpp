/*
 *  kinematicsTest.cpp
 *  Descriotion:
 *
 *  Created on: date, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */


#include "base_motion_planning/base_motion_planning.h"
namespace balance_controller {

//template<typename T>
//void BaseMotionPlanning::SaveAsFile(const std::string& file_name, const T& joint_position_vector)
//{
//    std::ofstream invFile;
//    std::cout << "Saving the data" << std::endl;
//    invFile.open(file_name);
//    if(invFile.fail())
//    {
//        std::cerr << "The file cannot be opened!";
//    }

//    double time_ = 0;
//    for (unsigned int i = 0; i < joint_position_vector.size(); i++) {
//        invFile <<time_ << " " << joint_position_vector[i] << " ";
//        invFile << "\r\n";
//        time_ = time_ + 0.0025;
//    }
//    invFile.close();

//    std::cout << "success store the file in"<< file_name << std::endl;
//}

void BaseMotionPlanning::save_base_motion_planning_data(const std::string& file_name, const std::vector<valuetype>& base_motion_planning)
{
    std::ofstream invFile;
    std::cout << "Saving the data" << std::endl;
    invFile.open(file_name);
    if(invFile.fail())
    {
        std::cerr << "The file cannot be opened!";
    }

    double time_ = 0;
    for (unsigned int i = 0; i < base_motion_planning.size(); i++) {
        invFile <<time_ << " " << base_motion_planning[i].transpose() << " ";
        invFile << "\r\n";
        time_ = time_ + 0.0025;
    }
    invFile.close();
    std::cout << "success store the file " << std::endl;
}

void BaseMotionPlanning::SaveAsFile_with_footheight(const std::string &file_name, const std::vector<JointPositions>& joint_position_vector, const std::vector<valuetype>& lf_foot_height,
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

BaseMotionPlanning::BaseMotionPlanning()
{
    period_t = 5.8;
    adjust_t = 2.4;
    delta_t = 0.0025;
    height_x = 0.382653;
    height_y = 0.305;
    height_z = -0.5;
    forward_d = 0.08;
    y_direction = 0.05;
    adjust_height = 0.03;
    step_dis = 0.3;
    base_pose.getPosition().setZero();
    base_pose.getRotation().setIdentity();
    lf_leg_info.is_contact = true;
    rf_leg_info.is_contact = true;
    rh_leg_info.is_contact = true;
    lh_leg_info.is_contact = true;
}
BaseMotionPlanning::~BaseMotionPlanning()
{
    std::cout << "finish the base motion plannging" << std::endl;
}
valuetype BaseMotionPlanning::generate_lf_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position)
{
    double x_start = height_x;
    double y_start = height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_lf;
    std::vector<std::vector<valuetype>>lf_leg_position;

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
    std::cout << "finish lf planning" << std::endl;
    return pos_final;
}
valuetype BaseMotionPlanning::generate_rf_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
    double rf_x_start = height_x;
    double rf_y_start = -height_y;
    double rf_z_start = height_z;

    std::vector<std::vector<Time>> times_of_rf;
    std::vector<std::vector<valuetype>>rf_leg_position;

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

    std::cout << "finish rf step 1" << std::endl;
    return pos_final;

}
valuetype BaseMotionPlanning::generate_lh_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = -height_x;
    y_start = height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;

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
    std::cout << "finish the lh leg planning" << std::endl;
    return pos_final;
}
valuetype BaseMotionPlanning::generate_rh_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    double x_start;
    double y_start;
    double z_start;
    x_start = -height_x;
    y_start = -height_y;
    z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;

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
    std::cout << "finish rh leg planning" << std::endl;
    return pos_final;
}

void BaseMotionPlanning::base_motion(const Pose& initial_pose)
{
    valuetype base_pose = initial_pose.getPosition().toImplementation();

    std::vector<std::vector<Time>> times_of_base;
    std::vector<std::vector<valuetype>> base_position_vector;

    unsigned long step_num = 8;
    times_of_base.resize(step_num);
    base_position_vector.resize(step_num);
    base_trajectory.resize(step_num);

    unsigned long j;

    j = 0;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d, base_pose.y() - y_direction, base_pose.z()));

    times_of_base[j].push_back(adjust_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d, base_pose.y() - y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    //step 2: lf leg moves
    j = 1;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d, base_pose.y() - y_direction, base_pose.z()));
    times_of_base[j].push_back(period_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d + step_dis / 3, base_pose.y() - y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    j = 2;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d + step_dis / 3, base_pose.y() - y_direction, base_pose.z()));
    times_of_base[j].push_back(adjust_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    //step 2: rh leg moves
    j = 3;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    times_of_base[j].push_back(period_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + 2 * step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    j = 4;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + 2 * step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    times_of_base[j].push_back(adjust_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d + 2 * step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    //step 3;rf leg moves;
    j = 5;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d + 2 * step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    times_of_base[j].push_back(period_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d + 3 * step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    j = 6;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() - forward_d + 3 * step_dis / 3, base_pose.y() + y_direction, base_pose.z()));
    times_of_base[j].push_back(adjust_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + 3 * step_dis / 3, base_pose.y() - y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);

    //step 4: lh leg moves
    j = 7;
    times_of_base[j].push_back(0.0);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + 3 * step_dis / 3, base_pose.y() - y_direction, base_pose.z()));
    times_of_base[j].push_back(adjust_t);
    base_position_vector[j].push_back(valuetype(base_pose.x() + forward_d + 4 * step_dis / 3, base_pose.y() - y_direction, base_pose.z()));
    base_trajectory[j].fitCurve(times_of_base[j], base_position_vector[j]);
    std::cout << "finish the base planning" << std::endl;
}

valuetype BaseMotionPlanning::GetBasePosition(double time)
{
    valuetype evaluate_value;
    std::vector<valuetype> base_motion_vector;
    double dt = MapTimeto4PeriodtandAdjust(time);
    std::cout << "the time dt is " << dt << std::endl;
    double dt_ = 0;
    if(dt <= 4 *(period_t + adjust_t))
    {
        std::cout << "in the if function" << std::endl;
        if(dt <= adjust_t)
        {
            base_trajectory[0].evaluate(evaluate_value, dt);
        }else if(dt > adjust_t && dt <= (adjust_t + period_t))
        {
            dt_ = dt - adjust_t;
            base_trajectory[1].evaluate(evaluate_value, dt_);
        }else if (dt > (adjust_t + period_t) && dt <= (adjust_t * 2 + period_t)) {
            dt_ = dt - (adjust_t + period_t);
            base_trajectory[2].evaluate(evaluate_value, dt_);
        }else if (dt > (adjust_t * 2 + period_t) && dt <= (adjust_t * 2 + period_t * 2)){
            dt_ = dt - (adjust_t * 2 + period_t);
            base_trajectory[3].evaluate(evaluate_value, dt_);
        }else if (dt > (adjust_t * 2 + period_t * 2) && dt <= (adjust_t * 3 + period_t * 2)){
            dt_ = dt - (adjust_t * 2 + period_t * 2);
            base_trajectory[4].evaluate(evaluate_value, dt_);
        }else if (dt > (adjust_t * 3 + period_t * 2) && dt <= (adjust_t * 3 + period_t * 3)){
            dt_ = dt - (adjust_t * 3 + period_t * 2);
            base_trajectory[5].evaluate(evaluate_value, dt_);
        }else if (dt > (adjust_t * 3 + period_t * 3) && dt <= (adjust_t * 4 + period_t * 3)){
            dt_ = dt - (adjust_t * 3 + period_t * 3);
            base_trajectory[6].evaluate(evaluate_value, dt_);
        }else if(dt > (adjust_t * 4 + period_t * 3) && dt <=(adjust_t * 4 + period_t * 4)){
            dt_ = dt - (adjust_t * 4 + period_t * 3);
            base_trajectory[7].evaluate(evaluate_value, dt_);
        }
    }else{
        std::cout << "please confirm the time is within 4 * (period + adjust), which is " << 4 * (period_t + adjust_t) << std::endl;
    }
    return evaluate_value;
}
double BaseMotionPlanning::MapTimeto4PeriodtandAdjust(double time)
{
    double map_num = double(time)/( 4 * (period_t + adjust_t));
    int num = int(time / (4 * (period_t + adjust_t)));
    double end_time = 4 * (period_t + adjust_t) * (map_num - num);
    return end_time;
}

std::vector<leg_info> BaseMotionPlanning::GetFootPosition(double time)
{
    double dt = MapTimeto4PeriodtandAdjust(time);
//    std::cout << " the time dt is " << dt << std::endl;
    double dt_ = 0;

    std::vector<leg_info> foot_position;

    valuetype evaluate_value;
    if(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            lf_leg_trajectory[0].evaluate(evaluate_value, dt);
            lf_leg_info.is_contact = true;
        }else if(dt > adjust_t && dt <= (0.25 * period_t + adjust_t))
        {
            dt_ = dt - adjust_t;
            lf_leg_trajectory[1].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = false;
        }else if(dt > (0.25 * period_t + adjust_t) && dt <= (0.75 * period_t + adjust_t))
        {
            dt_ = dt - (0.25 * period_t + adjust_t);
            lf_leg_trajectory[2].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = false;
        }else if(dt > (0.75 * period_t + adjust_t) && dt <= (period_t + adjust_t))
        {
            dt_ = dt - (0.75 * period_t + adjust_t);
            lf_leg_trajectory[3].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = false;
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            lf_leg_trajectory[4].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = true;
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            lf_leg_trajectory[5].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = true;
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            lf_leg_trajectory[6].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = true;
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            lf_leg_trajectory[7].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = true;
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            lf_leg_trajectory[8].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = true;
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            lf_leg_trajectory[9].evaluate(evaluate_value, dt_);
            lf_leg_info.is_contact = true;
        }else {
            std::cout << "nothing" << std::endl;
        }
    }else {
        std::cout << "LF LEG: please confirm the time is within 4 * (period + adjust), which is " << 4 * (period_t + adjust_t) << std::endl;
    }
    lf_leg_info.foot_position = evaluate_value;
    foot_position.push_back(lf_leg_info);

    if(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)
        {
            rf_leg_trajectory[0].evaluate(evaluate_value, dt);
            rf_leg_info.is_contact = true;
        }else if(dt > adjust_t && dt <=(period_t + adjust_t) )
        {
            dt_ = dt - adjust_t;
            rf_leg_trajectory[1].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = true;
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + adjust_t);
            rf_leg_trajectory[2].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = true;
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (period_t + 2 * adjust_t);
            rf_leg_trajectory[3].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = true;
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 2 * adjust_t);
            rf_leg_trajectory[4].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = true;
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(2.25 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rf_leg_trajectory[5].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = false;
        }else if(dt > (2.25 * period_t + 3 * adjust_t) && dt <=(2.75 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2.25 * period_t + 3 * adjust_t);
            rf_leg_trajectory[6].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = false;
        }else if(dt > (2.75 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - (2.75 * period_t + 3 * adjust_t);
            rf_leg_trajectory[7].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = false;
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 3 * adjust_t);
            rf_leg_trajectory[8].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = true;
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3 * period_t + 4 * adjust_t);
            rf_leg_trajectory[9].evaluate(evaluate_value, dt_);
            rf_leg_info.is_contact = true;
        }else {
            std::cout << "nothing" << std::endl;
        }
    }else{
        std::cout << "RF LEG: please confirm the time is within 4 * (period + adjust), which is " << 4 * (period_t + adjust_t) << std::endl;
    }
    rf_leg_info.foot_position = evaluate_value;
    foot_position.push_back(rf_leg_info);

    rh_leg_info.is_contact = true;
    if(dt <= 4 * (period_t + adjust_t))
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
            rh_leg_info.is_contact = false;
        }else if(dt > (1.25 * period_t + 2 * adjust_t) && dt <=(1.75 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.25 * period_t + 2 * adjust_t);
            rh_leg_trajectory[4].evaluate(evaluate_value, dt_);
            rh_leg_info.is_contact = false;
        }else if(dt > (1.75 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))
        {
            dt_ = dt - (1.75 * period_t + 2 * adjust_t);
            rh_leg_trajectory[5].evaluate(evaluate_value, dt_);
            rh_leg_info.is_contact = false;
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
    }else{
        std::cout << "RH LEG: please confirm the time is within 4 * (period + adjust), which is " << 4 * (period_t + adjust_t) << std::endl;
    }
    rh_leg_info.foot_position = evaluate_value;
    foot_position.push_back(rh_leg_info);

    lh_leg_info.is_contact = true;
    if(dt <= 4 * (period_t + adjust_t))
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
            lh_leg_info.is_contact = false;
        }else if(dt > (3.25 * period_t + 4 * adjust_t) && dt <=(3.75 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3.25 * period_t + 4 * adjust_t);
            lh_leg_trajectory[8].evaluate(evaluate_value, dt_);
            lh_leg_info.is_contact = false;
        }else if(dt > (3.75 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - (3.75 * period_t + 4 * adjust_t);
            lh_leg_trajectory[9].evaluate(evaluate_value, dt_);
            lh_leg_info.is_contact = false;
        }else {
            std::cout << "nothing" << std::endl;
        }
    }else{
        std::cout << "RH LEG: please confirm the time is within 4 * (period + adjust), which is " << 4 * (period_t + adjust_t) << std::endl;
    }
    lh_leg_info.foot_position = evaluate_value;
    foot_position.push_back(lh_leg_info);
    return foot_position;
}

}//namespace
