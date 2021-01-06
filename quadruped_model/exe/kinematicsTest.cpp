/*
 *  kinematicsTest.cpp
 *  Descriotion:This file sends planned joint angles to the quadruped model in rviz;
 *
 *  Created on: date, 2020
 *  Author: EdisonKun
 *  Institute: Harbin Institute of Technology, Shenzhen
 */

#include "quadruped_model/quadrupedkinematics.h"
#include "quadruped_model/QuadrupedModel.hpp"

#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

#include "iostream"
#include "fstream"
#include "sstream"

using namespace std;
using namespace quadruped_model;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_joint_angle");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::Rate loop_rate(200);
    sensor_msgs::JointState joint_state;

    joint_state.header.frame_id = "base_link";
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.name[0] = "front_left_1_joint";
    joint_state.name[1] = "front_left_2_joint";
    joint_state.name[2] = "front_left_3_joint";
    joint_state.name[3] = "front_right_1_joint";
    joint_state.name[4] = "front_right_2_joint";
    joint_state.name[5] = "front_right_3_joint";
    joint_state.name[6] = "rear_right_1_joint";
    joint_state.name[7] = "rear_right_2_joint";
    joint_state.name[8] = "rear_right_3_joint";
    joint_state.name[9] = "rear_left_1_joint";
    joint_state.name[10] = "rear_left_2_joint";
    joint_state.name[11] = "rear_left_3_joint";

    std::ifstream readfile;
//    readfile.open("/home/kun/catkin_ws_dependency/15_walk_and_kneel_down_read.txt");
    readfile.open("/home/kun/catkin_ws_dependency/throw_a_ball_read.txt");

    quadruped_model::JointPositions joint_position_file;
    std::vector<quadruped_model::JointPositions> joint_position_collection;
    double time;
    for(int i = 0; !readfile.eof(); i++)
    {
        readfile >> time >> joint_position_file(0) >> joint_position_file(1)>> joint_position_file(2)
            >> joint_position_file(3)>> joint_position_file(4)
            >> joint_position_file(5)>> joint_position_file(6)
            >> joint_position_file(7)>> joint_position_file(8)
            >> joint_position_file(9)>> joint_position_file(10)
            >> joint_position_file(11) >> time >> time >> time >> time >> time >> time >> time >> time >> time >>time >> time >> time;
        joint_position_collection.push_back(joint_position_file);
    }
    readfile.close();
    joint_position_collection.pop_back();
    ROS_WARN_STREAM(joint_position_collection.size());
    unsigned long i = 0;
    std::cout << "right here" << std::endl;
    std::cout << joint_position_collection.size() << std::endl;

    while(ros::ok())
    {
        if(i < joint_position_collection.size())
        {
            for (unsigned int j = 0; j < 12; j++) {
                joint_state.position[j] = joint_position_collection[i](j);
            }
            joint_pub.publish(joint_state);

        }
        i = i + 1;
//        if(i == joint_position_collection.size() - 1)
//        {
//            i = 0;
//        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}

