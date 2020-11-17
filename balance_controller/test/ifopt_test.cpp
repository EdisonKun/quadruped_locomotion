//#include <rosbag/bag.h>
//#include <rosbag/view.h>
//#include <sensor_msgs/JointState.h>
//#include <std_msgs/String.h>

//int main(int argc, char *argv[])
//{
//    rosbag::Bag bag;
//    bag.open("/home/kun/rosbags/2020-07-16-15-12-46.bag", rosbag::bagmode::Read);

//    std::string topics("/log/joint_state");
//    rosbag::View view(bag, rosbag::TopicQuery(topics));
//    std::vector<double> joint_temp;
//    joint_temp.resize(12);
//    std::vector<std::vector<double> >joint_angle;

//    rosbag::View::iterator it = view.begin();
//    for (it = view.begin(); it != view.end(); it++) {
//        auto m = *it;
//        sensor_msgs::JointState::ConstPtr s = m.instantiate<sensor_msgs::JointState>();

//        if(s != NULL)
//        {
//            for (unsigned int i = 0; i < 12 ; i++) {
//                joint_temp[i] = s->position.at(i);
//            }
//            joint_angle.push_back(joint_temp);
//        }
//    }

//    std::cout << joint_angle.size() << std::endl;
//    bag.close();

//    return 0;
//}

//test the ifopt
#include "ifopt/variable_set.h"
#include "ifopt/constraint_set.h"
#include "ifopt/cost_term.h"




