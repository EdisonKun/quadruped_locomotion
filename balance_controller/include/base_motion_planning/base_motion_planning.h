#ifndef BASE_MOTION_PLANNING_H_
#define BASE_MOTION_PLANNING_H_

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

typedef typename curves::Time Time;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType valuetype;
typedef struct leg_info
{
    bool is_contact;
    valuetype foot_position;
};
using namespace std;
using namespace quadruped_model;

namespace balance_controller {
class BaseMotionPlanning
{
public:
    BaseMotionPlanning();
    ~BaseMotionPlanning();
    valuetype generate_lf_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& lf_joint_positions, std::vector<valuetype>& lf_foot_position);
    valuetype generate_rf_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position);
    valuetype generate_lh_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position);
    valuetype generate_rh_motion(double height_1, double height_2, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position);
    /**
     * @brief base_motion,planning the base motion to the world frame;
     */

    void base_motion(const Pose& initial_pose);

    /**
     *column vector store;
     */
    template<class T>
    void SaveAsFile(const std::string& file_name, const T& joint_position_vector)
    {
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
                invFile <<time_ ;
                for (unsigned int j = 0; j < joint_position_vector[i].size(); j++) {
                invFile<< " " << joint_position_vector[i].at(j) << " ";
                }                
                invFile << "\r\n";

                time_ = time_ + 0.0025;
            }
            invFile.close();

            std::cout << "success store the file in"<< file_name << std::endl;
        }
    }

    template<class T>
    void SaveAsFile_1_vector(const std::string& file_name, const T& joint_position_vector)
    {
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
                invFile <<time_<<" " << joint_position_vector[i] << " ";
                invFile << "\r\n";
                time_ = time_ + 0.0025;
            }
            invFile.close();

            std::cout << "success store the file in"<< file_name << std::endl;
        }
    }

    void save_base_motion_planning_data(const std::string& file_name, const std::vector<valuetype>& base_motion_planning);
    void SaveAsFile_with_footheight(const std::string &file_name, const std::vector<JointPositions>& joint_position_vector, const std::vector<valuetype>& lf_foot_height,
                                    const std::vector<valuetype>& rf_foot_height, const std::vector<valuetype>& rh_foot_height,
                                    const std::vector<valuetype>& lh_foot_height);

    valuetype GetBasePosition(double time);
    std::vector<leg_info> GetFootPosition(double time);
    double MapTimeto4PeriodtandAdjust(double time);

private:
    leg_info lf_leg_info, rf_leg_info, rh_leg_info, lh_leg_info;
    QuadrupedKinematics QK;
    double period_t;
    double adjust_t;

    double delta_t;

    double height_x;
    double height_y;
    double height_z;

    double forward_d;
    double y_direction;
    double adjust_height;

    double step_dis;

    Pose base_pose;

    std::vector<curves::PolynomialSplineQuinticVector3Curve> base_trajectory;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_leg_trajectory;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;

};//BaseMotionPlanning
}//namespace
#endif
