/*
 *  kinematicsTest.cpp
 *  Descriotion:walking to the stairs
 *
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

Eigen::MatrixXd jacobian;
QuadrupedKinematics QK;

double Step_dis = 0.30;//step distance
double height_1 = 0.18;//stairs
double height_2 = 0.05;//plain

double period_t = 3.8;//period 6
double adjust_t = 1.2;//pose adjust time;

double knee_height = 0.00;//knee down height
double down_height = 0.00;//swing down height


double forward_d = 0.08;//adjust the COM

typedef typename curves::Time Time;
typedef typename curves::PolynomialSplineQuinticVector3Curve::ValueType valuetype;

double dt = 0;
double dt_ = 0;
double delta_t = 0.0025;

void SaveAsFile(const std::vector<JointPositions>& joint_position_vector, const std::vector<valuetype>& lf_foot_height,
                const std::vector<valuetype>& rf_foot_height, const std::vector<valuetype>& rh_foot_height,
                const std::vector<valuetype>& lh_foot_height)
{
    std::ofstream invFile;
    std::cout << "Saving the data" << std::endl;
    std::string file_name;
    file_name = "go_upstairs.txt";
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
    file_name = "";

    std::cout << "success store the file " << std::endl;
}

unsigned long j;//curves number;
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
    lf_leg_trajectory.resize(10);
    times_of_lf.resize(10);
    lf_leg_position.resize(10);

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

    //step 3: LF legs get from -0.45 to -0.35, and RH gets from -0.35 to -0.45, each one is adjust_t/2.
    //firstly up and then down

    times_of_lf[4].push_back(0.0);
    lf_leg_position[4].push_back(valuetype(x_start + Step_dis / 2 + forward_d , y_start, z_start + height_2));

    times_of_lf[4].push_back(adjust_t);
    lf_leg_position[4].push_back(valuetype(x_start + Step_dis / 2 - forward_d , y_start, z_start + height_2));

    lf_leg_trajectory[4].fitCurve(times_of_lf[4], lf_leg_position[4]);

    //step 4: LF leg moves with -0.35m;
    times_of_lf[5].push_back(period_t * 0);
    lf_leg_position[5].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_lf[5].push_back(period_t * 1);
    lf_leg_position[5].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[5].fitCurve(times_of_lf[5], lf_leg_position[5]);

    //step 5: LF leg moves from -0.35 to -0.45;
    times_of_lf[6].push_back(0.0);
    lf_leg_position[6].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start + height_2));

    times_of_lf[6].push_back(adjust_t);
    lf_leg_position[6].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[6].fitCurve(times_of_lf[6], lf_leg_position[6]);

    //step 6: LF legs moves as normal;
    times_of_lf[7].push_back(period_t * 0);
    lf_leg_position[7].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start  + height_2));

    times_of_lf[7].push_back(period_t * 1);
    lf_leg_position[7].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start  + height_2));

    lf_leg_trajectory[7].fitCurve(times_of_lf[7], lf_leg_position[7]);

    //step 7: LF leg unchanged;
    times_of_lf[8].push_back(adjust_t * 0);
    lf_leg_position[8].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start + height_2));

    times_of_lf[8].push_back(adjust_t * 1);
    lf_leg_position[8].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[8].fitCurve(times_of_lf[8], lf_leg_position[8]);

    //step 8: LF leg moves as normal;
    times_of_lf[9].push_back(period_t * 0);
    lf_leg_position[9].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start + height_2));

    times_of_lf[9].push_back(period_t * 1);
    lf_leg_position[9].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_2));

    lf_leg_trajectory[9].fitCurve(times_of_lf[9], lf_leg_position[9]);



    JointPositionsLimb joints;
    valuetype lf_leg_valuetype;
    Pose lf_leg_pose;

    dt = 0;
    dt_ = 0;
    /***** evaluate the leg position ******/

    while(dt <= 4 * (period_t + adjust_t))
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
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - 3 * period_t - 3 * adjust_t;
            lf_leg_trajectory[8].evaluate(lf_leg_valuetype, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
            dt_ = dt - 3 * period_t - 4 * adjust_t;
            lf_leg_trajectory[9].evaluate(lf_leg_valuetype, dt_);
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
void generate_rh_motion(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rh_joint_positions, std::vector<valuetype>& rh_foot_position)
{
    /*******planning the RH leg *******/
    double x_start = -height_x;
    double y_start = -height_y;
    double z_start = height_z;

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;
    times_of_rh.resize(10);
    rh_leg_position.resize(10);
    rh_leg_trajectory.resize(10);
    //step 1: RH leg changes from -0.45 to -0.35, when time is adjust_t/2;
    times_of_rh[0].push_back(0);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rh[0].push_back(adjust_t);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    rh_leg_trajectory[0].fitCurve(times_of_rh[0], rh_leg_position[0]);

    //step 2: RH legs moves L/3 with 0.1m higher than the z_start;
    times_of_rh[1].push_back(0);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rh[1].push_back(period_t);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rh_leg_trajectory[1].fitCurve(times_of_rh[1], rh_leg_position[1]);//5s

    //step 3:RH stand up before adjust_t/2 and keep this joint ;
    times_of_rh[2].push_back(0);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rh[2].push_back(adjust_t);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rh_leg_trajectory[2].fitCurve(times_of_rh[2], rh_leg_position[2]);//6.2s


    //step 4:RH leg moves;
    j = 3;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//7.15s

//    valuetype value;
//    for (double t = 0; t < 0.25 * period_t; t = t + 0.1) {
//        rh_leg_trajectory[j].evaluate(value, t);
//        std::cout <<"In time" << t << "value is " << value.x() << std::endl;
//    }

    //step 4-1
    j = 8;
    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 8.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//1.9s
//    for (double t = 0; t < 0.5 * period_t; t = t + 0.1) {
//        rh_leg_trajectory[j].evaluate(value, t);
//        std::cout <<"In time" << t << "value is " << value.x() << std::endl;
//    }

    //step 4-2

    j = 9;

    times_of_rh[j].push_back(0.0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_rh[j].push_back(period_t * 4.0 / 16);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);//5s

    //step 5: rh legs unchanged;
    j = 4;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
    //    std::cout << "finish step 5" <<std::endl;

    //step 6: rh leg moves l/3 as before;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 7: rh leg unchanged;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
    //    std::cout << "finish step 7" <<std::endl;

    //step 8:
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start + height_2));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start + height_2));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
    //    std::cout << "finish step 8" <<std::endl;



    Pose rh_leg_pose;
    valuetype rh_leg_joint;
    JointPositionsLimb joints;
    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 4 * (period_t + adjust_t))
    {
//        std::cout <<setprecision(2) << fixed << "time is " << dt;
        if(dt <= adjust_t)//0-adjust_t
        {
//            std::cout <<"step 1" << std::endl;
            rh_leg_trajectory[0].evaluate(rh_leg_joint, dt);
        }else if(dt > adjust_t && dt <=(period_t + adjust_t))
        {
//            std::cout <<"step 2" << std::endl;
            dt_ = dt - adjust_t;
            rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }
        else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))
        {
//            std::cout <<"step 3" << std::endl;
            dt_ = dt - period_t - adjust_t;
            rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }
        else if( dt >(period_t + 2 * adjust_t) && dt <= (4.0 / 16 * period_t + period_t + 2 * adjust_t)) //6.2 ~7.15
        {
//            std::cout <<"step 4";
            dt_ = dt - (period_t + 2 * adjust_t);
            rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
//            std::cout << rh_leg_joint.x() << std::endl;
        }

        else if( dt > (0.25 * period_t + period_t + 2 * adjust_t) && dt <= (0.75 * period_t + period_t + 2 * adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout <<"step 5";
            dt_ = dt - (0.25 * period_t + period_t + 2 * adjust_t);
            rh_leg_trajectory[8].evaluate(rh_leg_joint, dt_);
//            std::cout << rh_leg_joint.x() << std::endl;
        }

        else if( dt > (0.75 * period_t + period_t + 2 * adjust_t) && dt <= (period_t + period_t + 2 * adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout <<"step 6";
            dt_ = dt - (0.75 * period_t + period_t + 2 * adjust_t);
            rh_leg_trajectory[9].evaluate(rh_leg_joint, dt_);
//            std::cout << rh_leg_joint.x() << std::endl;
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
//            std::cout <<"step 7" << std::endl;
            dt_ = dt - period_t * 2 - 2 * adjust_t;
            rh_leg_trajectory[4].evaluate(rh_leg_joint, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
//            std::cout <<"step 8" << std::endl;
            dt_ = dt - 2 * period_t - 3 * adjust_t;
            rh_leg_trajectory[5].evaluate(rh_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
//            std::cout <<"step 9" << std::endl;
            dt_ = dt - 3 * period_t - 3 * adjust_t;
            rh_leg_trajectory[6].evaluate(rh_leg_joint, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
//            std::cout <<"step 10" << std::endl;
            dt_ = dt - 3 * period_t - 4 * adjust_t;
            rh_leg_trajectory[7].evaluate(rh_leg_joint, dt_);
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
void generate_rf_motion(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& rf_joint_positions, std::vector<valuetype>& rf_foot_position)
{
//    std::cout << "rf_joint_position size is " <<rf_joint_positions.size() << std::endl;
    double x_start = height_x;
    double y_start = -height_y;
    double z_start = height_z;
    /*******planning the RF leg *******/

    std::vector<std::vector<Time>> times_of_rf;
    times_of_rf.resize(10);
    std::vector<std::vector<valuetype>>rf_leg_position;
    rf_leg_position.resize(10);
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;
    rf_leg_trajectory.resize(10);

    //step 1: RF changes from -0.35 to -0.45 with respect the first adjust/2;
    j = 0;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 2: rf leg leg unchanges;
    j = 1;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 3: RF unchanges;
    j = 2;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 4: RF unchanges;
    j = 3;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 5: RF unchanges;
    j = 4;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 6: RF moves swing;
    j = 5;

    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start));

    times_of_rf[j].push_back(period_t * 4 / 16);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);


    //step 2-1
    j = 8;

    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_rf[j].push_back(period_t * 8 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//1.9s

    //step 2-2

    j = 9;
    times_of_rf[j].push_back(0.0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_1));

    times_of_rf[j].push_back(period_t * 4 / 16);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);//5s

    //step 7: RF unchanges;
    j = 6;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(adjust_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    //step 8: RF unchanges;
    j = 7;
    times_of_rf[j].push_back(0);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    times_of_rf[j].push_back(period_t);
    rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start + height_2));

    rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

    /***** evaluate the leg position ******/
    Pose rf_leg_pose;
    valuetype rf_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 4 * (period_t + adjust_t))
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
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
//            std::cout << "step 3" << std::endl;
            dt_ = dt - period_t - adjust_t;
            rf_leg_trajectory[2].evaluate(rf_leg_joint, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//adjust_t + period_t * 2~ 2 * period_t + 2 * adjust_t
        {
//            std::cout << "step 4" << std::endl;
            dt_ = dt - period_t - 2 * adjust_t;
            rf_leg_trajectory[3].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
//            std::cout << "step 5" << std::endl;
            dt_ = dt - period_t * 2 - 2 * adjust_t;
            rf_leg_trajectory[4].evaluate(rf_leg_joint, dt_);
        }else if( dt > (2 * period_t + 3 * adjust_t) && dt <= (2.25 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout << "step 6" << std::endl;
            dt_ = dt - (2 * period_t + 3 * adjust_t);
            rf_leg_trajectory[5].evaluate(rf_leg_joint, dt_);
        }else if( dt > (2.25 * period_t + 3 * adjust_t) && dt <= (2.75 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout << "step 7" << std::endl;
            dt_ = dt - (2.25 * period_t + 3 * adjust_t);
            rf_leg_trajectory[8].evaluate(rf_leg_joint, dt_);
        }else if( dt > (2.75 * period_t + 3 * adjust_t) && dt <= (3 * period_t + 3 * adjust_t)) //adjust_t~adjust_t + period_t
        {
//            std::cout << "step 8" << std::endl;
            dt_ = dt - (12.0 / 16 * period_t + 2 * period_t + 3 * adjust_t);
            rf_leg_trajectory[9].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
//            std::cout << "step 9" << std::endl;
            dt_ = dt - 3 * period_t - 3 * adjust_t;
            rf_leg_trajectory[6].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
//            std::cout << "step 10" << std::endl;
            dt_ = dt - 3 * period_t - 4 * adjust_t;
            rf_leg_trajectory[7].evaluate(rf_leg_joint, dt_);
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
void generate_lh_motion(double height_1, double height_2, double height_x, double height_y, double height_z, std::vector<JointPositionsLimb>& lh_joint_positions, std::vector<valuetype>& lh_foot_position)
{
    /*******planning the LH leg *******/

    double x_start = -height_x;
    double y_start = height_y;
    double z_start = height_z;
    std::vector<std::vector<Time>> times_of_lh;
    std::vector<std::vector<valuetype>> lh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

    times_of_lh.resize(10);
    lh_leg_position.resize(10);
    lh_leg_trajectory.resize(10);

    //step 1: lh leg unchanged;
    j = 0;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start - down_height));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 2: lh leg moves l/3 with height -0.45m;
    j = 1;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 3: LH leg unchanged;
    j = 2;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 4: lh leg moves l/3 with height -0.45m;
    j = 3;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 5: lh leg knee down;
    j = 4;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start + knee_height));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 6: lh moves l/3 with -0.35;
    j = 5;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, z_start + knee_height));

    times_of_lh[j].push_back(period_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + knee_height));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 7: lh stand up in front adjust_t/2
    j = 6;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, z_start + knee_height));

    times_of_lh[j].push_back(adjust_t);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    //step 8: lh moves swing
    j = 7;
    times_of_lh[j].push_back(0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start));

    times_of_lh[j].push_back(period_t * 4 / 16);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

    j = 8;

    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(period_t * 8 / 16);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//1.9s

    //step 2-2

    j = 9;
    times_of_lh[j].push_back(0.0);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_1));

    times_of_lh[j].push_back(period_t * 4 / 16);
    lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, z_start + height_2));

    lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);//5s

    /***** evaluate the leg position ******/
    Pose lh_leg_pose;
    valuetype lh_leg_joint;
    JointPositionsLimb joints;

    /***** evaluate the leg position ******/

    dt = 0;
    dt_ = 0;
    while(dt <= 4 * (period_t + adjust_t))
    {
        if(dt <= adjust_t)//0-adjust_t
        {
            lh_leg_trajectory[0].evaluate(lh_leg_joint, dt);
        }else if( dt > adjust_t && dt <= (period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - adjust_t;
            lh_leg_trajectory[1].evaluate(lh_leg_joint, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
            dt_ = dt - period_t - adjust_t;
            lh_leg_trajectory[2].evaluate(lh_leg_joint, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//adjust_t + period_t * 2~ 2 * period_t + 2 * adjust_t
        {
            dt_ = dt - period_t - 2 * adjust_t;
            lh_leg_trajectory[3].evaluate(lh_leg_joint, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
            dt_ = dt - period_t * 2 - 2 * adjust_t;
            lh_leg_trajectory[4].evaluate(lh_leg_joint, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
            dt_ = dt - 2 * period_t - 3 * adjust_t;
            lh_leg_trajectory[5].evaluate(lh_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
            dt_ = dt - 3 * period_t - 3 * adjust_t;
            lh_leg_trajectory[6].evaluate(lh_leg_joint, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(3 * period_t + 0.25 * period_t + 4 * adjust_t))
        {
            dt_ = dt - 3 * period_t - 4 * adjust_t;
            lh_leg_trajectory[7].evaluate(lh_leg_joint, dt_);
        }else if( dt > (3 * period_t + 0.25 * period_t + 4 * adjust_t) && dt <= (3 * period_t + 0.75 * period_t + 4 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (3 * period_t + 0.25 * period_t + 4 * adjust_t);
            lh_leg_trajectory[8].evaluate(lh_leg_joint, dt_);
        }else if( dt > (3 * period_t + 0.75 * period_t + 4 * adjust_t) && dt <= (3 * period_t + period_t + 4 * adjust_t)) //adjust_t~adjust_t + period_t
        {
            dt_ = dt - (3 * period_t + 0.75 * period_t + 4 * adjust_t);
            lh_leg_trajectory[9].evaluate(lh_leg_joint, dt_);
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

int main(int argc, char **argv)
{

    //step 1, LF->0.13, RF->0.13, LH->0, RH->0
    double height_1 = 0.18;//up stairs maximum height
    double height_2 = 0.13;//plain
    double height_3 = 0.05;

    double height_x = 0.382653;
    double height_y = 0.305;
    double height_z = -0.48;

    std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;



    std::cout << "let us start" << std::endl;
    std::vector<JointPositionsLimb> lf_joint_positions, rh_joint_positions, rf_joint_positions, lh_joint_positions;

    generate_lf_motion(height_1, height_2, height_x, height_y, height_z, lf_joint_positions, lf_foot_height);
    generate_rf_motion(height_1, height_2, height_x, height_y, height_z, rf_joint_positions, rf_foot_height);
    generate_rh_motion(height_3, 0, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    generate_lh_motion(height_3, 0, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);
//    std::cout << lf_joint_positions.size() << std::endl;
//    for (unsigned int i = 0; i < lf_joint_positions.size(); i++) {
//        std::cout << lf_joint_positions[i] << std::endl;
//    }

    //step 2
    generate_lf_motion(height_1, height_2, height_x, height_y, height_z + height_2, lf_joint_positions, lf_foot_height);
    generate_rf_motion(height_1, height_2, height_x, height_y, height_z + height_2, rf_joint_positions, rf_foot_height);
    generate_rh_motion(height_3, 0, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    generate_lh_motion(height_3, 0, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);
    std::cout << "lf_joint_position 2 size is " << lf_joint_positions.size() << std::endl;
    std::cout << "rf_joint_position 2 size is " << rf_joint_positions.size() << std::endl;
    std::cout << "rh_joint_position 2 size is " << rh_joint_positions.size() << std::endl;
    std::cout << "lh_joint_position 2 size is " << lh_joint_positions.size() << std::endl;


    //step 3
    generate_lf_motion(height_3, 0, height_x, height_y, height_z + 2 * height_2, lf_joint_positions, lf_foot_height);
    generate_rf_motion(height_3, 0, height_x, height_y, height_z + 2 * height_2, rf_joint_positions, rf_foot_height);
    generate_rh_motion(height_1, height_2, height_x, height_y, height_z, rh_joint_positions, rh_foot_height);
    generate_lh_motion(height_1, height_2, height_x, height_y, height_z, lh_joint_positions, lh_foot_height);
    std::cout << "lf_joint_position 3 size is " << lf_joint_positions.size() << std::endl;
    std::cout << "rf_joint_position 3 size is " << rf_joint_positions.size() << std::endl;
    std::cout << "rh_joint_position 3 size is " << rh_joint_positions.size() << std::endl;
    std::cout << "lh_joint_position 3 size is " << lh_joint_positions.size() << std::endl;

    //step 4
    generate_lf_motion(height_3, 0, height_x, height_y, height_z + 2 * height_2, lf_joint_positions, lf_foot_height);
    generate_rf_motion(height_3, 0, height_x, height_y, height_z + 2 * height_2, rf_joint_positions, rf_foot_height);
    generate_rh_motion(height_1, height_2, height_x, height_y, height_z + height_2, rh_joint_positions, rh_foot_height);
    generate_lh_motion(height_1, height_2, height_x, height_y, height_z + height_2, lh_joint_positions, lh_foot_height);
    std::cout << "lf_joint_position 4 size is " << lf_joint_positions.size() << std::endl;
    std::cout << "rf_joint_position 4 size is " << rf_joint_positions.size() << std::endl;
    std::cout << "rh_joint_position 4 size is " << rh_joint_positions.size() << std::endl;
    std::cout << "lh_joint_position 4 size is " << lh_joint_positions.size() << std::endl;

    /******Now we store the joint angles to the txt file *******/

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

    SaveAsFile(joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);


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
