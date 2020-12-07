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

int main(int argc, char **argv)
{

  Eigen::MatrixXd jacobian;
  QuadrupedKinematics QK;

  std::vector<valuetype> lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height;
  std::vector<JointPositionsLimb> lf_joint_positions;
  JointPositionsLimb joints;
  valuetype lf_leg_position;
  Pose lf_leg_pose;

    std::vector<JointPositionsLimb> rh_joint_positions;
    Pose rh_leg_pose;
    valuetype rh_leg_joint;

    std::vector<JointPositionsLimb> rf_joint_positions;
    Pose rf_leg_pose;
    valuetype rf_leg_joint;

    std::vector<JointPositionsLimb> lh_joint_positions;
    Pose lh_leg_pose;
    valuetype lh_leg_joint;

    quadruped_model::JointPositions joint_positions;
    std::vector<quadruped_model::JointPositions> joint_positions_total;


  std::cout << "let us begin" << std::endl;


//      lf_foot_height.clear();
//      rf_foot_height.clear();
//      rh_foot_height.clear();
//      lh_foot_height.clear();
  double Step_dis = 0.25;
  double height_1 = 0.10;
  double height_2 = 0.05;
  double stairs_height = 0.05;

  double period_t = 5.8;//period 6
  double adjust_t = 2.4;//pose adjust time;

  double knee_height = 0.00;//knee down height
  double down_height = 0.00;//swing down height

  double height_x = 0.382653;
  double height_y = 0.305;
  double height_z = -0.48;

  double forward_d = 0.05;

  double x_start = height_x;
  double y_start = height_y;
  double z_start = height_z;

  unsigned long j;//curves number;


  /****planning lf leg******/
  //step 1: RF leg up and RH leg down, LF and LH keep the prepare angle unchanged;
  double lf_z_start;

  for(int step_num = 0; step_num < 4; step_num++)
    {
      double Step_dis = 0.30;
      double height_1 = 0.15;
      double height_2 = 0.05;
      double stairs_height = 0.13;

      if(step_num == 0)
        {
          Step_dis = 0.30;
          lf_z_start = z_start;
          height_1 = 0.15;
        }

      if(step_num == 1)
        {
          lf_z_start = z_start + 0.05;
          Step_dis = 0.30;
          height_1 = 0.15;
        }

      double lf_stairs_height;
      lf_stairs_height = stairs_height;
      if(step_num == 2)
        {
          lf_stairs_height = 0;
          lf_z_start = z_start + 0.05 + 0.05;
          height_1 = height_2;
        }
      if(step_num == 3)
        {
          lf_stairs_height = 0;
          height_1 = height_2;
        }

  std::vector<Time> times_adjust_lf;
  std::vector<valuetype> lf_adjust_position;
  std::cout << "num_step is" << step_num << lf_z_start << std::endl;
  double x_start = height_x;
  double y_start = height_y;
  double z_start = height_z;
//  std::cout << "start point is " << x_start - Step_dis / 2 - forward_d << std::endl;
  times_adjust_lf.clear();
  lf_adjust_position.clear();

  times_adjust_lf.push_back(0.0);
  lf_adjust_position.push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, lf_z_start));

  times_adjust_lf.push_back(adjust_t);
  lf_adjust_position.push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, lf_z_start));

  std::vector<curves::PolynomialSplineQuinticVector3Curve> lf_adjust_trajectory;
  lf_adjust_trajectory.resize(8);
  lf_adjust_trajectory[0].fitCurve(times_adjust_lf, lf_adjust_position);

  //step 2:LF leg moves, RH moves with -0.35, the others unchanged;
  std::vector<Time> times_swing_lf;
  std::vector<valuetype> lf_xz_swing_position;

  times_swing_lf.push_back(0.0);
  lf_xz_swing_position.push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, lf_z_start));

  times_swing_lf.push_back(period_t * 4 / 16);
  lf_xz_swing_position.push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, lf_z_start + height_1));

  times_swing_lf.push_back(period_t * 8 / 16);
  lf_xz_swing_position.push_back(valuetype(x_start +forward_d , y_start, lf_z_start + height_2 + height_1));

//  if(step_num == 1)
//    {
//      Step_dis = 0.25;
//    }

  times_swing_lf.push_back(period_t * 12 / 16);
  lf_xz_swing_position.push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, lf_z_start + height_1));

  times_swing_lf.push_back(period_t);
  lf_xz_swing_position.push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[1].fitCurve(times_swing_lf, lf_xz_swing_position);

  //step 3: LF legs get from -0.45 to -0.35, and RH gets from -0.35 to -0.45, each one is adjust_t/2.
  //firstly up and then down
  std::vector<Time> times_down_lf_stance;
  std::vector<valuetype> lf_down_position_stance;

  times_down_lf_stance.push_back(0.0);
  lf_down_position_stance.push_back(valuetype(x_start + Step_dis / 2 + forward_d , y_start, lf_z_start + lf_stairs_height));

//  times_down_lf_stance.push_back(adjust_t / 2);
//  lf_down_position_stance.push_back(valuetype(x_start + Step_dis / 2, y_start, z_start));

  times_down_lf_stance.push_back(adjust_t);
  lf_down_position_stance.push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[2].fitCurve(times_down_lf_stance, lf_down_position_stance);

  //step 4: LF leg moves with -0.35m;
  std::vector<Time> times_stance_lf_035;
  std::vector<valuetype> lf_xz_stance_position_035;

  times_stance_lf_035.push_back(period_t * 0);
  lf_xz_stance_position_035.push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, lf_z_start + lf_stairs_height));

  times_stance_lf_035.push_back(period_t * 1);
  lf_xz_stance_position_035.push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[3].fitCurve(times_stance_lf_035, lf_xz_stance_position_035);

  //step 5: LF leg moves from -0.35 to -0.45;
  std::vector<Time> times_up_lf_stance;
  std::vector<valuetype> lf_up_position_stance;

  times_up_lf_stance.push_back(0.0);
  lf_up_position_stance.push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, lf_z_start + lf_stairs_height));

  times_up_lf_stance.push_back(adjust_t);
  lf_up_position_stance.push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[4].fitCurve(times_up_lf_stance, lf_up_position_stance);

  //step 6: LF legs moves as normal;
  std::vector<Time> times_moves_stance;
  std::vector<valuetype> lf_moves_position;

  times_moves_stance.push_back(period_t * 0);
  lf_moves_position.push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, lf_z_start + lf_stairs_height));

  times_moves_stance.push_back(period_t * 1);
  lf_moves_position.push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[5].fitCurve(times_moves_stance, lf_moves_position);

  //step 7: LF leg unchanged;
  std::vector<Time> times_unchanged_stance_lf;
  std::vector<valuetype> lf_unchanged_position_lf;

  times_unchanged_stance_lf.push_back(adjust_t * 0);
  lf_unchanged_position_lf.push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, lf_z_start + lf_stairs_height));

  times_unchanged_stance_lf.push_back(adjust_t * 1);
  lf_unchanged_position_lf.push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[6].fitCurve(times_unchanged_stance_lf, lf_unchanged_position_lf);

  //step 8: LF leg moves as normal;
  std::vector<Time> times_moves_stance_1;
  std::vector<valuetype> lf_moves_position_1;

  times_moves_stance_1.push_back(period_t * 0);
  lf_moves_position_1.push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, lf_z_start + lf_stairs_height));

  times_moves_stance_1.push_back(period_t * 1);
  lf_moves_position_1.push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, lf_z_start + lf_stairs_height));

  lf_adjust_trajectory[7].fitCurve(times_moves_stance_1, lf_moves_position_1);


//  std::vector<JointPositionsLimb> lf_joint_positions;
//  JointPositionsLimb joints;
//  valuetype lf_leg_position;
//  Pose lf_leg_pose;

  /***** evaluate the leg position ******/
  double dt = 0;
  double dt_ = 0;
  while(dt <= 4 * (period_t + adjust_t))
    {
      if(dt <= adjust_t)//0-adjust_t
        {
          lf_adjust_trajectory[0].evaluate(lf_leg_position, dt);
//          std::cout << lf_leg_position.transpose() << std::endl;
        }else if( dt > adjust_t && dt <= (period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
          dt_ = dt - adjust_t;
          lf_adjust_trajectory[1].evaluate(lf_leg_position, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
          dt_ = dt - period_t - adjust_t;
          lf_adjust_trajectory[2].evaluate(lf_leg_position, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//adjust_t + period_t * 2~ 2 * period_t + 2 * adjust_t
        {
          dt_ = dt - period_t - 2 * adjust_t;
          lf_adjust_trajectory[3].evaluate(lf_leg_position, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
          dt_ = dt - period_t * 2 - 2 * adjust_t;
          lf_adjust_trajectory[4].evaluate(lf_leg_position, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
          dt_ = dt - 2 * period_t - 3 * adjust_t;
          lf_adjust_trajectory[5].evaluate(lf_leg_position, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 3 * adjust_t;
          lf_adjust_trajectory[6].evaluate(lf_leg_position, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 4 * adjust_t;
          lf_adjust_trajectory[7].evaluate(lf_leg_position, dt_);
        }else{
          std::cout << "nothing" << std::endl;
        }
        dt = dt + 0.0025;
//        std::cout << "time is " << dt << std::endl;
        lf_leg_pose.getPosition().x() = lf_leg_position.x();
        lf_leg_pose.getPosition().y() = lf_leg_position.y();
        lf_leg_pose.getPosition().z() = lf_leg_position.z();

        QK.InverseKinematicsSolve(lf_leg_pose.getPosition(),LimbEnum::LF_LEG, joints, joints, "IN_LEFT");
        lf_joint_positions.push_back(joints);
        lf_foot_height.push_back(lf_leg_position);
    }
  std::cout << lf_foot_height.size() << std::endl;
  std::cout << lf_joint_positions.size() << std::endl;
    std::cout << "finish the lf_leg planning" << std::endl;

    /*******planning the RH leg *******/

    x_start = -height_x;
    y_start = -height_y;
    z_start = height_z;

    Step_dis = 0.30;
    double rh_z_start;

    double rh_stairs_height;
    rh_stairs_height = stairs_height;
    double adjust_dis = 0.0;

    if(step_num == 0)
    {
        rh_z_start = z_start;
        height_1 = 0.05;
    }

    if(step_num == 1)
    {
        rh_z_start = z_start - rh_stairs_height;
        height_1 = 0.05;
    }

    if(step_num == 2)
    {
        rh_stairs_height = -stairs_height;
        rh_z_start = z_start - 0.05 -0.05;
        height_1 = 0.18;
        adjust_dis = 0.05;
    }
    if(step_num == 3)
    {
        rh_stairs_height = -stairs_height;
        rh_z_start = z_start - 0.05;
        height_1 = 0.18;
        adjust_dis = 0.05;
        x_start = -height_x + adjust_dis;
    }

    std::vector<std::vector<Time>> times_of_rh;
    std::vector<std::vector<valuetype>>rh_leg_position;
    std::vector<curves::PolynomialSplineQuinticVector3Curve> rh_leg_trajectory;
    times_of_rh.resize(8);
    rh_leg_position.resize(8);
    rh_leg_trajectory.resize(8);
    //step 1: RH leg changes from -0.45 to -0.35, when time is adjust_t/2;
    times_of_rh[0].push_back(0);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, rh_z_start));

    times_of_rh[0].push_back(adjust_t);
    rh_leg_position[0].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, rh_z_start));

    rh_leg_trajectory[0].fitCurve(times_of_rh[0], rh_leg_position[0]);

    //step 2: RH legs moves L/3 with 0.1m higher than the z_start;
    times_of_rh[1].push_back(0);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, rh_z_start));

    times_of_rh[1].push_back(period_t);
    rh_leg_position[1].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, rh_z_start));

    rh_leg_trajectory[1].fitCurve(times_of_rh[1], rh_leg_position[1]);

    //step 3:RH stand up before adjust_t/2 and keep this joint ;
    times_of_rh[2].push_back(0);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, rh_z_start));

    times_of_rh[2].push_back(adjust_t);
    rh_leg_position[2].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, rh_z_start));

    rh_leg_trajectory[2].fitCurve(times_of_rh[2], rh_leg_position[2]);

    //step 4:RH leg moves;
    j = 3;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, rh_z_start));

    times_of_rh[j].push_back(period_t * 4 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, rh_z_start + height_1));

    times_of_rh[j].push_back(period_t * 8 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - forward_d, y_start, rh_z_start + height_2 + height_1));

    times_of_rh[j].push_back(period_t * 12 / 16);
    rh_leg_position[j].push_back(valuetype(x_start - forward_d + Step_dis / 2 + adjust_dis, y_start, rh_z_start + height_1));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - forward_d + Step_dis / 2 + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
//    std::cout << "finish step 4" <<std::endl;

    //step 5: rh legs unchanged;
    j = 4;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
//    std::cout << "finish step 5" <<std::endl;

    //step 6: rh leg moves l/3 as before;
    j = 5;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);

    //step 7: rh leg unchanged;
    j = 6;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    times_of_rh[j].push_back(adjust_t);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
//    std::cout << "finish step 7" <<std::endl;

    //step 8:
    j = 7;
    times_of_rh[j].push_back(0);
    rh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    times_of_rh[j].push_back(period_t);
    rh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d + adjust_dis, y_start, rh_z_start - rh_stairs_height));

    rh_leg_trajectory[j].fitCurve(times_of_rh[j], rh_leg_position[j]);
//    std::cout << "finish step 8" <<std::endl;


//  std::vector<JointPositionsLimb> rh_joint_positions;
//  Pose rh_leg_pose;
//  valuetype rh_leg_joint;

  /***** evaluate the leg position ******/

  dt = 0;
  dt_ = 0;
  while(dt <= 4 * (period_t + adjust_t))
    {
      if(dt <= adjust_t)//0-adjust_t
        {
          rh_leg_trajectory[0].evaluate(rh_leg_joint, dt);
        }else if( dt > adjust_t && dt <= (period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
          dt_ = dt - adjust_t;
          rh_leg_trajectory[1].evaluate(rh_leg_joint, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
          dt_ = dt - period_t - adjust_t;
          rh_leg_trajectory[2].evaluate(rh_leg_joint, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//adjust_t + period_t * 2~ 2 * period_t + 2 * adjust_t
        {
          dt_ = dt - period_t - 2 * adjust_t;
          rh_leg_trajectory[3].evaluate(rh_leg_joint, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
          dt_ = dt - period_t * 2 - 2 * adjust_t;
          rh_leg_trajectory[4].evaluate(rh_leg_joint, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
          dt_ = dt - 2 * period_t - 3 * adjust_t;
          rh_leg_trajectory[5].evaluate(rh_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 3 * adjust_t;
          rh_leg_trajectory[6].evaluate(rh_leg_joint, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 4 * adjust_t;
          rh_leg_trajectory[7].evaluate(rh_leg_joint, dt_);
        }else{
          std::cout << "nothing" << std::endl;
        }
        dt = dt + 0.0025;
        rh_leg_pose.getPosition().x() = rh_leg_joint.x();
        rh_leg_pose.getPosition().y() = rh_leg_joint.y();
        rh_leg_pose.getPosition().z() = rh_leg_joint.z();

        QK.InverseKinematicsSolve(rh_leg_pose.getPosition(),LimbEnum::RH_LEG, joints, joints, "IN_LEFT");
        rh_joint_positions.push_back(joints);
        rh_foot_height.push_back(rh_leg_joint);
    }
     std::cout << "finish the RH_leg planning" << std::endl;

    /*******planning the RF leg *******/

     x_start = height_x;
     y_start = -height_y;
     z_start = height_z;

     double rf_z_start;
     double rf_stairs_height;

     if(step_num == 0)
     {
         rf_z_start = z_start;
         height_1 = 0.15;
     }

     if(step_num == 1)
     {
         rf_z_start = z_start + 0.05;
         height_1 = 0.15;
     }

     rf_stairs_height = stairs_height;
     if(step_num == 2)
     {
         rf_stairs_height = 0;
         rf_z_start = z_start + 0.05 + 0.05;
         height_1 = height_2;
     }
     if(step_num == 3)
     {
         rf_stairs_height = 0;
         height_1 = height_2;
         rf_z_start = z_start + 0.05 + 0.05;
     }


  std::vector<std::vector<Time>> times_of_rf;
  times_of_rf.resize(8);
  std::vector<std::vector<valuetype>>rf_leg_position;
  rf_leg_position.resize(8);
  std::vector<curves::PolynomialSplineQuinticVector3Curve> rf_leg_trajectory;
  rf_leg_trajectory.resize(8);

  //step 1: RF changes from -0.35 to -0.45 with respect the first adjust/2;
  j = 0;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, rf_z_start));

  times_of_rf[j].push_back(adjust_t);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, rf_z_start));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 2: rf leg leg unchanges;
  j = 1;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, rf_z_start));

  times_of_rf[j].push_back(period_t);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, rf_z_start));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 3: RF unchanges;
  j = 2;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, rf_z_start));

  times_of_rf[j].push_back(adjust_t);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, rf_z_start));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 4: RF unchanges;
  j = 3;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, rf_z_start));

  times_of_rf[j].push_back(period_t);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, rf_z_start));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 5: RF unchanges;
  j = 4;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, rf_z_start));

  times_of_rf[j].push_back(adjust_t);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, rf_z_start));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 6: RF moves swing;
  j = 5;

  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, rf_z_start));

  times_of_rf[j].push_back(period_t * 4 / 16);
  rf_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, rf_z_start + height_1));

  times_of_rf[j].push_back(period_t * 8 / 16);
  rf_leg_position[j].push_back(valuetype(x_start + forward_d, y_start, rf_z_start + height_2 + height_1));

  times_of_rf[j].push_back(period_t * 12 / 16);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, rf_z_start + height_1));

  times_of_rf[j].push_back(period_t);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, rf_z_start + rf_stairs_height));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 7: RF unchanges;
  j = 6;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, rf_z_start + rf_stairs_height));

  times_of_rf[j].push_back(adjust_t);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, rf_z_start + rf_stairs_height));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  //step 8: RF unchanges;
  j = 7;
  times_of_rf[j].push_back(0);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, rf_z_start + rf_stairs_height));

  times_of_rf[j].push_back(period_t);
  rf_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, rf_z_start + rf_stairs_height));

  rf_leg_trajectory[j].fitCurve(times_of_rf[j], rf_leg_position[j]);

  /***** evaluate the leg position ******/

//  std::vector<JointPositionsLimb> rf_joint_positions;
//  Pose rf_leg_pose;
//  valuetype rf_leg_joint;

  /***** evaluate the leg position ******/

  dt = 0;
  dt_ = 0;
  while(dt <= 4 * (period_t + adjust_t))
    {
      if(dt <= adjust_t)//0-adjust_t
        {
          rf_leg_trajectory[0].evaluate(rf_leg_joint, dt);
        }else if( dt > adjust_t && dt <= (period_t + adjust_t)) //adjust_t~adjust_t + period_t
        {
          dt_ = dt - adjust_t;
          rf_leg_trajectory[1].evaluate(rf_leg_joint, dt_);
        }else if(dt > (period_t + adjust_t) && dt <=(period_t + 2 * adjust_t))//adjust_t + period_t~ period_t + 2 * adjust_t
        {
          dt_ = dt - period_t - adjust_t;
          rf_leg_trajectory[2].evaluate(rf_leg_joint, dt_);
        }else if(dt > (period_t + 2 * adjust_t) && dt <=(2 * period_t + 2 * adjust_t))//adjust_t + period_t * 2~ 2 * period_t + 2 * adjust_t
        {
          dt_ = dt - period_t - 2 * adjust_t;
          rf_leg_trajectory[3].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + 2 * adjust_t) && dt <=(2 * period_t + 3 * adjust_t))
        {
          dt_ = dt - period_t * 2 - 2 * adjust_t;
          rf_leg_trajectory[4].evaluate(rf_leg_joint, dt_);
        }else if(dt > (2 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 3 * adjust_t))
        {
          dt_ = dt - 2 * period_t - 3 * adjust_t;
          rf_leg_trajectory[5].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 3 * adjust_t) && dt <=(3 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 3 * adjust_t;
          rf_leg_trajectory[6].evaluate(rf_leg_joint, dt_);
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 4 * adjust_t;
          rf_leg_trajectory[7].evaluate(rf_leg_joint, dt_);
        }else{
          std::cout << "nothing" << std::endl;
        }
        dt = dt + 0.0025;
        rf_leg_pose.getPosition().x() = rf_leg_joint.x();
        rf_leg_pose.getPosition().y() = rf_leg_joint.y();
        rf_leg_pose.getPosition().z() = rf_leg_joint.z();

        QK.InverseKinematicsSolve(rf_leg_pose.getPosition(),LimbEnum::RF_LEG, joints, joints, "OUT_LEFT");
        rf_joint_positions.push_back(joints);
        rf_foot_height.push_back(rf_leg_joint);
    }

    std::cout << "finish the RF_leg planning" << std::endl;

    /*******planning the LH leg *******/

    x_start = -height_x;
    y_start = height_y;
    z_start = height_z;

    double lh_z_start;

    double lh_stairs_height;
    lh_stairs_height = stairs_height;

    if(step_num == 0)
    {
        lh_z_start = z_start;
        height_1 = 0.05;
    }

    if(step_num == 1)
    {
        lh_z_start = z_start - lh_stairs_height;
        height_1 = 0.05;
    }

    if(step_num == 2)
    {
        lh_stairs_height = - stairs_height;
        lh_z_start = z_start - 0.05 -0.05;
        height_1 = 0.18;
    }
    if(step_num == 3)
    {
        lh_stairs_height = -stairs_height;
        lh_z_start = z_start - 0.05;
        height_1 = 0.18;
    }

  std::vector<std::vector<Time>> times_of_lh;
  std::vector<std::vector<valuetype>> lh_leg_position;
  std::vector<curves::PolynomialSplineQuinticVector3Curve> lh_leg_trajectory;

  times_of_lh.resize(8);
  lh_leg_position.resize(8);
  lh_leg_trajectory.resize(8);

  //step 1: lh leg unchanged;
  j = 0;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, lh_z_start - down_height));

  times_of_lh[j].push_back(adjust_t);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 2: lh leg moves l/3 with height -0.45m;
  j = 1;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 + forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(period_t);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 3: LH leg unchanged;
  j = 2;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 + forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(adjust_t);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 4: lh leg moves l/3 with height -0.45m;
  j = 3;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 6 - forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(period_t);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 5: lh leg knee down;
  j = 4;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 - forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(adjust_t);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 6: lh moves l/3 with -0.35;
  j = 5;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 6 + forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(period_t);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 7: lh stand up in front adjust_t/2
  j = 6;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 + forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(adjust_t);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, lh_z_start));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  //step 8: lh moves swing
  j = 7;
  times_of_lh[j].push_back(0);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, lh_z_start));

  times_of_lh[j].push_back(period_t * 4 / 16);
  lh_leg_position[j].push_back(valuetype(x_start - Step_dis / 2 - forward_d, y_start, lh_z_start + height_1));

  times_of_lh[j].push_back(period_t * 8 / 16);
  lh_leg_position[j].push_back(valuetype(x_start - forward_d, y_start, lh_z_start + height_2 + height_1));

  times_of_lh[j].push_back(period_t * 12 / 16);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, lh_z_start + height_1));

  times_of_lh[j].push_back(period_t);
  lh_leg_position[j].push_back(valuetype(x_start + Step_dis / 2 - forward_d, y_start, lh_z_start - lh_stairs_height));

  lh_leg_trajectory[j].fitCurve(times_of_lh[j], lh_leg_position[j]);

  /***** evaluate the leg position ******/

//  std::vector<JointPositionsLimb> lh_joint_positions;
//  Pose lh_leg_pose;
//  valuetype lh_leg_joint;

  /***** evaluate the leg position ******/

  dt = 0;
  dt_ = 0;
  while(dt <= 4 * (period_t + adjust_t))
    {
      if(dt <= adjust_t)//0-adjust_t
        {
          lh_leg_trajectory[0].evaluate(lh_leg_joint, dt);
        }else if ( dt > adjust_t && dt <= (period_t + adjust_t)) //adjust_t~adjust_t + period_t
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
        }else if(dt > (3 * period_t + 4 * adjust_t) && dt <=(4 * period_t + 4 * adjust_t))
        {
          dt_ = dt - 3 * period_t - 4 * adjust_t;
          lh_leg_trajectory[7].evaluate(lh_leg_joint, dt_);
        }else{
          std::cout << "nothing" << std::endl;
        }
        dt = dt + 0.0025;
        lh_leg_pose.getPosition().x() = lh_leg_joint.x();
        lh_leg_pose.getPosition().y() = lh_leg_joint.y();
        lh_leg_pose.getPosition().z() = lh_leg_joint.z();

        QK.InverseKinematicsSolve(lh_leg_pose.getPosition(),LimbEnum::LH_LEG, joints, joints, "OUT_LEFT");
        lh_joint_positions.push_back(joints);
        lh_foot_height.push_back(lh_leg_joint);
    }

    std::cout << "finish the LH_leg planning" << std::endl;

    /******Now we store the joint angles to the txt file *******/
    std::cout <<"LF LEG data size is" << lf_joint_positions.size() <<std::endl;
    std::cout <<"RH LEG data size is" << rh_joint_positions.size() <<std::endl;
    std::cout <<"RF LEG data size is" << rf_joint_positions.size() <<std::endl;
    std::cout <<"LH LEG data size is" << lh_joint_positions.size() <<std::endl;

//    quadruped_model::JointPositions joint_positions;
//    std::vector<quadruped_model::JointPositions> joint_positions_total;
//    for(unsigned int i = 0; i < lf_joint_positions.size(); i++)
//      {
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

//      }
//    std::cout << "size is " << joint_positions_total.size() << std::endl;
    }//step_num
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
  std::string file_name = "calculated.txt";
  SaveAsFile(file_name, joint_positions_total);
  file_name = "calculated_read.txt";
  SaveAsFile_with_footheight(file_name, joint_positions_total, lf_foot_height, rf_foot_height, rh_foot_height, lh_foot_height);

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
