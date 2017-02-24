/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * test_module.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: sch
 */

#include <stdio.h>
#include "dxl_test_module/test_module.h"

using namespace dxl_test;

TestModule::TestModule()
  : control_cycle_sec_(0.004),
    is_moving_(false),
    cnt_(0)
{
  enable_       = false;
  module_name_  = "test_module";
  control_mode_ = robotis_framework::TorqueControl;

  result_["dxl_1"]   = new robotis_framework::DynamixelState();

  joint_name_to_id_["dxl_1"]   = 1;

  /* joint information */
  present_joint_position_   = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
  present_joint_velocity_   = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
  present_joint_effort_     = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);

  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
  goal_joint_velocity_      = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
  goal_joint_acceleration_  = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);

  goal_joint_effort_        = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
  joint_position_integral_  = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);

  /* torque offset (friction dynamic */
//  torque_offset_      = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
//  torque_offset_sign_ = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
//  torque_offset_sign_.fill(0.0);

  /* joint gain */
//  ff_torque_gain_    = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
//  fb_p_torque_gain_  = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);

//  ff_joint_vel_gain_    = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
//  fb_p_joint_pos_gain_  = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
//  fb_i_joint_pos_gain_  = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);
//  fb_d_joint_pos_gain_  = Eigen::VectorXd::Zero(MAX_DXL_NUM + 1);

  /* target joint values */
//  via_num_ = 1;
//  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);

//  target_joint_position_          = Eigen::VectorXd::Zero(MAX_JOINT_ID + 1);
//  target_joint_via_position_      = Eigen::MatrixXd::Zero(via_num_,MAX_JOINT_ID + 1);
//  target_joint_via_velocity_      = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
//  target_joint_via_acceleration_  = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
}

TestModule::~TestModule()
{
  queue_thread_.join();
}

void TestModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  ROS_INFO("control cycle : %f", control_cycle_sec_);
  queue_thread_ = boost::thread(boost::bind(&TestModule::queueThread, this));
}

void TestModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  /* subscribe topics */

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

//void TestModule::setStartTrajectory()
//{
//  ROS_INFO("[start] send trajectory");
//  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
//}

//void TestModule::setEndTrajectory()
//{
//  if (is_moving_ == true)
//  {
//    if (cnt_ >= all_time_steps_)
//    {
//      ROS_INFO("[end] send trajectory");
//      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

//      is_moving_ = false;
//      wb_ik_solving_ = false;

//      wb_pelvis_planning_ = false;
//      wb_l_arm_planning_ = false;
//      wb_r_arm_planning_ = false;
//      cnt_ = 0;

//      if (display_joint_angle_ == true)
//      {
//        for (int id=1; id<=12; id++)
//          ROS_INFO("%s : %f",
//                   robotis_->link_data_[id]->name_.c_str(),
//                   robotis_->link_data_[id]->joint_angle_ * RADIAN2DEGREE);
//      }

//      ROS_INFO("-----");

//      for (int id=1; id<=MAX_JOINT_ID; id++)
//        ROS_INFO("torque[%d] : %f", id, goal_joint_effort_(id));

//      PRINT_MAT(robotis_->link_data_[ID_PELVIS_ROLL]->joint_torque_);
//      PRINT_MAT(robotis_->link_data_[ID_PELVIS_PITCH]->joint_torque_);
//      PRINT_MAT(robotis_->link_data_[ID_PELVIS_YAW]->joint_torque_);

//      PRINT_MAT(robotis_->link_data_[ID_L_LEG_END]->position_);
//      PRINT_MAT(robotis_->link_data_[ID_R_LEG_END]->position_);

//      Eigen::Vector3d mass_center = robotis_->calcMassCenter(0);
//      center_of_mass_ = robotis_->calcCenterOfMass(mass_center);
//      PRINT_MAT(center_of_mass_);

//      ROS_INFO("-----");

//      torque_offset_sign_.fill(0.0);

//    }
//  }
//}

void TestModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, double> sensors)
{
  ros::Time begin = ros::Time::now();
  ros::Duration time_duration;

  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Motion Module");
    return;
  }

  process_mutex_.lock();
  /*----- Get Joint State -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    present_joint_position_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_velocity_;
    present_joint_effort_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_torque_;
  }


//  if (is_moving_ == true)
//  {
//    if (cnt_ == 0)
//    {
//      setStartTrajectory();
//    }

//    for (int id=1; id<=MAX_JOINT_ID; id++)
//    {
//      goal_joint_position_(id)      = goal_joint_position_tra_(cnt_, id);
//      goal_joint_velocity_(id)      = goal_joint_velocity_tra_(cnt_, id);
//      goal_joint_acceleration_(id)  = goal_joint_acceleration_tra_(cnt_, id);
//    }
//    cnt_++;
//  }

//  setEndTrajectory();

  // Joint Controller
  for (int id=1; id<=MAX_DXL_NUM; id++)
    goal_joint_effort_(id) = 0.0;


  /*----- Set Goal Joint State -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_torque_ = goal_joint_effort_(joint_name_to_id_[joint_name]);
  }
  process_mutex_.unlock();

  time_duration = ros::Time::now() - begin;
  double loop_time = time_duration.toSec(); //time_duration.sec + time_duration.nsec * 1e-9;

  if (loop_time > 0.004)
    ROS_WARN("Calculation Time : %f", loop_time );
}


void TestModule::stop()
{
  return;
}

bool TestModule::isRunning()
{
  return is_moving_;
}

void TestModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "TestModule";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
