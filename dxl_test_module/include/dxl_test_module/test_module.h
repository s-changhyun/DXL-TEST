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
 * test_module.h
 *
 *  Created on: Feb 24, 2017
 *      Author: sch
 */

#ifndef DXL_TEST_MODULE_H_
#define DXL_TEST_MODULE_H_

#include <map>
#include <fstream>
#include <vector>
#include <string>
#include <iterator>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace dxl_test
{

#define MAX_DXL_NUM   (1)
#define TASK_DEMENSION  (6)

class TestModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<TestModule>
{
private:
  double control_cycle_sec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;

  ros::Publisher goal_state_pub_;

  boost::thread* tra_gene_tread_;

  std::map<std::string, int> joint_name_to_id_;

  /* joint information */
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd present_joint_effort_;

  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd goal_joint_velocity_;
  Eigen::VectorXd goal_joint_acceleration_;

  Eigen::VectorXd goal_joint_effort_;

  double goal_torque_;

  /* movement */
  bool is_moving_;
  double mov_time_;
  int all_time_steps_;
  int cnt_;

  std::vector<double> present_torque_vector_;
  std::vector<double> goal_torque_vector_;
  std::vector<double> present_velocity_vector_;

  boost::mutex process_mutex_;
  void queueThread();

  void setGoalTorqueMsgCallback(const std_msgs::Float64::ConstPtr& msg);

  void outputSave();

public:
  TestModule();
  virtual ~TestModule();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  void publishStatusMsg(unsigned int type, std::string msg);
};

}

#endif /* ROBOTIS_OP2_MOTION_MODULE_H_ */
