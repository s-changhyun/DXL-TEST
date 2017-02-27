/**
 * @file /include/dxl_test_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dxl_test_gui_QNODE_HPP_
#define dxl_test_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <ros/callback_queue.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>

#include <std_msgs/Float64.h>

#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dxl_test_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

  /*********************
  ** Logging
  **********************/
  enum LogLevel
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg, std::string sender);

  void sendSetModeMsg();
  void sendGoalTorqueMsg(std_msgs::Float64 msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;
  ros::Publisher chatter_publisher;
  QStringListModel logging_model;

  ros::Publisher set_mode_msg_pub_;
  ros::Publisher set_goal_torque_msg_pub_;
};

}  // namespace dxl_test_gui

#endif /* dxl_test_gui_QNODE_HPP_ */
