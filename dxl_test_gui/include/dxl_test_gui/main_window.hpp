/**
 * @file /include/dxl_test_gui/main_window.hpp
 *
 * @brief Qt based gui for dxl_test_gui.
 *
 * @date November 2010
 **/
#ifndef dxl_test_gui_MAIN_WINDOW_H
#define dxl_test_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dxl_test_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();

  void on_set_value_button_clicked(bool check);
  void on_set_mode_button_clicked(bool check);

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;
};

}  // namespace dxl_test_gui

#endif // dxl_test_gui_MAIN_WINDOW_H
