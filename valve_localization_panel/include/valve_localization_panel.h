/*****************************************************************************
 * Copyright (c) 2013, Worcester Polytechnic Institute                       *
 * All rights reserved.                                                      *
 *                                                                           *
 * Author: Nicholas Alunni <nick.alunni@gmail.com>                           *
 * Date: June 20, 2013                                                       *
 *                                                                           *
 * Human Interaction in Virtual Enviroments (HIVE) Lab                       *
 * Worcester Polytechnic Institute, Worcester Massachusetts                  *
 * Director: Robert W. Lindeman     <gogo@wpi.edu>                           *
 * Website: http://web.cs.wpi.edu/~hive/                                     *
 *                                                                           *
 *                                                                           *
 * Contributors:                                                             *                                   *
 *                                                                           *
 * Thanks:                                                                   *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 * This file is provided under the following "BSD-style" License:            *
 *   Redistribution and use in source and binary forms, with or without      *
 *   modification, are permitted provided that the following conditions are  *   
 *   met:                                                                    *
 *   * Redistributions of source code must retain the above copyright        *
 *     notice, this list of conditions and the following disclaimer.         *
 *   * Redistributions in binary form must reproduce the above               *
 *     copyright notice, this list of conditions and the following           *
 *     disclaimer in the documentation and/or other materials provided       *
 *     with the distribution.                                                *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND                  *
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,             *
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF                *
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                *
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR                   *
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,            *
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF        *
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED         *
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT             *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN       *
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         *
 *   POSSIBILITY OF SUCH DAMAGE.                                             *
 ****************************************************************************/

#ifndef HUBO_VALVE_LOCALIZATION_PANEL_H_
#define HUBO_VALVE_LOCALIZATION_PANEL_H_

//DRC Includes

//Ros Specific Includes
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Empty.h>
#include <valve_localization_panel_msgs/PanelUpdate.h>

//C++ Includes
#include <vector>
#include <stdio.h>

//QT Gui Includes
#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QSlider>
#include <QCheckBox>
#include <QPixmap>
#include <QSpinBox>
#include <QGridLayout>
#include <QButtonGroup>

//Namespace for the project/plugin
namespace Valve_Localization
{

//We will call the class something obvious ROBOT/TYPEOFWIDGET
class HuboValveLocalizationWidget;

//This class will handle refreshing for the widget. As far as I can tell it
//Is responsible for waking up at a given time and updating the interface
//Even if nothing has been touched or interacted with by the user.
class HuboSensorControlRefreshManager : public QThread
{
Q_OBJECT
public:
    HuboValveLocalizationWidget* parentWidget;
    bool alive;
    int waitTime;

protected:
    virtual void run();

protected slots:
    void getWaitTime(int t);

signals:
    void signalRefresh();

};

//=============================================================================

// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
class HuboValveLocalizationWidget: public QTabWidget
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  HuboValveLocalizationWidget( QWidget* parent = 0 );
  ~HuboValveLocalizationWidget();

  // Update timer
  HuboSensorControlRefreshManager* refreshManager;
  int getRefreshTime();

  //This will control how a whole bunch of the boxes look in the RVIZ panel
  QString groupStyleSheet;

protected: //Variables go here, ints and such

    

//These slots will be connected to the signals generated from the GUI in order
//To perform the various actions that are needed
protected Q_SLOTS:

  void handlePlanGetready(void);
  void handlePlanGrasp(void);
  void handlePlanTurning(void);
  void handlePlanUngrasp(void);
  void handlePlanFinish(void);

  void handlePreview(void);
  void handleExecute(void);

  void handlePreviewGetready(void);
  void handlePreviewGrasp(void);
  void handlePreviewTurning(void);
  void handlePreviewUngrasp(void);
  void handlePreviewFinish(void);

  void handleExecuteGetready(void);
  void handleExecuteGrasp(void);
  void handleExecuteTurning(void);
  void handleExecuteUngrasp(void);
  void handleExecuteFinish(void);


  void handleClockwiseTurnDirection(bool isChecked);
  void handleCounterClockwiseTurnDirection(bool isChecked);
  void handleRound(bool isChecked);
  void handleLeftLever(bool isChecked);
  void handleRightLever(bool isChecked);
  void handlePlannerBoth(bool isChecked);
  void handlePlannerLeft(bool isChecked);
  void handlePlannerRight(bool isChecked);
  void handleUserBoth(bool isChecked);
  void handleUserLeft(bool isChecked);
  void handleUserRight(bool isChecked);
  void handleGrippers(bool isChecked);
  void handlePegs(bool isChecked);
  void handleTurnAmount(int value);
  void handleValveRadius(int value);

  void handleLeftCompliant(bool isChecked);
  void handleRightCompliant(bool isChecked);

  void handlePlanInBox(bool isChecked);

  void handleGrabMiddle(bool isChecked);

  void handleJointLimitDisrupt(void);
  void handleResetPosition(void);

  void handleFixedTurn(bool isChecked);

  void handleIkSeed(bool isChecked);

private:

    //Ros Node
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber state_rec_;
    ros::ServiceClient joint_client_;

    valve_localization_panel_msgs::PanelUpdate state_msg_;

    void publishState(void);
    void updateCB(const valve_localization_panel_msgs::PanelUpdate::ConstPtr &msg);

    //===== Planner Control Tab =====
    void initializeMainTab();
    QWidget* mainTab;

    QLabel* robotStateCurrent_;
    QLabel* informationCurrent_;

    QPushButton* plan_getready_button_;
    QPushButton* plan_grasp_button_;
    QPushButton* plan_turning_button_;
    QPushButton* plan_ungrasp_button_;
    QPushButton* plan_finish_button_;
    QPushButton* preview_button_;
    QPushButton* execute_button_;

    QPushButton* preview_getready_button_;
    QPushButton* preview_grasp_button_;
    QPushButton* preview_turning_button_;
    QPushButton* preview_ungrasp_button_;
    QPushButton* preview_finish_button_;

    QPushButton* execute_getready_button_;
    QPushButton* execute_grasp_button_;
    QPushButton* execute_turning_button_;
    QPushButton* execute_ungrasp_button_;
    QPushButton* execute_finish_button_;



    //===== Planner Settings =====
    void initializeSecondTab();
    QWidget* secondTab;

    QSpinBox* turnAmountSpinBox_;
    QSpinBox* valveRadiusSpinBox_;

    QRadioButton* clockwiseRadioButton_;
    QRadioButton* counterClockwiseRadioButton_;
    QRadioButton* roundRadioButton_;
    QRadioButton* leftLeverRadioButton_;
    QRadioButton* rightLeverRadioButton_;
    QRadioButton* plannerBothRadioButton_;
    QRadioButton* plannerLeftRadioButton_;
    QRadioButton* plannerRightRadioButton_;
    QRadioButton* userBothRadioButton_;
    QRadioButton* userLeftRadioButton_;
    QRadioButton* userRightRadioButton_;
    QRadioButton* grippersRadioButton_;
    QRadioButton* pegsRadioButton_;

    QCheckBox* fixedTurnBox_;
    QCheckBox* ikseedTurnBox_;

    QCheckBox* leftArmCompliantBox_;
    QCheckBox* rightArmCompliantBox_;
    QCheckBox* planInBoxBox_;
    QCheckBox* grabMiddleBox_;

    QPushButton* jimsButton_;
    QPushButton* resetPositionButton_;

};


class HuboValveLocalizationPanel : public rviz::Panel
{
Q_OBJECT
public:
    HuboValveLocalizationPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:

    HuboValveLocalizationWidget* content;

};

} // end namespace Valve_Localization


#endif //HUBO_VALVE_LOCALIZATION_PANEL_H_
