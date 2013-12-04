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
 * Contributors:                                                             *                            *
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

#include "valve_localization_panel.h"

//Namespace for the project/plugin
namespace Valve_Localization {

//Constructor for the Actual RVIZ Panel that we are creating
HuboValveLocalizationPanel::HuboValveLocalizationPanel(QWidget *parent) : rviz::Panel(parent) {

    //Create the widget itself
    content = new HuboValveLocalizationWidget;

    //Create a new boxlayout for the panel
    QHBoxLayout* panelLayout = new QHBoxLayout;

    //Add the widget to the panel
    panelLayout->addWidget(content);

    //Set it up, or do something, I don't really know yet ...
    setLayout(panelLayout);
}

//Deconstructor for the actual widget
HuboValveLocalizationWidget::~HuboValveLocalizationWidget()
{

    //Kill the refresh manager
    refreshManager->alive = false;
    refreshManager->quit();
    refreshManager->wait();
}


//Constructor for the actual Hubo Widget
HuboValveLocalizationWidget::HuboValveLocalizationWidget(QWidget *parent) : QTabWidget(parent) {

    //Setup the stylesheet that will be used throughout the code
    //This is mostly used for the boxes that contain other elements.
    //It creates a pleasant box with rounded corners in order to contain
    //The other elements of the GUI.
    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";

    //Create the publisher and subscriber for the panel
    state_pub_ = nh_.advertise<valve_localization_panel_msgs::PanelUpdate>("valve_localization_panel/state", 1);
    state_rec_ = nh_.subscribe("valve_localization_panel/update", 1, &HuboValveLocalizationWidget::updateCB, this);
    joint_client_ = nh_.serviceClient<std_srvs::Empty>("/valve_planner/drchubo_planner/joint_limit_disrupt");

    //Setup the state message
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::NO_COMMAND;
    state_msg_.Direction = valve_localization_panel_msgs::PanelUpdate::CLOCKWISE;
    state_msg_.EndEffector = valve_localization_panel_msgs::PanelUpdate::GRIPPER;
    state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_BOTH;
    state_msg_.TurnAmount = 30;
    state_msg_.ValveRadius = 23;
    state_msg_.ValveType = valve_localization_panel_msgs::PanelUpdate::ROUND;
    state_msg_.FixedTurn = true;
    state_msg_.PlanInBox = true;
    state_msg_.IkSeed = false;

    //The First tab controls data about the robots' state
    initializeMainTab();
    addTab(mainTab, "Planner Control");

    //The Second tab controls data about the robots' vision
    initializeSecondTab();
    addTab(secondTab, "Planner Settings");

    refreshManager = new HuboSensorControlRefreshManager;
    refreshManager->parentWidget = this;
//    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
    refreshManager->start();
}

void HuboValveLocalizationWidget::publishState(void){

    ROS_INFO("--------------------");
    ROS_INFO("Sending Panel Update");

    if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::NO_COMMAND) ROS_INFO("Command = NONE");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::PLAN_GETREADY) ROS_INFO("Command = PLAN_GETREADY");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::PLAN_GRASP) ROS_INFO("Command = PLAN_GRASP");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::PLAN_TURNING) ROS_INFO("Command = PLAN_TURNING");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::PLAN_UNGRASP) ROS_INFO("Command = PLAN_UNGRASP");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::PLAN_FINISH) ROS_INFO("Command = PLAN_FINISH");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::PREVIEW) ROS_INFO("Command = PREVIEW");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::EXECUTE) ROS_INFO("Command = EXECUTE");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::TELEOP_START) ROS_INFO("Command = TELEOP_START");
    else if (state_msg_.Command == valve_localization_panel_msgs::PanelUpdate::TELEOP_STOP) ROS_INFO("Command = TELEOP_STOP");
    else ROS_WARN("COMMAND = ???? UNKNOWN ????");

    ROS_INFO("Radius = %5.2f", state_msg_.ValveRadius);
    ROS_INFO("Turn Amount = %5.2f", state_msg_.TurnAmount);
    if (state_msg_.FixedTurn) ROS_INFO("Fixed Turn = TRUE");
    else ROS_INFO("Fixed Turn = FALSE");

    if (state_msg_.IkSeed) ROS_INFO("Ik Seed = TRUE");
    else ROS_INFO("Ik Seed = FALSE");

    if (state_msg_.ValveType == valve_localization_panel_msgs::PanelUpdate::ROUND) ROS_INFO("Valve Type = ROUND");
    else if (state_msg_.ValveType == valve_localization_panel_msgs::PanelUpdate::LEFT_LEVER) ROS_INFO("Valve Type = LEFT_LEVER");
    else if (state_msg_.ValveType == valve_localization_panel_msgs::PanelUpdate::RIGHT_LEVER) ROS_INFO("Valve Type = RIGHT_LEVER");
    else ROS_WARN("Valve Type = ???? UNKNOWN ????");

    if (state_msg_.EndEffector == valve_localization_panel_msgs::PanelUpdate::GRIPPER) ROS_INFO("End Effector = GRIPPER");
    else if (state_msg_.EndEffector == valve_localization_panel_msgs::PanelUpdate::PEG) ROS_INFO("End Effector = PEG");
    else ROS_WARN("End Effector = ???? UNKNOWN ????");

    if (state_msg_.Hands == valve_localization_panel_msgs::PanelUpdate::PLANNER_BOTH) ROS_INFO("Hands = PLANNER_BOTH");
    else if (state_msg_.Hands == valve_localization_panel_msgs::PanelUpdate::PLANNER_LEFT) ROS_INFO("Hands = PLANNER_LEFT");
    else if (state_msg_.Hands == valve_localization_panel_msgs::PanelUpdate::PLANNER_RIGHT) ROS_INFO("Hands = PLANNER_RIGHT");
    else if (state_msg_.Hands == valve_localization_panel_msgs::PanelUpdate::USER_BOTH) ROS_INFO("Hands = USER_BOTH");
    else if (state_msg_.Hands == valve_localization_panel_msgs::PanelUpdate::USER_LEFT) ROS_INFO("Hands = USER_LEFT");
    else if (state_msg_.Hands == valve_localization_panel_msgs::PanelUpdate::USER_RIGHT) ROS_INFO("Hands = USER_RIGHT");
    else ROS_WARN("Hands = ???? UNKNOWN ????");

    if (state_msg_.Direction == valve_localization_panel_msgs::PanelUpdate::CLOCKWISE) ROS_INFO("Direction = CLOCKWISE");
    else if (state_msg_.Direction == valve_localization_panel_msgs::PanelUpdate::COUNTER_CLOCKWISE) ROS_INFO("Direction = COUNTER_CLOCKWISE");
    else ROS_WARN("Direction = ???? UNKNOWN ????");

    if (state_msg_.PlanInBox) ROS_INFO("Plan In Box = TRUE");
    else ROS_INFO("Plan In Box = FALSE");

    if (state_msg_.GrabMiddle) ROS_INFO("Grab Middle = TRUE");
    else ROS_INFO("Grab Middle = FALSE");

    if (state_msg_.LeftCompliance) ROS_INFO("Left Compliance = TRUE");
    else ROS_INFO("Left Compliance = FALSE");

    if (state_msg_.RightCompliance) ROS_INFO("Right Compliance = TRUE");
    else ROS_INFO("Right Compliance = FALSE");

    state_pub_.publish(state_msg_);

}

void HuboValveLocalizationWidget::updateCB(const valve_localization_panel_msgs::PanelUpdate::ConstPtr &msg){

    if (msg->State != "NONE") {
        robotStateCurrent_->setText(msg->State.c_str()); }


    if (msg->Info.length() <= 150) {
        informationCurrent_->setText(msg->Info.c_str());
    } else {
        std::string shortInfo (msg->Info.begin(), msg->Info.begin()+150);
        informationCurrent_->setText(shortInfo.c_str());
    }

}

//Run the fresh manager which runs at a given frequency.
//For right now it appears to be 1Hz.
void HuboSensorControlRefreshManager::run()
{
    alive = true;
    waitTime = 250;
    while(alive)
    {
        emit signalRefresh();
        this->msleep(waitTime);
    }
    emit finished();
}

//Get the wait time for the refresh manager
void HuboSensorControlRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}

void HuboValveLocalizationPanel::save(rviz::Config config) const
{
  rviz::Panel::save( config );
}

void HuboValveLocalizationPanel::load(const rviz::Config &config)
{
  rviz::Panel::load( config );
}


} // End hubo_init_space

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( Valve_Localization::HuboValveLocalizationPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( Valve_Localization::HuboValveLocalizationWidget, QTabWidget )
PLUGINLIB_EXPORT_CLASS( Valve_Localization::HuboSensorControlRefreshManager, QThread )
