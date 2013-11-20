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
 * Contributors:                                                             *
 * Michael X. Grey <mxgrey@gatech.edu>                                       *
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

void HuboValveLocalizationWidget::handleClockwiseTurnDirection(bool isChecked) {



    if (isChecked){
        state_msg_.Direction = valve_localization_panel_msgs::PanelUpdate::CLOCKWISE;
        publishState(); }
}

void HuboValveLocalizationWidget::handleCounterClockwiseTurnDirection(bool isChecked) {

    if (isChecked){
        state_msg_.Direction = valve_localization_panel_msgs::PanelUpdate::COUNTER_CLOCKWISE;
        publishState(); }
}

void HuboValveLocalizationWidget::handleRound(bool isChecked) {
    if (isChecked){

        clockwiseRadioButton_->setEnabled(true);
        counterClockwiseRadioButton_->setEnabled(true);
        turnAmountSpinBox_->setEnabled(true);
        valveRadiusSpinBox_->setEnabled(true);

        if (grabMiddleBox_->isChecked()) {
            plannerBothRadioButton_->setEnabled(false);
            userBothRadioButton_->setEnabled(false);
            userLeftRadioButton_->setEnabled(false);
            userRightRadioButton_->setEnabled(false);
            plannerRightRadioButton_->setChecked(true);
            counterClockwiseRadioButton_->setChecked(true);
        } else {
            plannerBothRadioButton_->setEnabled(true);
            userBothRadioButton_->setEnabled(true);
            userLeftRadioButton_->setEnabled(true);
            userRightRadioButton_->setEnabled(true);
            plannerBothRadioButton_->setChecked(true);
            counterClockwiseRadioButton_->setChecked(true);
        }

        state_msg_.ValveType = valve_localization_panel_msgs::PanelUpdate::ROUND;
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_BOTH;
        state_msg_.Direction = valve_localization_panel_msgs::PanelUpdate::COUNTER_CLOCKWISE;


        publishState(); }
}

void HuboValveLocalizationWidget::handleLeftLever(bool isChecked) {
    if (isChecked){

        clockwiseRadioButton_->setEnabled(false);
        counterClockwiseRadioButton_->setEnabled(false);
        plannerBothRadioButton_->setEnabled(false);
        userBothRadioButton_->setEnabled(false);
        userLeftRadioButton_->setEnabled(false);
        userRightRadioButton_->setEnabled(false);
        turnAmountSpinBox_->setEnabled(false);
        valveRadiusSpinBox_->setEnabled(false);

        plannerRightRadioButton_->setChecked(true);

        state_msg_.ValveType = valve_localization_panel_msgs::PanelUpdate::LEFT_LEVER;
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_RIGHT;

        publishState(); }
}

void HuboValveLocalizationWidget::handleRightLever(bool isChecked) {
    if (isChecked){

        clockwiseRadioButton_->setEnabled(false);
        counterClockwiseRadioButton_->setEnabled(false);
        plannerBothRadioButton_->setEnabled(false);
        userBothRadioButton_->setEnabled(false);
        userLeftRadioButton_->setEnabled(false);
        userRightRadioButton_->setEnabled(false);
        turnAmountSpinBox_->setEnabled(false);
        valveRadiusSpinBox_->setEnabled(false);

        plannerLeftRadioButton_->setChecked(true);

        state_msg_.ValveType = valve_localization_panel_msgs::PanelUpdate::RIGHT_LEVER;
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_LEFT;

        publishState(); }
}

void HuboValveLocalizationWidget::handlePlannerBoth(bool isChecked) {
    if (isChecked){
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_BOTH;
        publishState(); }
}

void HuboValveLocalizationWidget::handlePlannerLeft(bool isChecked) {
    if (isChecked){
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_LEFT;
        publishState(); }
}

void HuboValveLocalizationWidget::handlePlannerRight(bool isChecked) {
    if (isChecked){
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::PLANNER_RIGHT;
        publishState(); }
}

void HuboValveLocalizationWidget::handleUserBoth(bool isChecked) {
    if (isChecked){
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::USER_BOTH;
        publishState(); }
}

void HuboValveLocalizationWidget::handleUserLeft(bool isChecked) {
    if (isChecked){
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::USER_LEFT;
        publishState(); }
}

void HuboValveLocalizationWidget::handleUserRight(bool isChecked) {
    if (isChecked){
        state_msg_.Hands = valve_localization_panel_msgs::PanelUpdate::USER_RIGHT;
        publishState(); }
}

void HuboValveLocalizationWidget::handleGrippers(bool isChecked) {
    if (isChecked){
        state_msg_.EndEffector = valve_localization_panel_msgs::PanelUpdate::GRIPPER;
        publishState(); }
}

void HuboValveLocalizationWidget::handlePegs(bool isChecked) {
    if (isChecked){
        state_msg_.EndEffector = valve_localization_panel_msgs::PanelUpdate::PEG;
        publishState(); }
}

void HuboValveLocalizationWidget::handleTurnAmount(int value) {
    state_msg_.TurnAmount = value;
    publishState();
}

void HuboValveLocalizationWidget::handleValveRadius(int value) {
    state_msg_.ValveRadius = value;
    publishState();
}

void HuboValveLocalizationWidget::handleLeftCompliant(bool isChecked){
    state_msg_.LeftCompliance = isChecked;
    publishState();
}

void HuboValveLocalizationWidget::handleRightCompliant(bool isChecked){
    state_msg_.RightCompliance = isChecked;
    publishState();
}

void HuboValveLocalizationWidget::handlePlanInBox(bool isChecked){
    state_msg_.PlanInBox = isChecked;
    publishState();
}

void HuboValveLocalizationWidget::handleGrabMiddle(bool isChecked){
    state_msg_.GrabMiddle = isChecked;

    if(isChecked){
        plannerBothRadioButton_->setEnabled(false);
        userBothRadioButton_->setEnabled(false);
        userLeftRadioButton_->setEnabled(false);
        userRightRadioButton_->setEnabled(false);

        if (plannerLeftRadioButton_->isChecked() == false &&
            plannerRightRadioButton_->isChecked() == false) {

                plannerRightRadioButton_->setChecked(true);
        }

    } else {

        if(roundRadioButton_->isChecked()) {
            plannerBothRadioButton_->setEnabled(true);
            userBothRadioButton_->setEnabled(true);
            userLeftRadioButton_->setEnabled(true);
            userRightRadioButton_->setEnabled(true);
            //plannerBothRadioButton_->setChecked(true);
        }
    }

    publishState();
}


}
