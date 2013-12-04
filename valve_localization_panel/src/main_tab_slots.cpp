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
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES{ state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::;    publishState();    } LOSS OF        *
 *   USE, DATA, OR PROFITS{ state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::;    publishState();    } OR BUSINESS INTERRUPTION) HOWEVER CAUSED         *
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT             *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN       *
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         *
 *   POSSIBILITY OF SUCH DAMAGE.                                             *
 ****************************************************************************/

#include "valve_localization_panel.h"

//Namespace for the project/plugin
namespace Valve_Localization {

void HuboValveLocalizationWidget::handlePlanGetready(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PLAN_GETREADY;
    informationCurrent_->setText("Planning GetReady ... ");
    publishState();
}

void HuboValveLocalizationWidget::handlePlanGrasp(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::TELEOP_START;
    informationCurrent_->setText("Starting Teleoperation ... ");
    publishState();
}

void HuboValveLocalizationWidget::handlePlanTurning(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PLAN_TURNING;
    informationCurrent_->setText("Planning Turning ... ");
    publishState();
}

void HuboValveLocalizationWidget::handlePlanUngrasp(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::TELEOP_STOP;
    informationCurrent_->setText("Stopping Teleoperation ... ");
    publishState();    }

void HuboValveLocalizationWidget::handlePlanFinish(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PLAN_FINISH;
    informationCurrent_->setText("Planning Finish ... ");
    publishState();    }

void HuboValveLocalizationWidget::handlePreview(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PREVIEW;
    informationCurrent_->setText("Previewing ... ");
    publishState();    }

void HuboValveLocalizationWidget::handleExecute(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::EXECUTE;
    informationCurrent_->setText("Executing ... ");
    publishState();    }

void HuboValveLocalizationWidget::handlePreviewGetready(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PREVIEW;    publishState();    }
void HuboValveLocalizationWidget::handlePreviewGrasp(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PREVIEW;    publishState();    }
void HuboValveLocalizationWidget::handlePreviewTurning(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PREVIEW;    publishState();    }
void HuboValveLocalizationWidget::handlePreviewUngrasp(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PREVIEW;    publishState();    }
void HuboValveLocalizationWidget::handlePreviewFinish(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::PREVIEW;    publishState();    }

void HuboValveLocalizationWidget::handleExecuteGetready(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::EXECUTE;    publishState();    }
void HuboValveLocalizationWidget::handleExecuteGrasp(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::EXECUTE;    publishState();    }
void HuboValveLocalizationWidget::handleExecuteTurning(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::EXECUTE;    publishState();    }
void HuboValveLocalizationWidget::handleExecuteUngrasp(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::EXECUTE;    publishState();    }
void HuboValveLocalizationWidget::handleExecuteFinish(void){
    state_msg_.Command = valve_localization_panel_msgs::PanelUpdate::EXECUTE;    publishState();    }

}
