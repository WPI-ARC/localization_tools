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

void HuboValveLocalizationWidget::initializeMainTab()
{

    //======================================================
    //=====        Top Layout                          =====
    //======================================================  

    QVBoxLayout* masterLayout = new QVBoxLayout;

    //Create a Layout to hold the entire upper portion of the main panel
    QVBoxLayout* topLayout = new QVBoxLayout;

    QHBoxLayout* stateLayout = new QHBoxLayout;
    QHBoxLayout* buttonLayout = new QHBoxLayout;
    QHBoxLayout* informationLayout = new QHBoxLayout;

    QVBoxLayout* planButtonsLayout = new QVBoxLayout;
    QVBoxLayout* previewButtonsLayout = new QVBoxLayout;
    QVBoxLayout* executeButtonsLayout = new QVBoxLayout;

    QGridLayout* bottomLayout = new QGridLayout;

    //Make Group boxes for all of the different areas
    QGroupBox* topGroupBox = new QGroupBox;

    QGroupBox* stateGroupBox = new QGroupBox;
    QGroupBox* buttonGroupBox = new QGroupBox;
    QGroupBox* informationGroupBox = new QGroupBox;

    QGroupBox* planButtonsGroupBox = new QGroupBox;
    QGroupBox* previewButtonsGroupBox = new QGroupBox;
    QGroupBox* executeButtonsGroupBox = new QGroupBox;

    QGroupBox* bottomGroupBox = new QGroupBox;




    //Add the layouts to the boxes
    topGroupBox->setStyleSheet(groupStyleSheet);
    topGroupBox->setTitle("Top Panel");
    topGroupBox->setLayout(topLayout);

    stateGroupBox->setStyleSheet(groupStyleSheet);
    stateGroupBox->setTitle("Robot State");
    stateGroupBox->setLayout(stateLayout);

    buttonGroupBox->setStyleSheet(groupStyleSheet);
    buttonGroupBox->setTitle("Planner Buttons");
    buttonGroupBox->setLayout(buttonLayout);

    informationGroupBox->setStyleSheet(groupStyleSheet);
    informationGroupBox->setTitle("Information");
    informationGroupBox->setLayout(informationLayout);

    planButtonsGroupBox->setStyleSheet(groupStyleSheet);
    planButtonsGroupBox->setTitle("Plan");
    planButtonsGroupBox->setLayout(planButtonsLayout);

    previewButtonsGroupBox->setStyleSheet(groupStyleSheet);
    previewButtonsGroupBox->setTitle("Preview");
    previewButtonsGroupBox->setLayout(previewButtonsLayout);

    executeButtonsGroupBox->setStyleSheet(groupStyleSheet);
    executeButtonsGroupBox->setTitle("Execute");
    executeButtonsGroupBox->setLayout(executeButtonsLayout);

    bottomGroupBox->setStyleSheet(groupStyleSheet);
    bottomGroupBox->setTitle("Planner Properties");
    bottomGroupBox->setLayout(bottomLayout);


    //Add the group boxes and layouts to eachother
    masterLayout->addWidget(topGroupBox);
    //masterLayout->addWidget(bottomGroupBox);

    topLayout->addWidget(stateGroupBox);
    topLayout->addWidget(buttonGroupBox);
    topLayout->addWidget(informationGroupBox);

    buttonLayout->addWidget(planButtonsGroupBox);
    buttonLayout->addWidget(previewButtonsGroupBox);
    buttonLayout->addWidget(executeButtonsGroupBox);


    //================================
    //     Robot State Layout
    //================================

    QLabel* robotStateText = new QLabel;
    robotStateText->setText("Robot State: ");
    robotStateText->setStyleSheet("QLabel { color : black; font: bold 20px;}");
    stateLayout->addWidget(robotStateText, 0, Qt::AlignRight);

    robotStateCurrent_ = new QLabel;
    robotStateCurrent_->setText("IDLE");
    robotStateCurrent_->setStyleSheet("QLabel { color : blue; font: bold 20px;}");
    stateLayout->addWidget(robotStateCurrent_, 0, Qt::AlignLeft);

    //================================
    //     Buttons Layout
    //================================

    QSize minimumButtonSize(150,25);

    //Add buttons to the various panels
    plan_getready_button_ = new QPushButton;
    plan_getready_button_ ->setText("Plan GETREADY");
    planButtonsLayout->addWidget(plan_getready_button_, 0, Qt::AlignCenter);
    plan_getready_button_ ->setMinimumSize(minimumButtonSize);

    plan_turning_button_ = new QPushButton;
    plan_turning_button_ ->setText("Plan TURNING");
    planButtonsLayout->addWidget(plan_turning_button_, 0, Qt::AlignCenter);
    plan_turning_button_ ->setMinimumSize(minimumButtonSize);

    plan_finish_button_ = new QPushButton;
    plan_finish_button_ ->setText("Plan FINISH");
    planButtonsLayout->addWidget(plan_finish_button_, 0, Qt::AlignCenter);
    plan_finish_button_ ->setMinimumSize(minimumButtonSize);

    plan_grasp_button_ = new QPushButton;
    plan_grasp_button_ ->setText("TELEOP ENABLED");
    planButtonsLayout->addWidget(plan_grasp_button_, 0, Qt::AlignCenter);
    plan_grasp_button_ ->setMinimumSize(minimumButtonSize);

    plan_grasp_button_->setEnabled(false);

    plan_ungrasp_button_ = new QPushButton;
    plan_ungrasp_button_ ->setText("TELEOP DISABLE");
    planButtonsLayout->addWidget(plan_ungrasp_button_, 0, Qt::AlignCenter);
    plan_ungrasp_button_ ->setMinimumSize(minimumButtonSize);
    
    plan_ungrasp_button_->setEnabled(false);

    preview_button_ = new QPushButton;
    preview_button_ ->setText("Preview");
    previewButtonsLayout->addWidget(preview_button_, 0, Qt::AlignCenter);
    QSize bigButton(150,150);
    preview_button_ ->setMinimumSize(bigButton);


    /**
    preview_grasp_button_ = new QPushButton;
    preview_grasp_button_ ->setText("Preview GRASP");
    previewButtonsLayout->addWidget(preview_grasp_button_, 0, Qt::AlignCenter);
    preview_grasp_button_->setDisabled(true);
    preview_grasp_button_ ->setMinimumSize(minimumButtonSize);

    preview_turning_button_ = new QPushButton;
    preview_turning_button_ ->setText("Preview TURNING");
    previewButtonsLayout->addWidget(preview_turning_button_, 0, Qt::AlignCenter);
    preview_turning_button_->setDisabled(true);
    preview_turning_button_ ->setMinimumSize(minimumButtonSize);

    preview_ungrasp_button_ = new QPushButton;
    preview_ungrasp_button_ ->setText("Preview UNGRASP");
    previewButtonsLayout->addWidget(preview_ungrasp_button_, 0, Qt::AlignCenter);
    preview_ungrasp_button_->setDisabled(true);
    preview_ungrasp_button_ ->setMinimumSize(minimumButtonSize);

    preview_finish_button_ = new QPushButton;
    preview_finish_button_ ->setText("Preview FINISH");
    previewButtonsLayout->addWidget(preview_finish_button_, 0, Qt::AlignCenter);
    preview_finish_button_->setDisabled(true);
    preview_finish_button_ ->setMinimumSize(minimumButtonSize);

    **/

    execute_button_ = new QPushButton;
    execute_button_ ->setText("Execute");
    executeButtonsLayout->addWidget(execute_button_, 0, Qt::AlignCenter);
    execute_button_ ->setMinimumSize(bigButton);

    /**
    execute_grasp_button_ = new QPushButton;
    execute_grasp_button_ ->setText("Execute GRASP");
    executeButtonsLayout->addWidget(execute_grasp_button_, 0, Qt::AlignCenter);
    execute_grasp_button_->setDisabled(true);
    execute_grasp_button_ ->setMinimumSize(minimumButtonSize);

    execute_turning_button_ = new QPushButton;
    execute_turning_button_ ->setText("Execute TURNING");
    executeButtonsLayout->addWidget(execute_turning_button_, 0, Qt::AlignCenter);
    execute_turning_button_->setDisabled(true);
    execute_turning_button_ ->setMinimumSize(minimumButtonSize);

    execute_ungrasp_button_ = new QPushButton;
    execute_ungrasp_button_ ->setText("Execute UNGRASP");
    executeButtonsLayout->addWidget(execute_ungrasp_button_, 0, Qt::AlignCenter);
    execute_ungrasp_button_->setDisabled(true);
    execute_ungrasp_button_ ->setMinimumSize(minimumButtonSize);

    execute_finish_button_ = new QPushButton;
    execute_finish_button_ ->setText("Execute FINISH");
    executeButtonsLayout->addWidget(execute_finish_button_, 0, Qt::AlignCenter);
    execute_finish_button_->setDisabled(true);
    execute_finish_button_ ->setMinimumSize(minimumButtonSize);
    **/

    connect(plan_getready_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePlanGetready(void)));

    connect(plan_grasp_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePlanGrasp(void)));

    connect(plan_turning_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePlanTurning(void)));

    connect(plan_ungrasp_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePlanUngrasp(void)));

    connect(plan_finish_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePlanFinish(void)));

    connect(preview_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePreview(void)));

        /**
    connect(preview_grasp_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePreviewGrasp(void)));

    connect(preview_turning_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePreviewTurning(void)));

    connect(preview_ungrasp_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePreviewUngrasp(void)));

    connect(preview_finish_button_, SIGNAL(clicked(void)),
            this,              SLOT(handlePreviewFinish(void)));
            **/

    connect(execute_button_, SIGNAL(clicked(void)),
            this,              SLOT(handleExecute(void)));

    /**
    connect(execute_grasp_button_, SIGNAL(clicked(void)),
            this,              SLOT(handleExecuteGrasp(void)));

    connect(execute_turning_button_, SIGNAL(clicked(void)),
            this,              SLOT(handleExecuteTurning(void)));

    connect(execute_ungrasp_button_, SIGNAL(clicked(void)),
            this,              SLOT(handleExecuteUngrasp(void)));

    connect(execute_finish_button_, SIGNAL(clicked(void)),
            this,              SLOT(handleExecuteFinish(void)));
            **/
    //================================
    //     Information Layout
    //================================

    //QLabel* informationText = new QLabel;
    //informationText->setText("Information: ");
    //informationText->setStyleSheet("QLabel { color : black; font: 20px;}");
    //informationLayout->addWidget(informationText, 0, Qt::AlignRight);

    informationCurrent_ = new QLabel;
    informationCurrent_->setText("Waiting For Input ...");
    informationCurrent_->setStyleSheet("QLabel { background-color: white; color : black; font: 20px;}");
    informationLayout->addWidget(informationCurrent_, 0, Qt::AlignCenter);
    informationCurrent_->setWordWrap(true);

    //================================
    //     Turn Amount Layout
    //================================


    QLabel* turnAmountText = new QLabel;
    turnAmountText->setText("Turn Amount: ");
    bottomLayout->addWidget(turnAmountText, 0, 0, Qt::AlignRight);

    QSpinBox* turnAmountSpinBox = new QSpinBox;
    turnAmountSpinBox->setRange(0, 360);
    turnAmountSpinBox->setSingleStep(5);
    turnAmountSpinBox->setValue(30);
    bottomLayout->addWidget(turnAmountSpinBox, 0, 1, Qt::AlignCenter);

    QLabel* degreesText = new QLabel;
    degreesText->setText(" Degrees");
    bottomLayout->addWidget(degreesText, 0, 2, Qt::AlignLeft);


    //================================
    //     Valve Radius Layout
    //================================

    QLabel* valveRadiusText = new QLabel;
    valveRadiusText->setText("Valve Radius: ");
    bottomLayout->addWidget(valveRadiusText, 2, 0, Qt::AlignRight);

    QSpinBox* valveRadiusSpinBox = new QSpinBox;
    valveRadiusSpinBox->setRange(0, 40);
    valveRadiusSpinBox->setSingleStep(1);
    valveRadiusSpinBox->setValue(20);
    bottomLayout->addWidget(valveRadiusSpinBox, 2, 1, Qt::AlignCenter);

    QLabel* cmText = new QLabel;
    cmText->setText(" cm");
    bottomLayout->addWidget(cmText, 2, 2, Qt::AlignLeft);


    //Turn Direction
    QLabel* turnDirectionText = new QLabel;
    turnDirectionText->setText("Turn Direction: ");
    bottomLayout->addWidget(turnDirectionText, 0, 4, Qt::AlignRight);

    QLabel* clockwiseText = new QLabel;
    clockwiseText->setText("Clockwise ");
    bottomLayout->addWidget(clockwiseText, 1, 4, Qt::AlignRight);

    QLabel* counterClockwiseText = new QLabel;
    counterClockwiseText->setText("Counter-Clockwise ");
    bottomLayout->addWidget(counterClockwiseText, 2, 4, Qt::AlignRight);

    QButtonGroup* turnDirectionButtonGroup = new QButtonGroup;

    QRadioButton* clockwiseRadioButton = new QRadioButton;
    bottomLayout->addWidget(clockwiseRadioButton, 1, 5, Qt::AlignLeft);

    QRadioButton* counterClockwiseRadioButton = new QRadioButton;
    bottomLayout->addWidget(counterClockwiseRadioButton, 2, 5, Qt::AlignLeft);

    turnDirectionButtonGroup->addButton(clockwiseRadioButton);
    turnDirectionButtonGroup->addButton(counterClockwiseRadioButton);

    //Valve Type
    QLabel* valveTypeText = new QLabel;
    valveTypeText->setText("Valve Type: ");
    bottomLayout->addWidget(valveTypeText, 0, 7, Qt::AlignRight);

    QLabel* roundText = new QLabel;
    roundText->setText("Round ");
    bottomLayout->addWidget(roundText, 1, 7, Qt::AlignRight);

    QLabel* leftLeverText = new QLabel;
    leftLeverText->setText("Left Lever ");
    bottomLayout->addWidget(leftLeverText, 2, 7, Qt::AlignRight);

    QLabel* rightLeverText = new QLabel;
    rightLeverText->setText("Right Lever ");
    bottomLayout->addWidget(rightLeverText, 3, 7, Qt::AlignRight);

    QButtonGroup* valveTypeButtonGroup = new QButtonGroup;

    QRadioButton* roundRadioButton = new QRadioButton;
    bottomLayout->addWidget(roundRadioButton, 1, 8, Qt::AlignLeft);

    QRadioButton* leftLeverRadioButton = new QRadioButton;
    bottomLayout->addWidget(leftLeverRadioButton, 2, 8, Qt::AlignLeft);

    QRadioButton* rightLeverRadioButton = new QRadioButton;
    bottomLayout->addWidget(rightLeverRadioButton, 3, 8, Qt::AlignLeft);


    valveTypeButtonGroup->addButton(roundRadioButton);
    valveTypeButtonGroup->addButton(leftLeverRadioButton);
    valveTypeButtonGroup->addButton(rightLeverRadioButton);

    //Make the unused columns a little bigger
    bottomLayout->setColumnMinimumWidth(3, 20);
    bottomLayout->setColumnMinimumWidth(6, 20);


    //Add the entire layout to the tab
    mainTab = new QWidget;
    mainTab->setLayout(masterLayout);






/**

    //Create a label for the tickmarks on the slider
    QLabel* jointAnglesSliderTicks = new QLabel;
    jointAnglesSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    jointAnglesSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the jointAnglesLayout
    jointAnglesLayout->addWidget(jointAnglesSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* jointAnglesSlider = new QSlider;
    jointAnglesSlider->setObjectName(QString::fromUtf8("jointAnglesSlider"));
    jointAnglesSlider->setOrientation(Qt::Horizontal);
    jointAnglesSlider->setGeometry(QRect(0, 0, 257, 29));
    jointAnglesSlider->setMaximum(7);
    jointAnglesSlider->setSliderPosition(0);
    jointAnglesSlider->setTickPosition(QSlider::TicksAbove);
    jointAnglesSlider->setMinimumSize(QSize(257, 29));
    jointAnglesSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the jointAnglesLayout
    jointAnglesLayout->addWidget(jointAnglesSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* jointAnglesBox = new QGroupBox;
    jointAnglesBox->setStyleSheet(groupStyleSheet);
    jointAnglesBox->setTitle("Joint Angles");
    jointAnglesBox->setLayout(jointAnglesLayout);

    //======================================================
    //===== Forse Sensor Box                           =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* forseSensorLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* forseSensorSliderTicks = new QLabel;
    forseSensorSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    forseSensorSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the forseSensorLayout
    forseSensorLayout->addWidget(forseSensorSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* forceSensorSlider = new QSlider;
    forceSensorSlider->setObjectName(QString::fromUtf8("forceSensorSlider"));
    forceSensorSlider->setOrientation(Qt::Horizontal);
    forceSensorSlider->setGeometry(QRect(0, 0, 257, 29));
    forceSensorSlider->setMaximum(7);
    forceSensorSlider->setSliderPosition(0);
    forceSensorSlider->setTickPosition(QSlider::TicksAbove);
    forceSensorSlider->setMinimumSize(QSize(257, 29));
    forceSensorSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the forseSensorLayout
    forseSensorLayout->addWidget(forceSensorSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* forceSensorBox = new QGroupBox;
    forceSensorBox->setStyleSheet(groupStyleSheet);
    forceSensorBox->setTitle("Force Sensors");
    forceSensorBox->setLayout(forseSensorLayout);

    //======================================================
    //===== Acceleromter / Gyro Box                    =====
    //======================================================  
    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* accelGryoLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* accelGryoSliderTicks = new QLabel;
    accelGryoSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    accelGryoSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the accelGryoLayout
    accelGryoLayout->addWidget(accelGryoSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* accelGryoSlider = new QSlider;
    accelGryoSlider->setObjectName(QString::fromUtf8("accelGryoSlider"));
    accelGryoSlider->setOrientation(Qt::Horizontal);
    accelGryoSlider->setGeometry(QRect(0, 0, 257, 29));
    accelGryoSlider->setMaximum(7);
    accelGryoSlider->setSliderPosition(0);
    accelGryoSlider->setTickPosition(QSlider::TicksAbove);
    accelGryoSlider->setMinimumSize(QSize(257, 29));
    accelGryoSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the accelGryoLayout
    accelGryoLayout->addWidget(accelGryoSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* accelGryoBox = new QGroupBox;
    accelGryoBox->setStyleSheet(groupStyleSheet);
    accelGryoBox->setTitle("Accelerometers and Gryos");
    accelGryoBox->setLayout(accelGryoLayout);

    //======================================================
    //===== Touch Sensor Box                           =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* touchSensorLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* touchSensorSliderTicks = new QLabel;
    touchSensorSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    touchSensorSliderTicks->setText
        ("XX      1        5      10      25     50    100    oo (Hz)");

    //Add the Label to the touchSensorLayout
    touchSensorLayout->addWidget(touchSensorSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* touchSensorSlider = new QSlider;
    touchSensorSlider->setObjectName(QString::fromUtf8("touchSensorSlider"));
    touchSensorSlider->setOrientation(Qt::Horizontal);
    touchSensorSlider->setGeometry(QRect(0, 0, 257, 29));
    touchSensorSlider->setMaximum(7);
    touchSensorSlider->setSliderPosition(0);
    touchSensorSlider->setTickPosition(QSlider::TicksAbove);
    touchSensorSlider->setMinimumSize(QSize(257, 29));
    touchSensorSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the touchSensorLayout
    touchSensorLayout->addWidget(touchSensorSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* touchSensorBox = new QGroupBox;
    touchSensorBox->setStyleSheet(groupStyleSheet);
    touchSensorBox->setTitle("Touch Sensors");
    touchSensorBox->setLayout(touchSensorLayout);

    //======================================================
    //===== Build the Overall Tab                      =====
    //======================================================

    connect(jointAnglesSlider, SIGNAL(valueChanged(int)), 
            this,              SLOT(handleJointAngles(int)));

    connect(forceSensorSlider, SIGNAL(valueChanged(int)), 
            this,              SLOT(handleForceSensor(int)));

    connect(accelGryoSlider, SIGNAL(valueChanged(int)), 
            this,            SLOT(handleAccelGryo(int)));

    connect(touchSensorSlider, SIGNAL(valueChanged(int)), 
            this,              SLOT(handleTouchSensor(int)));

    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(jointAnglesBox);
    masterCTLayout->addWidget(forceSensorBox);
    masterCTLayout->addWidget(accelGryoBox);
    masterCTLayout->addWidget(touchSensorBox);

    //Make a new apply button
    state_apply_button_ = new QPushButton;
    state_apply_button_->setText("Apply");
    masterCTLayout->addWidget(state_apply_button_, 0, Qt::AlignCenter);

    connect(state_apply_button_, SIGNAL(released(void)),
            this,              SLOT(handleStateApply(void)));

**/

}

}
