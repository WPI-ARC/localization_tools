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

void HuboValveLocalizationWidget::initializeSecondTab() {

    //======================================================
    //=====        Top Layout                          =====
    //======================================================

    QVBoxLayout* masterLayout = new QVBoxLayout;

    //Create a Layout to hold the entire upper portion of the main panel
    QGridLayout* bottomLayout = new QGridLayout;

    //Make Group boxes for all of the different areas
    QGroupBox* bottomGroupBox = new QGroupBox;




    //Add the layouts to the boxes
    bottomGroupBox->setStyleSheet(groupStyleSheet);
    bottomGroupBox->setTitle("Planner Properties");
    bottomGroupBox->setLayout(bottomLayout);


    //Add the group boxes and layouts to eachother
    masterLayout->addWidget(bottomGroupBox);

    //Try and get the stretch right
    bottomLayout->setRowStretch(0, 0);
    bottomLayout->setRowStretch(1, 0);
    bottomLayout->setRowStretch(2, 0);
    bottomLayout->setRowStretch(3, 0);
    bottomLayout->setRowStretch(4, 0);
    bottomLayout->setRowStretch(5, 0);
    bottomLayout->setRowStretch(6, 0);
    bottomLayout->setRowStretch(7, 0);
    bottomLayout->setRowStretch(8, 0);
    bottomLayout->setRowStretch(9, 1);


    //================================
    //     Turn Amount Layout
    //================================


    QLabel* turnAmountText = new QLabel;
    turnAmountText->setText("Max Turn: ");
    bottomLayout->addWidget(turnAmountText, 0, 0, Qt::AlignRight);

    turnAmountSpinBox_ = new QSpinBox;
    turnAmountSpinBox_->setRange(0, 360);
    turnAmountSpinBox_->setSingleStep(5);
    turnAmountSpinBox_->setValue(30);
    bottomLayout->addWidget(turnAmountSpinBox_, 0, 1, Qt::AlignCenter);

    QLabel* degreesText = new QLabel;
    degreesText->setText(" Degrees");
    bottomLayout->addWidget(degreesText, 0, 2, Qt::AlignLeft);

    connect(turnAmountSpinBox_, SIGNAL(valueChanged(int)),
            this,              SLOT(handleTurnAmount(int)));


    //================================
    //     Valve Radius Layout
    //================================

    QLabel* valveRadiusText = new QLabel;
    valveRadiusText->setText("Valve Radius: ");
    bottomLayout->addWidget(valveRadiusText, 1, 0, Qt::AlignRight);

    valveRadiusSpinBox_ = new QSpinBox;
    valveRadiusSpinBox_->setRange(0, 40);
    valveRadiusSpinBox_->setSingleStep(1);
    valveRadiusSpinBox_->setValue(20);
    bottomLayout->addWidget(valveRadiusSpinBox_, 1, 1, Qt::AlignCenter);

    QLabel* cmText = new QLabel;
    cmText->setText(" cm");
    bottomLayout->addWidget(cmText, 1, 2, Qt::AlignLeft);

    connect(valveRadiusSpinBox_, SIGNAL(valueChanged(int)),
            this,              SLOT(handleValveRadius(int)));

    //================================
    //     Plan In Box
    //================================

    QLabel* planInBoxText = new QLabel;
    planInBoxText->setText("Plan In Box? ");
    bottomLayout->addWidget(planInBoxText, 2, 0, Qt::AlignRight);

    planInBoxBox_ = new QCheckBox;
    planInBoxBox_->setChecked(true);
    bottomLayout->addWidget(planInBoxBox_, 2, 1, Qt::AlignCenter);

    connect(planInBoxBox_, SIGNAL(toggled(bool)),
            this,          SLOT(handlePlanInBox(bool)));


    //================================
    //     Turn Direction Layout
    //================================
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

    clockwiseRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(clockwiseRadioButton_, 1, 5, Qt::AlignLeft);

    counterClockwiseRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(counterClockwiseRadioButton_, 2, 5, Qt::AlignLeft);

    turnDirectionButtonGroup->addButton(clockwiseRadioButton_);
    turnDirectionButtonGroup->addButton(counterClockwiseRadioButton_);

    connect(clockwiseRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleClockwiseTurnDirection(bool)));

    connect(counterClockwiseRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleCounterClockwiseTurnDirection(bool)));

    //================================
    //     Valve Type Layout
    //================================
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

    roundRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(roundRadioButton_, 1, 8, Qt::AlignLeft);

    leftLeverRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(leftLeverRadioButton_, 2, 8, Qt::AlignLeft);

    rightLeverRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(rightLeverRadioButton_, 3, 8, Qt::AlignLeft);

    valveTypeButtonGroup->addButton(roundRadioButton_);
    valveTypeButtonGroup->addButton(leftLeverRadioButton_);
    valveTypeButtonGroup->addButton(rightLeverRadioButton_);

    connect(roundRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleRound(bool)));

    connect(leftLeverRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleLeftLever(bool)));

    connect(rightLeverRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleRightLever(bool)));

    //================================
    //     Hand Use Layout
    //================================
    QLabel* handsText = new QLabel;
    handsText->setText("Hand Use: ");
    bottomLayout->addWidget(handsText, 5, 0, Qt::AlignRight);

    QLabel* plannerBothText = new QLabel;
    plannerBothText->setText("Planner BOTH");
    bottomLayout->addWidget(plannerBothText, 6, 0, Qt::AlignRight);

    QLabel* plannerLeftText = new QLabel;
    plannerLeftText->setText("Planner LEFT");
    bottomLayout->addWidget(plannerLeftText, 7, 0, Qt::AlignRight);

    QLabel* plannerRightText = new QLabel;
    plannerRightText->setText("Planner RIGHT");
    bottomLayout->addWidget(plannerRightText, 8, 0, Qt::AlignRight);

    QLabel* userBothText = new QLabel;
    userBothText->setText("User BOTH");
    bottomLayout->addWidget(userBothText, 6, 2, Qt::AlignRight);

    QLabel* userLeftText = new QLabel;
    userLeftText->setText("User LEFT");
    bottomLayout->addWidget(userLeftText, 7, 2, Qt::AlignRight);

    QLabel* userRightText = new QLabel;
    userRightText->setText("User RIGHT");
    bottomLayout->addWidget(userRightText, 8, 2, Qt::AlignRight);

    QButtonGroup* handsButtonGroup = new QButtonGroup;

    plannerBothRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(plannerBothRadioButton_, 6, 1, Qt::AlignLeft);

    plannerLeftRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(plannerLeftRadioButton_, 7, 1, Qt::AlignLeft);

    plannerRightRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(plannerRightRadioButton_, 8, 1, Qt::AlignLeft);

    userBothRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(userBothRadioButton_, 6, 3, Qt::AlignLeft);

    userLeftRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(userLeftRadioButton_, 7, 3, Qt::AlignLeft);

    userRightRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(userRightRadioButton_, 8, 3, Qt::AlignLeft);

    handsButtonGroup->addButton(plannerBothRadioButton_);
    handsButtonGroup->addButton(plannerLeftRadioButton_);
    handsButtonGroup->addButton(plannerRightRadioButton_);
    handsButtonGroup->addButton(userBothRadioButton_);
    handsButtonGroup->addButton(userLeftRadioButton_);
    handsButtonGroup->addButton(userRightRadioButton_);

    connect(plannerBothRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handlePlannerBoth(bool)));

    connect(plannerLeftRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handlePlannerLeft(bool)));

    connect(plannerRightRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handlePlannerRight(bool)));

    connect(userBothRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleUserBoth(bool)));

    connect(userLeftRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleUserLeft(bool)));

    connect(userRightRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleUserRight(bool)));


    QLabel* grabMiddleText = new QLabel;
    grabMiddleText->setText("Grab Middle? ");
    bottomLayout->addWidget(grabMiddleText, 10, 0, Qt::AlignRight);

    grabMiddleBox_ = new QCheckBox;
    grabMiddleBox_->setChecked(true);
    bottomLayout->addWidget(grabMiddleBox_, 10, 1, Qt::AlignCenter);

    grabMiddleBox_->setChecked(false);

    connect(grabMiddleBox_, SIGNAL(toggled(bool)),
            this,          SLOT(handleGrabMiddle(bool)));

    //================================
    //     Compliance Layout
    //================================
    QLabel* complianceText = new QLabel;
    complianceText->setText("Compliance: ");
    bottomLayout->addWidget(complianceText, 5, 4, Qt::AlignRight);

    QLabel* leftComplianceText = new QLabel;
    leftComplianceText->setText("Left Arm ");
    bottomLayout->addWidget(leftComplianceText, 6, 4, Qt::AlignRight);

    QLabel* rightComplianceText = new QLabel;
    rightComplianceText->setText("Right Arm ");
    bottomLayout->addWidget(rightComplianceText, 7, 4, Qt::AlignRight);

    leftArmCompliantBox_ = new QCheckBox;
    leftArmCompliantBox_->setChecked(false);
    bottomLayout->addWidget(leftArmCompliantBox_, 6, 5, Qt::AlignCenter);

    rightArmCompliantBox_ = new QCheckBox;
    rightArmCompliantBox_->setChecked(false);
    bottomLayout->addWidget(rightArmCompliantBox_, 7, 5, Qt::AlignCenter);

    connect(leftArmCompliantBox_, SIGNAL(toggled(bool)),
            this,          SLOT(handleLeftCompliant(bool)));

    connect(rightArmCompliantBox_, SIGNAL(toggled(bool)),
            this,          SLOT(handleRightCompliant(bool)));


    //================================
    //     End Effector Layout
    //================================
    QLabel* endEffectorText = new QLabel;
    endEffectorText->setText("End Effector: ");
    bottomLayout->addWidget(endEffectorText, 5, 7, Qt::AlignRight);

    QLabel* grippersText = new QLabel;
    grippersText->setText("Grippers");
    bottomLayout->addWidget(grippersText, 6, 7, Qt::AlignRight);

    QLabel* pegsText = new QLabel;
    pegsText->setText("Pegs");
    bottomLayout->addWidget(pegsText, 7, 7, Qt::AlignRight);

    QButtonGroup* endEffectorButtonGroup = new QButtonGroup;

    grippersRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(grippersRadioButton_, 6, 8, Qt::AlignLeft);

    pegsRadioButton_ = new QRadioButton;
    bottomLayout->addWidget(pegsRadioButton_, 7, 8, Qt::AlignLeft);

    endEffectorButtonGroup->addButton(grippersRadioButton_);
    endEffectorButtonGroup->addButton(pegsRadioButton_);

    connect(grippersRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handleGrippers(bool)));

    connect(pegsRadioButton_, SIGNAL(toggled(bool)),
            this,              SLOT(handlePegs(bool)));

    //Make the unused columns a little bigger
    bottomLayout->setColumnMinimumWidth(3, 20);
    bottomLayout->setColumnMinimumWidth(6, 20);
    bottomLayout->setRowMinimumHeight(4, 40);

    //Defaults
    counterClockwiseRadioButton_->setChecked(true);
    roundRadioButton_->setChecked(true);
    plannerBothRadioButton_->setChecked(true);
    grippersRadioButton_->setChecked(true);


    //Add the entire layout to the tab
    secondTab = new QWidget;
    secondTab->setLayout(masterLayout);


    //*****************************
    // Jim's Button
    //*****************************

    jimsButton_ = new QPushButton();
    jimsButton_->setText("Joint Limit Disruption");
    bottomLayout->addWidget(jimsButton_, 10, 5, 2, 5, Qt::AlignCenter);

    connect(jimsButton_, SIGNAL(clicked(void)),
            this,        SLOT(handleJointLimitDisrupt(void)));

    //*****************************
    // Enable or Disable Buttons
    //*****************************

    leftArmCompliantBox_->setEnabled(false);
    rightArmCompliantBox_->setEnabled(false);
    userBothRadioButton_->setEnabled(false);
    userLeftRadioButton_->setEnabled(false);
    userRightRadioButton_->setEnabled(false);
    rightLeverRadioButton_->setEnabled(false);
    grabMiddleBox_->setEnabled(false);
    pegsRadioButton_->setEnabled(false);




    /**

    //======================================================
    //=====             Camera Feed Box                =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* cameraFeedLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* cameraFeedSliderTicks = new QLabel;
    cameraFeedSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    cameraFeedSliderTicks->setText
        ("XX      .1      .3       .5       1        5       10     oo (Hz)");

    //Add the Label to the cameraFeedLayout
    cameraFeedLayout->addWidget(cameraFeedSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* cameraFeedSlider = new QSlider;
    cameraFeedSlider->setObjectName(QString::fromUtf8("cameraFeedSlider"));
    cameraFeedSlider->setOrientation(Qt::Horizontal);
    cameraFeedSlider->setGeometry(QRect(0, 0, 257, 29));
    cameraFeedSlider->setMaximum(7);
    cameraFeedSlider->setSliderPosition(0);
    cameraFeedSlider->setTickPosition(QSlider::TicksAbove);
    cameraFeedSlider->setMinimumSize(QSize(257, 29));
    cameraFeedSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the cameraFeedLayout
    cameraFeedLayout->addWidget(cameraFeedSlider);

    //Add the checkbox for Greyscale
    QCheckBox* cameraFeedGrayscale = new QCheckBox;
    cameraFeedGrayscale->setText("Grayscale"); 

    //Add the checkbox to the cameraFeedLayout
    cameraFeedLayout->addWidget(cameraFeedGrayscale);

    //Create the box that will hold all of the camera controls
    QGroupBox* cameraFeedBox = new QGroupBox;
    cameraFeedBox->setStyleSheet(groupStyleSheet);
    cameraFeedBox->setTitle("Head Camera Feed");
    cameraFeedBox->setLayout(cameraFeedLayout);
    cameraFeedBox->setMaximumSize(QSize(325, 100));

    //======================================================
    //=====           Planar Laser Scanner             =====
    //======================================================  

    //Create the layout for the specific Camera Slider Item
    QVBoxLayout* planarLaserLayout = new QVBoxLayout;

    //Create a label for the tickmarks on the slider
    QLabel* planerLaserSliderTicks = new QLabel;
    planerLaserSliderTicks->setSizePolicy
        (QSizePolicy::Maximum, QSizePolicy::Maximum);
    planerLaserSliderTicks->setText
            ("XX      .1      .3       .5       1        5       10     oo (Hz)");

    //Add the Label to the planarLaserLayout
    planarLaserLayout->addWidget(planerLaserSliderTicks, 0, Qt::AlignLeft);
    
    //Create the slider for the camera feed
    QSlider* planarLaserSlider = new QSlider;
    planarLaserSlider->setObjectName(QString::fromUtf8("planarLaserSlider"));
    planarLaserSlider->setOrientation(Qt::Horizontal);
    planarLaserSlider->setGeometry(QRect(0, 0, 257, 29));
    planarLaserSlider->setMaximum(7);
    planarLaserSlider->setSliderPosition(0);
    planarLaserSlider->setTickPosition(QSlider::TicksAbove);
    planarLaserSlider->setMinimumSize(QSize(257, 29));
    planarLaserSlider->setMaximumSize(QSize(257, 29));

    //Add the Slider to the planarLaserLayout
    planarLaserLayout->addWidget(planarLaserSlider);

    //Create the box that will hold all of the camera controls
    QGroupBox* planarLaserBox = new QGroupBox;
    planarLaserBox->setStyleSheet(groupStyleSheet);
    planarLaserBox->setTitle("Planar Laser Scanner");
    planarLaserBox->setLayout(planarLaserLayout);
    planarLaserBox->setMaximumSize(QSize(325, 100));

    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(cameraFeedBox, 0, Qt::AlignTop);
    masterCTLayout->addWidget(planarLaserBox, 0, Qt::AlignTop);

    //Make a new apply button
    vision_apply_button_ = new QPushButton;
    vision_apply_button_->setText("Apply");

    masterCTLayout->addWidget(vision_apply_button_, 0, Qt::AlignCenter);

    //======================================================
    //=====           Build the Overall Tab            =====
    //======================================================

    connect(cameraFeedSlider, SIGNAL(valueChanged(int)),
            this,              SLOT(handleCameraFeed(int)));

    connect(planarLaserSlider, SIGNAL(valueChanged(int)),
            this,              SLOT(handlePlanar(int)));

    connect(vision_apply_button_, SIGNAL(released(void)),
            this,              SLOT(handleVisionApply(void)));


    robotVisionFeedTab = new QWidget;
    robotVisionFeedTab->setLayout(masterCTLayout);

    **/
}

}
