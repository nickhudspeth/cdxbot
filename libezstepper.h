/*************************************************************************
Title:    libezstepper.h - CDXBot EZStepper Single-Axis Driver
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libezstepper.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:
    

NOTES:


LICENSE:
    Copyright (C) 2017 Nicholas Morrow

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*************************************************************************/
#define MOVE_CURRENT_MAX 100
#define MICROSTEPS_PER_STEP 256
#define STEPS_PER_REVOLUTION 200
#define MM_PER_REVOLUTION 1
#define LINEAR_MULTIPLIER int(round(STEPS_PER_REVOLUTION * MICROSTEPS_PER_STEP/MM_PER_REVOLUTION))
/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include "GantryModule.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/

class EZStepperModule : GantryModule
{
public:
    EZStepperModule (unsigned int id);
    ~EZStepperModule ();
    void dwell(int t);
    void emergencyStop();
    void emergencyStopReset();
    int home(unsigned int axis = 0);
    int motorsDisable(unsigned int axis = 0);
    int motorsEnable(void);
    int moveAbsolute(float pos, bool units = UNITS_MM);
    int moveRelative(float pos, bool units = UNITS_MM);
    int setAxisStepsPerMM(unsigned int axis, unsigned int steps);
    int setUnits(unsigned int u = UNITS_MM);
    void setCurrentPosition(float pos, bool units = UNITS_MM);
    double getPos(unsigned int axis);


private:
    int sendCommand(const std::string s);
    int readResponse(void);
    void setHomeFlagPolarity(bool p1, bool p2, bool p3, bool p4);\
    void setPositiveRotationDirection(bool dir);
    void setVelocity(int vel);
    void setAccelerationFactor(unsigned int factor);
    void setMaxMoveCurrent(unsigned int cmax);
    void setMaxHoldCurrent(unsigned int cmax);
    void setMicroStepResolution(unsigned int res);
    void setBaudRate(unsigned int baud);
    void delay(unsigned int msecs);
    void setBacklashCompensation(unsigned int k);
    unsigned int getCommandedMotorPosition(void);
    unsigned int getPositionModeMotorSpeed(void);
    unsigned int getInputStatus(void);
    unsigned int getVelocityModeMotorSpeed(void);
    unsigned int getMotorStepSize(void);
    int _current_move_max = 100;
    int _current_hold_max = 50;
    int _linear_multiplier = 1;
    int _microsteps_per_step = 256;
    int _steps_per_revolution = 200;
    int _mm_per_revolution = 1;
    int _current_pos = 0;
    int _current_velocity = 1;
    unsigned int _id = 0;
    int _USB = -1; // Serial port file descriptor
    std::string _last_msg;
    // bool _units = UNITS_MM;
    /* data */
};

/***********************    FUNCTION PROTOTYPES    ***********************/



