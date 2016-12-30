/************************************************************************
Title:    gantry-controller.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     gantry-controller.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file gantry-controller.h.

LICENSE:
    Copyright (C) 2016 Nicholas Morrow

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

************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdlib.h>
#include <iostream>
#include "GantryController.h"

GantryController::GantryController(const char *file){

}

GantryController::~GantryController(){

}

void GantryController::dwell(int t){

}

void GantryController::emergencyStop(void){

}

void GantryController::emergencyStopReset(void){

}

int GantryController::home(unsigned int axis){

}

int GantryController::motorsDisable(unsigned int axis){

}

int GantryController::motorsEnable(void){

}

int GantryController::moveAbsolute(float x, float y, float z){

}

int GantryController::moveRelative(float x, float y, float z){

}

int GantryController::setAxisStepsPerMM(unsigned int axis, unsigned int steps){

}

int GantryController::setUnits(unsigned int u){

}

int GantryController::initComms(void){

}

int GantryController::loadConfig(const std::string file){

}

int GantryController::sendCommand(const std::string &s){

}

int GantryController::readResponse(void){

}
