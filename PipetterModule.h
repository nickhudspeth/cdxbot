/*************************************************************************
Title:    PipetterModule.h - Interface Class for CDXBot Pipetter Drivers
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     PipetterModule.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:


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
#pragma once

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>
#include "CDXModule.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/


/***********************    FUNCTION PROTOTYPES    ***********************/
class PipetterModule : public CDXModule {
  public:
    PipetterModule (void) {};
    virtual ~PipetterModule (void) {};

    virtual void moveZDrive(double pos, double vel) {};

    virtual void pickUpTip(void) {};

    virtual void ejectTip(void) {};

    virtual void aspirate(double vol) {};

    virtual void dispense(double vol) {};


  protected:
    bool _z_axis_mobile;
    double _zpos;
    double _zpos_min;
    double _zpos_max;
    /* data */
};
