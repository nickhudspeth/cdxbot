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
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "CDXModule.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
// struct deck_geometry_t {
    // unsigned int index;
    // unsigned int min_traverse_height;
    // unsigned int min_z_pos;
    // unsigned int botpp; // Beginning of Tip Picking Position
    // unsigned int eotpp; // End of Tip Picking Position
    // unsigned int potdp; // Position of Tip Deposit Process
// };


/***********************    FUNCTION PROTOTYPES    ***********************/
class PipetterModule : public CDXModule {
  public:
    PipetterModule (void) {};
    virtual ~PipetterModule (void) {};

    virtual void moveZ(double pos, double vel) {};

    virtual void pickUpTip(int index) {};
    virtual void pickUpTip(struct container_cell c) {};
    virtual void ejectTip(void) {};

    virtual void aspirate(double vol) {};

    virtual void dispense(double vol) {};

    // void setTipParams(struct tip_params t){
        // _tp.min_traverse_height = t.min_traverse_height;
        // _

    // }

    double getZPos(void) {
        return _zpos;
    }
    double &getZPosRef(void) {
        return _zpos;
    }
    bool getZAxisEnabled(void) {
        return _z_axis_enabled;
    }
    bool &getZAxisEnabledRef(void) {
        return _z_axis_enabled;
    }
    double getFeedPlaneHeight(void) {
        return _feed_plane_height;
    }
    double &getFeedPlaneHeightRef(void) {
        return _feed_plane_height;
    }

    std::string type;
    std::string driver_name;
    std::string driver_path;

  protected:
    double _zpos;
    double _zpos_min;
    double _zpos_max;
    bool _z_axis_enabled = 1;
    double _feed_plane_height = 50;
    // struct tip_params _tp;
    /* data */
};


typedef PipetterModule *create_t();
typedef void destroy_t(PipetterModule *);
