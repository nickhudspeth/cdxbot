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
// unsigned int min_feed_plane;
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
    virtual bool aspirate(double vol) {};
    virtual bool dispense(double vol) {};
    virtual bool ejectTip(void) {};
    virtual bool makeDeckGeometry(unsigned int index, double feed_plane,\
                                  double container_offset_z, \
                                  double tip_engagement_len, \
                                  double tip_deposit_height);
    virtual bool moveZ(double pos, double vel) {};
    virtual bool pickUpTip(unsigned int tt_idx, unsigned int dg_idx, bool speed){};
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
    double getFeedPlane(void) {
        return _feed_plane;
    }
    double &getFeedPlaneRef(void) {
        return _feed_plane;
    }
    double getTipPickupSpeed(void){
        return _tip_pickup_speed;
    }
    double &getTipPickupSpeedRef(void){
        return _tip_pickup_speed;
    }

    std::string type;
    std::string driver_name;
    std::string driver_path;
    double _tip_pickup_speed;

  protected:
    double _zpos;
    double _zpos_min;
    double _zpos_max;
    bool _z_axis_enabled = 1;
    double _feed_plane = 1800;
};


typedef PipetterModule *create_t();
typedef void destroy_t(PipetterModule *);
