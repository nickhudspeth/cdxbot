/*************************************************************************
Title:    libairz.h - CDXBot Driver for TriContinent Air-Z Premier Pipetter
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libairz.h
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

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define N0 0 // Normal increment mode
#define N1 1 // Micro-increment mode
#define N2 2 // Microliter mode
#define NORMAL_INCREMENT_MAX 3143 // Maximum displacement in normal increment mode
#define MICRO_INCREMENT_MAX 50288 // Maximum displacement in micro-increment mode
#define MICROLITER_INCREMENT_MAX 1100.050 // Maximum displacement in microliter increment mode

#define IDLE 0
#define BUSY 1
class airZModule : public PipetterModule {
  public:
    airZModule (void);
    virtual ~airZModule ();
    int init(void);
    int deinit(void);
    int lconf(void);
    // void seterrfunc(void(*ef)(std::string s)) {
        // PRINT_ERROR = ef;
    // }
    void moveZ(double pos, double vel) {};
    void pickUpTip(int index) {};
    void pickUpTip(struct container_cell c) {};
    void ejectTip(void) {};
    void aspirate(double vol) {};
    void dispense(double vol) {};

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
    int setID(unsigned int id) {
        if((id < 1) || (id > 16)) {
            /* ERROR: ID out of valid range [1, 16] */
            return -1;
        }
        _id = id;
    }
    unsigned int getID(void) {
        return _id;
    }
    unsigned int &getIDRef(void) {
        return _id;
    }


  private:
    unsigned int _id;
    float _volume;
    unsigned int _capacitance_change_threshold;
    int sendCommand(std::string s, bool run = 1);
    bool liquidLevelDetect(int timeout);
    void terminate(void);
    bool getStatus(void);
        /* data */
};

/***********************    FUNCTION PROTOTYPES    ***********************/
