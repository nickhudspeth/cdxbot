/*************************************************************************
Title:    GantryModule.h - CDXBot Gantry Module Template Class
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     GantryModule.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    Provides a template class implementation for all CDXBot hardware modules of
    the gantry type. This class is derived from the CDX module base class.

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
#define UNITS_MM 0
#define UNITS_IN 1
#define AXIS_ALL 0
#define AXIS_X 1
#define AXIS_Y 2
#define AXIS_Z 3
#define MOVE_MODE_ABSOLUTE 0
#define MOVE_MODE_RELATIVE 1

class GantryModule : public CDXModule {
  public:
    GantryModule (void) {};
    virtual ~GantryModule(void) {};

    /*************************************************************************
    * Function :   dwell()
    * Purpose  :   Dwell for t milliseconds
    * Input    :   int t
    * Returns  :   virtual void
    *************************************************************************/
    virtual void dwell(int t) {};

    /*************************************************************************
    * Function :   emergencyStop()
    * Purpose  :   Performs an emergency stop, immediately halting the motion
    *              the machine.
    * Input    :   void
    * Returns  :   void
    *************************************************************************/
    virtual void emergencyStop() {};

    /*************************************************************************
    * Function :   emergencyStopReset()
    * Purpose  :   Resets from a halted state caused by a limit switch, M112,
    *              or kill switch.
    * Input    :   void
    * Returns  :   virtual void
    ************************************************************************/
    virtual void emergencyStopReset() {};

    /*************************************************************************
    * Function :   home()
    * Purpose  :   Homes the given axis. If no argument is passed, all three
    *              axes will be homed simultaneously.
    * Input    :   unsigned int axis
    * Returns  :   virtual int
    *************************************************************************/
    virtual int home(unsigned int axis = 0) {};


    /*************************************************************************
    * Function :   motorsDisable()
    * Purpose  :   Disables the stepper motors
    * Input    :   int axis
    * Returns  :   virtual int
    *************************************************************************/
    virtual int motorsDisable(unsigned int axis = 0) {};

    /*************************************************************************
    * Function :   motorsEnable()
    * Purpose  :   Enables the stepper motors
    * Input    :   void
    * Returns  :   virtual int
    *************************************************************************/
    virtual int motorsEnable(unsigned int axis) {};

    /*************************************************************************
    * Function :   moveAbsolute()
    * Purpose  :   Command the end effector to move to position specified by
    *              the position vector <x,y,z>.
    * Input    :   std::float x, std::float y, std::float z
    * Returns  :   int
    *************************************************************************/
    virtual int moveAbsolute(float x, float y, float z) {};

    /*************************************************************************
    * Function :   moveRelative()
    * Purpose  :   Command the end effector to move to the position specified
    *              by the position vector <cx + x, cy + y, cz +z>, where c(n)
    *              is the nth element of the vector specifying the current
    *              position of the end effector.
    * Input    :   float x, float y, float z
    * Returns  :   int
    *************************************************************************/
    virtual int moveRelative(float x, float y, float z) {};

    /*************************************************************************
    * Function :   setAxisStepsPerUnit()
    * Purpose  :   Sets the axis drive ratio in steps/mm
    * Input    :   unsigned int axis, unsigned int steps
    * Returns  :   virtual int
    *************************************************************************/
    virtual int setAxisStepsPerUnit(unsigned int axis, unsigned int steps) {};

    /*************************************************************************
    * Function :   setUnits()
    * Purpose  :   Sets the default system of units used by the controller.
    * Input    :   unsigned int u
    * Returns  :   virtual int
    *************************************************************************/
    virtual int setUnits(unsigned int u = UNITS_MM) {};

    /*************************************************************************
    * Function :   getPos()
    * Purpose  :   Returns the current position for the given axis.
    * Input    :   const char *axis
    * Returns  :   double
    *************************************************************************/
    double getPos(unsigned int axis) {
        switch(axis) {
        case AXIS_X:
            return _pos[0];
            break;
        case AXIS_Y:
            return _pos[1];
            break;
        case AXIS_Z:
            return _pos[2];
            break;
        default:
            break;
        }
        return 0;
    }

    std::string getType() {
        return _type;
    }

    std::string &getTypeRef() {
        return _type;
    }

    void setType(std::string s) {
        _type = s;
    }

    // std::string getDriverName() {
    // return _driver_name;
    // }

    // std::string &getDriverNameRef() {
    // return _driver_name;
    // }

    // void setDriverName(std::string s) {
    // _driver_name = s;
    // }

    // std::string getDriverPath() {
    // return _driver_path;
    // }

    // std::string &getDriverPathRef() {
    // return _driver_path;
    // }

    // void setDriverPath(std::string s) {
    // _driver_path = s;
    // }

    std::string getIPAddress() {
        return _ip_address;
    }

    std::string &getIPAddressRef() {
        return _ip_address;
    }

    void setIPAddress(std::string s) {
        _ip_address = s;
    }

    int getPort(void) {
        return _port;
    }

    int &getPortRef(void) {
        return _port;
    }

    void setPort(unsigned int p) {
        _port = p;
    }

    int getTimeout(void) {
        return _timeout;
    }

    int &getTimeoutRef(void) {
        return _timeout;
    }

    void setTimeout(unsigned int p) {
        _timeout = p;
    }

    int getBufferSize(void) {
        return _buffer_size;
    }

    int &getBufferSizeRef(void) {
        return _buffer_size;
    }

    void setBufferSize(unsigned int p) {
        _buffer_size = p;
    }

    bool getUnits() {
        return _units;
    }

    bool &getUnitsRef() {
        return _units;
    }

    double getTraverseVelocity(void) {
        return _traverse_velocity;
    }

    double &getTraverseVelocityRef(void) {
        return _traverse_velocity;
    }

    void setMinFeedrate(int axis, double rate) {
        if(axis == AXIS_X) {
            _feedrate_x_min = rate;
        } else if(axis == AXIS_Y) {
            _feedrate_y_min = rate;
        }
        if(axis == AXIS_Z) {
            _feedrate_z_min = rate;
        }
        if(axis == AXIS_ALL) {
            _feedrate_x_min = rate;
            _feedrate_y_min = rate;
            _feedrate_z_min = rate;
        }
    }

    void setMaxFeedrate(int axis, double rate) {
        if(axis == AXIS_X) {
            _feedrate_x_max = rate;
        } else if(axis == AXIS_Y) {
            _feedrate_y_max = rate;
        }
        if(axis == AXIS_Z) {
            _feedrate_z_max = rate;
        }
        if(axis == AXIS_ALL) {
            _feedrate_x_max = rate;
            _feedrate_y_max = rate;
            _feedrate_z_max = rate;
        }
    }

    void setCurrentFeedrate(int axis, double rate) {
        if(axis == AXIS_X) {
            _feedrate_x_cur = rate;
        } else if(axis == AXIS_Y) {
            _feedrate_y_cur = rate;
        }
        if(axis == AXIS_Z) {
            _feedrate_z_cur = rate;
        }
        if(axis == AXIS_ALL) {
            _feedrate_x_cur = rate;
            _feedrate_y_cur = rate;
            _feedrate_z_cur = rate;
        }
    }

    void setMinAccel(int axis, double rate) {
        if(axis == AXIS_X) {
            _accel_x_min = rate;
        } else if(axis == AXIS_Y) {
            _accel_y_min = rate;
        }
        if(axis == AXIS_Z) {
            _accel_z_min = rate;
        }
        if(axis == AXIS_ALL) {
            _accel_x_min = rate;
            _accel_y_min = rate;
            _accel_z_min = rate;
        }
    }

    void setMaxAccel(int axis, double rate) {
        if(axis == AXIS_X) {
            _accel_x_max = rate;
        } else if(axis == AXIS_Y) {
            _accel_y_max = rate;
        }
        if(axis == AXIS_Z) {
            _accel_z_max = rate;
        }
        if(axis == AXIS_ALL) {
            _accel_x_max = rate;
            _accel_y_max = rate;
            _accel_z_max = rate;
        }
    }

    void setCurrentAccel(int axis, double rate) {
        if(axis == AXIS_X) {
            _accel_x_cur = rate;
        } else if(axis == AXIS_Y) {
            _accel_y_cur = rate;
        }
        if(axis == AXIS_Z) {
            _accel_z_cur = rate;
        }
        if(axis == AXIS_ALL) {
            _accel_x_cur = rate;
            _accel_y_cur = rate;
            _accel_z_cur = rate;
        }
    }
    void setTraverseVelocity(double p) {
        _traverse_velocity = p;
    }

    double getRapidFeedVelocity(void) {
        return _rapid_feed_velocity;
    }

    double &getRapidFeedVelocityRef(void) {
        return _rapid_feed_velocity;
    }

    void setRapidFeedVelocity(double p) {
        _rapid_feed_velocity = p;
    }

    double getXPosMin(void) {
        return _pos_min[0];
    }

    double &getXPosMinRef(void) {
        return _pos_min[0];
    }

    void setXPosMin(double p) {
        _pos_min[0] = p;
    }

    double getXPosMax(void) {
        return _pos_max[0];
    }

    double &getXPosMaxRef(void) {
        return _pos_max[0];
    }

    void setXPosMax(double p) {
        _pos_max[0] = p;
    }

    double getYPosMin(void) {
        return _pos_min[1];
    }

    double &getYPosMinRef(void) {
        return _pos_min[1];
    }

    void setYPosMin(double p) {
        _pos_min[1] = p;
    }

    double getYPosMax(void) {
        return _pos_max[1];
    }

    double &getYPosMaxRef(void) {
        return _pos_max[1];
    }

    void setYPosMax(double p) {
        _pos_max[1] = p;
    }

    double getZPosMin(void) {
        return _pos_min[2];
    }

    double &getZPosMinRef(void) {
        return _pos_min[2];
    }

    void setZPosMin(double p) {
        _pos_min[2] = p;
    }

    double getZPosMax(void) {
        return _pos_max[2];
    }

    double &getZPosMaxRef(void) {
        return _pos_max[2];
    }

    void setZPosMax(double p) {
        _pos_max[2] = p;
    }

    void setMoveMode(int m) {
        if(m > 0) {
            m = 1;
        }
        if(m < 0) {
            m = 0;
        }
        _move_mode = m;
    }

    int getMoveMode(void) {
        return _move_mode;
    }

    int &getMoveModeRef(void) {
        return _move_mode;
    }

  protected:
    std::string _type = "";
    // std::string _units = "";
    bool _units = UNITS_MM;
    double _traverse_velocity;
    double _rapid_feed_velocity;
    double _feedrate_x_cur;
    double _feedrate_x_max;
    double _feedrate_x_min;
    double _feedrate_y_cur;
    double _feedrate_y_max;
    double _feedrate_y_min;
    double _feedrate_z_cur;
    double _feedrate_z_max;
    double _feedrate_z_min;
    double _accel_x_cur;
    double _accel_x_max;
    double _accel_x_min;
    double _accel_y_cur;
    double _accel_y_max;
    double _accel_y_min;
    double _accel_z_cur;
    double _accel_z_max;
    double _accel_z_min;
    double _steps_per_unit[3];
    double _pos[3];
    double _pos_min[3];
    double _pos_max[3];
    unsigned int _id;

    std::string _ip_address;
    int _port;
    int _timeout;
    int _buffer_size;
    int _move_mode = MOVE_MODE_ABSOLUTE; /* [0,1] Absolute/Relative
    /*************************************************************************
    * Function :   sendCommand()
    * Purpose  :   Send the G-Code command specified by cmd to the controller.
    * Input    :   const std::string cmd
    * Returns  :   virtual int
    *************************************************************************/
    virtual int sendCommand(const std::string s) {};

    /*************************************************************************
    * Function :   readResponse()
    * Purpose  :   What does this function do?
    * Input    :   void
    * Returns  :   virtual int - number of bytes read.
    *************************************************************************/
    virtual int  readResponse(void) {};
};

/***********************    FUNCTION PROTOTYPES    ***********************/

typedef GantryModule *create_t();
typedef void destroy_t(GantryModule *);
