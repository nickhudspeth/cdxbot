/*************************************************************************
Title:    GantryController.h -
Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
File:     ControllerBase.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:


LICENSE:
    Copyright (C) 2016 Crystal Diagnostics

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
#include <cstdlib>
#include <boost/timer.hpp>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "common.h"
#include <dlfcn.h>
#include <ros/ros.h>
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/

/***********************    FUNCTION PROTOTYPES    ***********************/
class GantryController {
  public:
    GantryController () {
    }

    virtual ~GantryController ();


    /*************************************************************************
    * Function :   loadDriver()
    * Purpose  :   Loads a hardware driver fron the specified location.
    * Input    :   std::string file
    * Returns  :   int
    *************************************************************************/
    int loadDriver(void);


    /*************************************************************************
    * Function :   dwell()
    * Purpose  :   Dwell for t milliseconds
    * Input    :   int t
    * Returns  :   virtual void
    *************************************************************************/
    virtual void dwell(int t);

    /*************************************************************************
    * Function :   emergencyStop()
    * Purpose  :   Performs an emergency stop, immediately halting the motion
    *              the machine.
    * Input    :   void
    * Returns  :   void
    *************************************************************************/
    virtual void emergencyStop(void);

    /*************************************************************************
    * Function :   emergencyStopReset()
    * Purpose  :   Resets from a halted state caused by a limit switch, M112,
    *              or kill switch.
    * Input    :   void
    * Returns  :   virtual void
    *************************************************************************/
    virtual void emergencyStopReset(void);

    /*************************************************************************
    * Function :   home()
    * Purpose  :   Homes the given axis. If no argument is passed, all three
    *              axes will be homed simultaneously.
    * Input    :   unsigned int axis
    * Returns  :   virtual int
    *************************************************************************/
    virtual int home(unsigned int axis = 0);


    /*************************************************************************
    * Function :   motorsDisable()
    * Purpose  :   Disables the stepper motors
    * Input    :   int axis
    * Returns  :   virtual int
    *************************************************************************/
    virtual int motorsDisable(unsigned int axis = 0);

    /*************************************************************************
    * Function :   motorsEnable()
    * Purpose  :   Enables the stepper motors
    * Input    :   void
    * Returns  :   virtual int
    *************************************************************************/
    virtual int motorsEnable(void);

    /*************************************************************************
    * Function :   moveAbsolute()
    * Purpose  :   Command the end effector to move to position specified by
    *              the position vector <x,y,z>.
    * Input    :   std::float x, std::float y, std::float z
    * Returns  :   int
    *************************************************************************/
    virtual int moveAbsolute(float x, float y, float z);

    /*************************************************************************
    * Function :   moveRelative()
    * Purpose  :   Command the end effector to move to the position specified
    *              by the position vector <cx + x, cy + y, cz +z>, where c(n)
    *              is the nth element of the vector specifying the current
    *              position of the end effector.
    * Input    :   float x, float y, float z
    * Returns  :   int
    *************************************************************************/
    virtual int moveRelative(float x, float y, float z);

    /*************************************************************************
    * Function :   setAxisStepsPerMM()
    * Purpose  :   Sets the axis drive ratio in steps/mm
    * Input    :   unsigned int axis, unsigned int steps
    * Returns  :   virtual int
    *************************************************************************/
    virtual int setAxisStepsPerMM(unsigned int axis, unsigned int steps);

    /*************************************************************************
    * Function :   setUnits()
    * Purpose  :   Sets the default system of units used by the controller.
    * Input    :   unsigned int u
    * Returns  :   virtual int
    *************************************************************************/
    virtual int setUnits(unsigned int u = UNITS_MM);

    /*************************************************************************
    * Function :   getPos()
    * Purpose  :   Returns the current position for the given axis.
    * Input    :   const char *axis
    * Returns  :   double
    *************************************************************************/
    double getPos(const char axis) {
        switch(axis) {
        case 'x':
            return _pos_x;
            break;
        case 'y':
            return _pos_y;
            break;
        case 'z':
            return _pos_z;
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

    std::string getDriverName() {
        return _driver_name;
    }

    std::string &getDriverNameRef() {
        return _driver_name;
    }

    void setDriverName(std::string s) {
        _driver_name = s;
    }

    std::string getDriverPath() {
        return _driver_path;
    }

    std::string &getDriverPathRef() {
        return _driver_path;
    }

    void setDriverPath(std::string s) {
        _driver_path = s;
    }

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

    std::string getUnits() {
        return _units;
    }

    std::string &getUnitsRef() {
        return _units;
    }

    void setUnits(std::string s) {
        _units = s;
    }

    double getTraverseVelocity(void) {
        return _traverse_velocity;
    }

    double &getTraverseVelocityRef(void) {
        return _traverse_velocity;
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
        return _xpos_min;
    }

    double &getXPosMinRef(void) {
        return _xpos_min;
    }

    void setXPosMin(double p) {
        _xpos_min = p;
    }

    double getXPosMax(void) {
        return _xpos_max;
    }

    double &getXPosMaxRef(void) {
        return _xpos_max;
    }

    void setXPosMax(double p) {
        _xpos_max = p;
    }

    double getYPosMin(void) {
        return _ypos_min;
    }

    double &getYPosMinRef(void) {
        return _ypos_min;
    }

    void setYPosMin(double p) {
        _ypos_min = p;
    }

    double getYPosMax(void) {
        return _ypos_max;
    }

    double &getYPosMaxRef(void) {
        return _ypos_max;
    }

    void setYPosMax(double p) {
        _ypos_max = p;
    }

    double getZPosMin(void) {
        return _zpos_min;
    }

    double &getZPosMinRef(void) {
        return _zpos_min;
    }

    void setZPosMin(double p) {
        _zpos_min = p;
    }

    double getZPosMax(void) {
        return _zpos_max;
    }

    double &getZPosMaxRef(void) {
        return _zpos_max;
    }

    void setZPosMax(double p) {
        _zpos_max = p;
    }


    int (*driver_init)(GantryController &gc);
    int (*driver_deinit)(void);
    int (*driver_lconf)(void);
    void (*driver_seterrfunc)(void);

  private:

    std::string _type = "";
    std::string _driver_name = "";
    std::string _driver_path = "";
    std::string _ip_address = "";
    int _port = 0;
    int _timeout = 0;
    int _buffer_size = 0;
    std::string _units = "";
    double _traverse_velocity;
    double _rapid_feed_velocity;

    double _xpos_min = 0.0;
    double _xpos_max = 0.0;
    double _ypos_min = 0.0;
    double _ypos_max = 0.0;
    double _zpos_min = 0.0;
    double _zpos_max = 0.0;

    std::string _defaultDriverLocation = "/home/cdx/catkin_ws/devel/lib/libsmoothie.so";
    void* driver_handle;
    double _pos_x;
    double _pos_y;
    double _pos_z;
    bool _modalSpacePrefix; /* Specifies whether or not the controller
                                     requires a space to be prepended to
                                     line arguments following a modal command */
    unsigned int _id;
    /* Networking configuration */
    char _buffer[NETBUFSIZE];
    double _speed = 0.0;
    /*************************************************************************
    * Function :   initComms()
    * Purpose  :   Initializes the communications bridge to the controller
    * Input    :   void
    * Returns  :   virtual int
    *************************************************************************/
    virtual int initComms(void);

    /*************************************************************************
    * Function :   loadConfig()
    * Purpose  :   Loads the controller parameters from a specified
    *              configuration file
    * Input    :   const std::string file
    * Returns  :   int
    *************************************************************************/
    // int loadConfig(const std::string file = "");

    /*************************************************************************
    * Function :   sendCommand()
    * Purpose  :   Send the G-Code command specified by cmd to the controller.
    * Input    :   const std::string &cmd
    * Returns  :   virtual int
    *************************************************************************/
    virtual int sendCommand(const std::string &s);

    /*************************************************************************
    * Function :   readResponse()
    * Purpose  :   What does this function do?
    * Input    :   void
    * Returns  :   virtual int - number of bytes read.
    *************************************************************************/
    virtual int  readResponse(void);


};
