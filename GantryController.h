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
        if(loadDriver(_defaultDriverLocation) == 0) {
            ROS_INFO_STREAM("Loaded gantry controller hardware driver from "\
                            << _defaultDriverLocation);

        } else {
            ROS_WARN_STREAM("Could not load gantry controller hardware driver.");
        }
    };

    ~GantryController ();


    /*************************************************************************
    * Function :   loadDriver()
    * Purpose  :   Loads a hardware driver fron the specified location.
    * Input    :   std::string file
    * Returns  :   int
    *************************************************************************/
    int loadDriver(std::string file);


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


    std::string type;
    std::string cominterface;
    std::string ipaddress;
    std::string port;
    std::string timeout;

  private:
    void* _driver_handle;
    int (*_driver_init)(void);
    int (*_driver_deinit)(void);
    int (*_driver_lconf)(void);
    void (*_driver_seterrfunc)(void);
    std::string _defaultDriverLocation = "/home/cdx/catkin_ws/devel/lib/libsmoothie.so";
    double _pos_x;
    double _pos_y;
    double _pos_z;
    bool _modalSpacePrefix; /* Specifies whether or not the controller
                                     requires a space to be prepended to
                                     line arguments following a modal command */
    unsigned int _id;
    std::string _pipetterType = "";
    std::string _pipetterPath = "";
    std::string _controllerType = "";
    std::string _controllerPath = "";
    std::string _comInterface = "";
    std::string _comProtocol = "";
    std::string _type = "";
    std::string _file = "";
    /* Networking configuration */
    int _sockfd = 0;
    std::string _host_ip = "";
    unsigned int _host_port = 0;
    char _buffer[NETBUFSIZE];
    double _netTimeoutMS = 0.0;
    double _speed = 0.0;
    struct sockaddr_in _remote;
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
