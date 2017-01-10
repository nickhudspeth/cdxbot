/*************************************************************************
Title:    PipetterController.h -
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     PipetterController.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:


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

*************************************************************************/
#pragma once

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <dlfcn.h>
#include <ros/ros.h>
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
class PipetterController {
  public:
    PipetterController () {
        if(loadDriver(_defaultDriverLocation) == 0) {
            ROS_INFO_STREAM("Loaded pipetter hardware driver from "\
                            << _defaultDriverLocation);

        } else {
            ROS_WARN_STREAM("Could not load pipetter hardware driver.");
        }
    };
    virtual ~PipetterController () {};

    /*************************************************************************
    * Function :   loadDriver()
    * Purpose  :   Loads a gantry controller driver from the file specified.
    * Input    :   std::string file
    * Returns  :   int
    *************************************************************************/
    int loadDriver(std::string file);

    /*************************************************************************
    * Function :   getZPos()
    * Purpose  :   What does this function do?
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getZPos(void);

    std::string type;
    std::string driver_name;
    std::string driver_path;

  private:
    void* _driver_handle;
    int (*_driver_init)(void);
    int (*_driver_deinit)(void);
    int (*_driver_lconf)(void);
    void (*_driver_seterrfunc)(void);
    std::string _defaultDriverLocation = "/home/cdx/catkin_ws/devel/lib/libzeus.so";

    /* data */
};

/***********************    FUNCTION PROTOTYPES    ***********************/
