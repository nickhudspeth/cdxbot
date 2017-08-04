/*************************************************************************
Title:    ShakerModule.h - Interface Class for CDXBot Shaker Drivers
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     ShakerModule.h
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


/***********************    FUNCTION PROTOTYPES    ***********************/
class ShakerModule : public CDXModule {
  public:
    ShakerModule (void) {};
    virtual ~ShakerModule (void) {};
    virtual bool reset(void){};
    virtual bool start(void) {};
    virtual bool stop(void) {};
    virtual bool setFrequency(unsigned int f) {};
    virtual bool setPower(float percent) {};

    std::string type;
    std::string driver_name;
    std::string driver_path;

  protected:
    double freq;
};


typedef ShakerModule *create_t();
typedef void destroy_t(ShakerModule *);
