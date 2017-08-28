/*************************************************************************
Title:    libeppendorf.h - CDXBot driver for eppendorf hybrid pipetter
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libeppendorf.h
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
#pragma once

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <boost/format.hpp>
#include <iomanip>
#include <sstream>
#include "PipetterModule.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
class EppendorfModule : PipetterModule {
  public:
    EppendorfModule();
    ~EppendorfModule();
    bool ejectTip(void);
    bool aspirate(double vol);
    bool dispense(double vol);
  private:
    int _fd = -1;
    int _speed = 115200;
    char _retbuf[32];
    int retcnt;
    std::string _portname = "/dev/ttyACM0";
    int set_interface_attribs(int fd, int speed, int parity);
    void set_blocking(int fd, int should_block);
    void waitForStatus(void);
    /* data */
};

/***********************    FUNCTION PROTOTYPES    ***********************/

