/*************************************************************************
Title:    common.h - CDXBOT common definitions
Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
File:     common.h
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
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/

#define AXIS_X   1
#define AXIS_Y   2
#define AXIS_Z   3

#define UNITS_MM 0
#define UNITS_IN 1

#define COM_ETHERNET 0
#define COM_USB      1
#define COM_TELNET   0

#define NETBUFSIZE 64

#define TO_MM(x) x*25.40
#define TO_IN(x) x/25.40

/***********************    FUNCTION PROTOTYPES    ***********************/


