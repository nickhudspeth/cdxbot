/*************************************************************************
Title:    commands.h - Commands list for CDXBot
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     commands.h
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
#include "CDXBot.h"
#include <boost/function.hpp>
// #include "common.h"
#include <map>
#include <stdio.h>
#include <stdlib.h>

/***********************    FUNCTION PROTOTYPES    ***********************/
extern void aspirateCallback(CDXBot &cd, const struct action a);
extern void dispenseCallback(CDXBot &cd, const struct action a);
extern void ejectCallback(CDXBot &cd, const struct action a);
extern void homeCallback(CDXBot &cd, const struct action a);
extern void mixCallback(CDXBot &cd, const struct action a);
extern void moveCallback(CDXBot &cd, const struct action a);
extern void pauseCallback(CDXBot &cd, const struct action a);
extern void pickupCallback(CDXBot &cd, const struct action a);
extern void pierceCallback(CDXBot &cd, const struct action a);
extern void shakerResetCallback(CDXBot &cd, const struct action a);
extern void shakerSetFreqCallback(CDXBot &cd, const struct action a);
extern void shakerSetPowerCallback(CDXBot &cd, const struct action a);
extern void shakerStartCallback(CDXBot &cd, const struct action a);
extern void shakerStopCallback(CDXBot &cd, const struct action a);
extern void waitCallback(CDXBot &cd, const struct action a);

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
std::map<std::string, boost::function<void(CDXBot &cd, const struct action a)>> command_table = {
    {"aspirate", aspirateCallback},
    {"dispense", dispenseCallback},
    {"eject", ejectCallback},
    {"home", homeCallback},
    {"mix", mixCallback},
    {"move", moveCallback},
    {"pause", pauseCallback},
    {"pickup", pickupCallback},
    {"pierce", pierceCallback},
    {"shakerreset", shakerResetCallback},
    {"shakersetfreq", shakerSetFreqCallback},
    {"shakersetpower", shakerSetPowerCallback},
    {"shakerstart", shakerStartCallback},
    {"shakerstop", shakerStopCallback},
    {"wait", waitCallback}
};


