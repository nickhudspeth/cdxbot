/************************************************************************
Title:    libairz.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libairz.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libairz.h.

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

************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include "libairz.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/
int airZModule::init() {
    // set up communications interface
    // start thread to read incoming data
    // send pump initialize command
    sendCommand("Z");
    // enable microliter increment mode

    /* TODO: nam - Check to see if this command requires the run parameter to be
     * disabled.  Fri 19 May 2017 11:01:36 AM MDT */
    sendCommand("N2");

}

int airZModule::deinit() {

}

void airZModule::pickUpTip() {
    /* The air-Z pipetter does not have an z-axis drive independent of the
     * gantry, therefore the pickup motion must be performed by the gantry
     * z-axis drive. We simply enable tip loss monitoring for the duration that
     * the tip is expected to be attached */
    std::string s = "Y";
    sendCommand(s);
}

void airZModule::ejectTip() {
    // eject tip and enable successful tip ejection monitoring
    std::string s = "E0";
    sendCommand(s);
    /* wait for acknowledgement that tip was ejected successfully. If not,
     * stop execution throw a fatal error. */


}

void airZModule::aspirate(float volume) {
    std::string s = "";
    if((_volume + volume) > MICROLITER_INCREMENT_MAX) {
        // print a warning
        // move pipetter to maximum absolute position
        s += "A" + std::to_string(MICROLITER_INCREMENT_MAX);
        _volume = MICROLITER_INCREMENT_MAX;
    } else {
        s += "P" + std::to_string(volume);
        _volume += volume;
    }
    // send aspirate command
    sendCommand(s);
    // get verification [q]
}

void airZModule::dispense(float volume) {
    std::string s = "";
    if((_volume - volume) < 0) {
        // print a warning
        // move pipetter to minimum absolute position
        s += "A0";
        volume = 0;
    } else {
        s += "P" + std::to_string(volume);
        _volume -= volume;
    }
    // send dispense command
    sendCommand(s);
    // get verification [q]
}

void airZModule::sendCommand(std::string s, bool run) {
    s = "/" + std::to_string(_id) + s;
    if(run) s += "R";
}

bool airZModule::liquidLevelDetect(int timeout) {
    /* Activate capacitive liquid level sensing. Wait up to [timeout] seconds
     * for pipetter to return valid indication of liquid level. Terminate and
     * throw error if not found. */
    std::string s = "^" + std::to_string("_capacitance_change_threshold");
    sendCommand(s);
    time_t time_start = time(NULL);
    while((time(NULL) - time_start < timeout)) {
        /* query pump status every 10ms until command has completed and pump
         * has returned to idle state. */
        if(getStatus() == IDLE) return;
        sleep(10000);

    }
}

void airZModule::terminate(void) {
    std::string s = "T";
    sendCommand(s);
}
