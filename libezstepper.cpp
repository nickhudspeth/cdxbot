/************************************************************************
Title:    libezstepper.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libezstepper.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libezstepper.h.

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
#include "libezstepper.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/
EZStepperModule::EZStepperModule(unsigned int id) {
    _id = id;
    _USB = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    struct termios tty;
    struct termios tty_old;
    if(tcgetattr(_USB, &tty) != 0) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }
    tty_old = tty;
    /* Set baud rate */
    cfsetospeed(&tty, (speed_t)9600);
    cfsetispeed(&tty, (speed_t)9600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= CREAD | CLOCAL;
    /* Make raw */
    cfmakeraw(&tty);
    /* Flush port then apply attributes */
    tcflush(_USB, TCIFLUSH);
    if(tcsetattr(_USB, TCSANOW, &tty) !=0) {
        std::cout << "Error: " << errno << " from tcsetattr." << std::endl;
    }
}

EZStepperModule::~EZStepperModule(void) {

}

int EZStepperModule::sendCommand(const std::string s) {
    int nbytes = 0;
    std::string cmd = "/" + std::to_string(_id) + s + 'R' + "\r\n";
    nbytes = write(_USB, cmd.c_str(), sizeof(cmd.c_str()) - 1);
    return nbytes;
}

int EZStepperModule::readResponse(void) {
    int n = 0, spot = 0;
    char buf = '\0';
    char ret[1024];
    memset(ret, '\0', sizeof(ret));
    do {
        n = read(_USB, &buf, 1);
        sprintf(&ret[spot], "%c", buf);
        spot += n;

    } while(buf != '\r' && n > 0);
    if(spot > 0) {
        _last_msg.clear();
        _last_msg = std::string(ret);
    }
    return n;
}

int EZStepperModule::moveAbsolute(float pos, bool units) {
    if(abs(pos) > pow(2, 31)) {
        printf("ERROR: Commanded position out of range.\n");
    }
    pos = (units == UNITS_MM) ? pos * _linear_multiplier : pos;
    std::string cmd = "A" + std::to_string(int(round(pos)));
    sendCommand(cmd);
    return 0;
}

int EZStepperModule::moveRelative(float pos, bool units) {
    if(abs(pos) > pow(2, 31)) {
        printf("ERROR: Commanded position out of range.\n");
    }
    std::string cmd;
    if(pos > 0) {
        cmd = "P";
        pos = (units == UNITS_MM) ? abs(pos) * _linear_multiplier : abs(pos);
        sendCommand(cmd + std::to_string(round(pos)));
    } else {
        cmd = "D";
        _current_pos = abs(pos) + 1;
        pos = (units == UNITS_MM) ? abs(pos) * _linear_multiplier : abs(pos);
        float tmp = _current_pos / _linear_multiplier;
        sendCommand(cmd + std::to_string(round(pos)));
        _current_pos = (tmp - (pos / _linear_multiplier));
    }
}

void EZStepperModule::setCurrentPosition(float pos, bool units) {
    if(abs(pos) > pow(2, 31)) {
        printf("ERROR: Commanded position out of range.\n");
    }
    pos = (units == UNITS_MM) ? pos : pos * _linear_multiplier;
    std::string cmd = "z" + std::to_string(pos);
    sendCommand(cmd);
    _current_pos = pos;
}

void EZStepperModule::setHomeFlagPolarity(bool p1, bool p2, bool p3, bool p4) {
    int byte = 0 | (p1 << 3) | (p2 << 2) | (p3 << 1) | p4;
    std::string cmd = "ap" + std::to_string(byte);
    sendCommand(cmd);
}

void EZStepperModule::setPositiveRotationDirection(bool dir) {
    std::string cmd = "F" + std::to_string(dir);
    sendCommand(cmd);
}

void EZStepperModule::setVelocity(int vel) {
    int vout = 0;
    if (vel == -1){
    vout = pow(2, 24);
    } else {
        vout = (abs(vel) > pow(2, 24)) ? pow(2, 24) : abs(vel) * 0.01 * pow(2, 24);
    }
    _current_velocity = vout;
    sendCommand("V" + std::to_string(vout));
}

void EZStepperModule::setAccelerationFactor(unsigned int factor){
    factor = (factor > 65000) ? 65000 : factor; 
    sendCommand("L" + std::to_string(factor));
}

void EZStepperModule::setMaxMoveCurrent(unsigned int cmax){
    cmax = (cmax > 100) ? 100 : cmax;
    _current_move_max = cmax;
    sendCommand("m" + std::to_string(cmax));
}

void EZStepperModule::setMaxHoldCurrent(unsigned int cmax){
    cmax = (cmax > 50) ? 50 : cmax;
    _current_hold_max = cmax;
    sendCommand("h" + std::to_string(cmax));
}

void EZStepperModule::setMicroStepResolution(unsigned int res){
    unsigned int set[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    std::vector<unsigned int> setv(set, set + sizeof(set) / sizeof(set[0]));
    
}
