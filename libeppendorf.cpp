/************************************************************************
Title:    libeppendorf.c
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libeppendorf.c
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libeppendorf.h.

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
#include "libeppendorf.h"


/*********************    CONSTANTS AND MACROS    **********************/
#define CHAR_TRANSMIT_TIME_US 70

/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/

EppendorfModule::EppendorfModule(void) {
    _fd = open(_portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if(_fd < 0) {
        //error_message("error %d opening %s: %s", errno, _portname, strerror(errno));
        return;
    }
    /* set speed to 115, 200 bps, 8n1 (no parity)*/
    set_interface_attribs(_fd, B115200, 0);
    set_blocking(_fd, 0);
}

EppendorfModule::~EppendorfModule(void) {
    close(_fd);
}

void EppendorfModule::sendCommand(std::string &s) {
    memset(_retbuf, 0, sizeof(_retbuf));
    int stat = 0;
    // Wait until pipetter is ready.
    time_t start = time(NULL);
    while(time(NULL) < (start + 1)) {
        write(_fd, "s\r\n", 3);
        usleep((10) * CHAR_TRANSMIT_TIME_US);  // Wait for outgoing chars + received chars
        if(read(_fd, _retbuf, sizeof(_retbuf))) {
            stat = atoi(_retbuf);
            if(stat == 1) {
                write(_fd, s.c_str(), (sizeof(s.c_str())/sizeof(char)));
                usleep((10) * CHAR_TRANSMIT_TIME_US);  // Wait for outgoing chars + received chars
            }
        }
    }
}
void EppendorfModule::ejectTip(void) {
    std::string s = "e\r\n";
    sendCommand(s);
    /*
     * memset(_retbuf, 0, sizeof(_retbuf));
     * int stat = 0;
     * // Wait until pipetter is ready.
     * time_t start = time();
     * while(time() < (start + 1)) {
     *     write(_fd, "s\r\n", 3);
     *     usleep((10) * CHAR_TRANSMIT_TIME_US);  // Wait for outgoing chars + received chars
     *     if(read(_fd, _retbuf, sizeof(_retbuf))) {
     *         stat = atoi(_retbuf);
     *         if(stat == 1) {
     *             write(_fd, "e\r\n", 3);
     *             usleep((10) * CHAR_TRANSMIT_TIME_US);  // Wait for outgoing chars + received chars
     *         }
     *     }
     * }
     */
}

void EppendorfModule::aspirate(double vol) {
    std::string s = "f" + std::to_string(vol);
    sendCommand(s);
}

void EppendorfModule::dispense(double vol) {
    std::string s = "f" + std::to_string(vol);
    sendCommand(s);
}

int EppendorfModule::set_interface_attribs(int fd, int speed, int parity) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if(tcgetattr(fd, &tty) != 0) {
        //error_message("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; //8-bit chars
    /* Disable IGNBRK for mismatched speed tests; otherwise receive break
     * as \000 chars */
    tty.c_iflag &= ~IGNBRK;     // disable break processing
    tty.c_lflag = 0;            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;            // no remapping, no delays
    tty.c_cc[VMIN] = 0;         // read doesn't block
    tty.c_cc[VTIME] = 5;        // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem ctrls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        //error_message("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void EppendorfModule::set_blocking(int fd, int should_block) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0) {
        //error_message("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;    // 0.5 seconds read timeout
    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        //error_message("error %d setting term attributes", errno);
    }
}
