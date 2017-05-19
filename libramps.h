/*************************************************************************
Title:    libramps.h - CDXBot Gantry Controller Driver for RAMPS 1.4 board
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libramps.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:
    2017-01-04: The driver currently only supports ethernet communications.
                If USB support is to be added, separate instructions will
                have to be added in init(), deinit(), and sendCommand().

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
#include <algorithm>
#include <arpa/inet.h>
#include <boost/timer.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include "GantryModule.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define NETBUFSIZE 128
#define UNITS_MM 0

typedef struct {
    int usbfd;
    char *buffer;
    double timeout = 0.0;
    // struct sockaddr_in remote;
} thread_params_t;


bool ready_flag = 1;
// std::mutex ready_flag_mutex;
/* The 'extern "C"' keyword must be used here to force the compiler to use
 * C rather than C++ linkage. Otherwise, the compiler mangles the symbol
 * name and causes dlsym to not be able to locate any symbols in the library.*/
extern "C" {
    class RampsModule : public GantryModule {
      public:
        RampsModule (void);
        virtual ~RampsModule ();
        int init(void);
        int deinit(void);
        int lconf(void);
        void seterrfunc(void(*ef)(std::string s));
        void dwell(int t);
        void emergencyStop(void);
        void emergencyStopReset(void);
        int home(unsigned int axis);
        int motorsDisable(unsigned int axis);
        int motorsEnable(void);
        int moveAbsolute(float x, float y, float z);
        int moveRelative(float x, float y, float z);
        int setUnits(unsigned int u = UNITS_MM);
        int setAxisStepsPerUnit(unsigned int axis, unsigned int steps);
        int getSocket(void) {
            return _usbfd;
        }
        char *getBuffer(void) {
            return _buffer;
        }
        bool getReadyFlag(void) {
            return _ready_flag;
        }
        bool setReadyFlag(bool f) {
            _ready_flag = f;
        }
      private:
        int sendCommand(std::string s, bool wfr = 0);
        int readResponse(void);
        void waitForOK(void);
        void waitForString(std::string s, unsigned int timeout = 10);
        bool _units = UNITS_MM;
        /* Networking configuration */
        std::string _usb_addr = "/dev/ttyUSB0";
        int _usb_baud = B115200;
        int _usbfd = 0;
        char _buffer[NETBUFSIZE];
        double _netTimeoutMS = 0.0;
        double _pos[3] = {0, 0, 0};
        bool _ready_flag = 1;
        pthread_t _thread_id;
        thread_params_t _thread_params;
        // struct sockaddr_in _remote;
    };

}

/***********************    FUNCTION PROTOTYPES    ***********************/
