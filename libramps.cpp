/************************************************************************
Title:    libramps.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libramps.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libramps.h.

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
#include "libramps.h"


/*********************    CONSTANTS AND MACROS    **********************/
#define KBLU  "\x1B[34m""]"
#define KNRM  "\x1B[0m""]"
/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   maker()
* Purpose  :   Returns a pointer to a new GantryModule instance
* Input    :   void
* Returns  :   GantryModule*
*************************************************************************/


extern "C" GantryModule *create(void) {
    return new RampsModule;
}

extern "C" void destroy(GantryModule *gc) {
    delete gc;
}

extern "C" void *thread_func(void *arg) {
    int n = 0;
    std::string s;
    char buffer[NETBUFSIZE];
    std::memset(buffer, 0, NETBUFSIZE);
    thread_params_t *tp = (thread_params_t *)arg;
    while(1) {
        n = 0;
        std::memset(buffer, 0, NETBUFSIZE);
        if((n = read(tp->usbfd, buffer, NETBUFSIZE - 1) > 0)) {
            s += std::string(buffer);
            if(s.find("\n") != std::string::npos)  {
                // printf("%s%s%s\n", KBLU, s.c_str(), KNRM);
                s.clear();
            }
        }
    }
}

void RampsModule::waitForOK(void) {
    int n = 0;
    std::string s;
    char buffer[NETBUFSIZE];
    std::memset(buffer, 0, NETBUFSIZE);
    time_t start = time(NULL);
    while((time(NULL) - start) < 15) {
        n = 0;
        std::memset(buffer, 0, NETBUFSIZE);
        if((n = read(_usbfd, buffer, NETBUFSIZE - 1) > 0)) {
            s += std::string(buffer);
            if(s.find("ok") != std::string::npos)  {
#ifdef PRINT_OUTPUT
                printf("LIBRAMPS::WORKER THREAD: %s\n", s.c_str());
#endif
                s.clear();
                return;
            }
        }
    }
    return;
}

void RampsModule::waitForString(std::string s, unsigned int timeout) {
    int n = 0;
    std::string s2 = "";
    char buffer[NETBUFSIZE] = { 0 };
    time_t start = time(NULL);
    write(_usbfd, "M114\n", 5);

    /* Try to find string for [timeout] secomds */
    while((time(NULL) - start) < timeout) {
        std::memset(buffer, 0, NETBUFSIZE);
        /* If there is a character available in the read buffer, append it to
         * string s2.*/
        if(read(_usbfd, buffer, 1) == 1) {
            s2 += std::string(buffer);
            if(buffer[0] == '\n') {
#ifdef PRINT_OUTPUT
                // std::cout << KBLU << "LIBRAMPS::WORKER THREAD: Newline found. S2 = \"" << s2 << KNRM << std::endl;
#endif
                if (s2.find(s) != std::string::npos) {
#ifdef PRINT_OUTPUT
                    // std::cout << KBLU << "LIBRAMPS::WORKER THREAD: FOUND STRING: " << s2 << KNRM << std::endl;
#endif
                    s2.clear();
                    return;
                } else {
                    s2.clear();
                }
            }
        }
    }
}

bool RampsModule::verifyPosition(unsigned int axes, double x, double y, double z, unsigned int timeout) {
    bool ax = false, ay = false, az = false;
    if(!(axes | AXIS_X)) {
        ax = true;
    }
    if(!(axes | AXIS_Y)) {
        ay = true;
    }
    if(!(axes | AXIS_Z)) {
        az = true;
    }
    int n = 0;
    std::string s2 = "";
    int idx;
    char buffer[NETBUFSIZE] = { 0 };
    std::string xdec = std::to_string(x);
    idx = xdec.find(".");
    std::string xpos = "X:" + xdec.substr(0, idx+2);

    std::string ydec = std::to_string(y);
    idx = ydec.find(".");
    std::string ypos = "Y:" + ydec.substr(0, idx+2);

    std::string zdec = std::to_string(z);
    idx = zdec.find(".");
    std::string zpos = "Z:" + zdec.substr(0, idx+2);
    PRINT_DEBUG("LIBRAMPS: Waiting for gantry to arrive at position [" + \
                xpos + ", " + ypos + ", " + \
                zpos + "]");
    time_t start = time(NULL);
    time_t start2 = start;
    // auto t1 = std::chrono::high_resolution_clock::now();

    /* Try to find string for [timeout] secomds */
    while((time(NULL) - start) < timeout) {
        /* Issue M114 command every 1000 milliseconds */
        // auto t2 = std::chrono::high_resolution_clock::now();
        // if(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() > 1000) {
            // write(_usbfd, "M114\n", 5);
            // t1 = std::chrono::high_resolution_clock::now();
            // usleep(20000);
        // }
        if((time(NULL) - start2) > 0){
            PRINT_DEBUG("LIBRAMPS: Sending M114");
            write(_usbfd, "M114\n", 5);
            start2 = time(NULL);
            usleep(20000);
        }
        std::memset(buffer, 0, NETBUFSIZE);
        /* If there is a character available in the read buffer, append it to
         * string s2.*/
        if(read(_usbfd, buffer, 1) == 1) {
            s2 += std::string(buffer);
            /* Check to see if this marks the end of one line.*/
            if(buffer[0] == '\n') {
                PRINT_DEBUG("LIBRAMPS::Newline found. S2 = " + s2);
                /* If this line contains coordinate information, parse it.
                 * Else, throw it away. */
                if (s2.find(std::string("X:")) != std::string::npos) {
                    // PRINT_DEBUG("LIBRAMPS::FOUND STRING: " + s2);
                    if(axes & AXIS_X) {
                        if((s2.find(xpos) != std::string::npos) && ax == false) {
                            PRINT_DEBUG("LIBRAMPS: Gantry reached target in x-coordinate");
                            ax = true;
                        }
                    } else {
                        ax = true;
                    }
                    if(axes & AXIS_Y) {
                        if((s2.find(ypos) != std::string::npos) && ay == false) {
                            PRINT_DEBUG("LIBRAMPS: Gantry reached target in y-coordinate");
                            ay = true;
                        }
                    } else {
                        ay = true;
                    }
                    if(axes & AXIS_Z) {
                        if((s2.find(zpos) != std::string::npos) && az == false) {
                            PRINT_DEBUG("LIBRAAMPS: Gantry reached target in z-coordinate");
                            az = true;
                        }
                    } else {
                        az = true;
                    }
                    if((ax == true) && (ay == true) && (az ==true)) {
                        PRINT_DEBUG("LIBRAMPS:: Gantry move complete.");
                        s2.clear();
                        return true;
                    } else {
                        s2.clear();
                        // std::cout << KBLU << s2 << KNRM << std::endl;
                    }
                } else {
                    s2.clear();
                }
            }
        }
    }
    return false;
}

RampsModule::RampsModule() {}
RampsModule::~RampsModule() {}


int RampsModule::init(void) {
    /* Open USB Device */
    _usbfd = open(_usb_addr.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if(_usbfd < 0) {
        std::cout << __FILE__ << " " <<__PRETTY_FUNCTION__ << " ERROR: Code " << errno << \
                  " opening " << _usb_addr << " - " << strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "LIBRAMPS: Opened serial connection to USB device at " << _usb_addr << " with file descriptor " << _usbfd << std::endl;
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(_usbfd, &tty) != 0) {
        std::cout << __FILE__ << " " << __PRETTY_FUNCTION__ << " ERROR: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return -1;
    }
    /* Set baud rate */
    cfsetospeed(&tty, _usb_baud);
    cfsetispeed(&tty, _usb_baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= 0; // No parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if(tcsetattr(_usbfd, TCSANOW, &tty) != 0) {
        std::cout << __FILE__ << " " << __PRETTY_FUNCTION__ << " ERROR: " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        return -1;
    }
    /* Set serial port to non-blocking */
    // memset(&tty, 0, sizeof tty);
    // if(tcgetattr(_usbfd, &tty) != 0){
    // std::cout << __FILE__ << __PRETTY_FUNCTION__ << " ERROR: " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    // return -1;
    // }
    // tty.c_cc[VMIN] = 0;
    // tty.c_cc[VTIME] = 5;

    /* Create a listener thread */
    _thread_params.usbfd = _usbfd;
    _thread_params.buffer = _buffer;
    _thread_params.timeout = _netTimeoutMS;
    /* Configure attributes for creation of detached state thread. */
// pthread_attr_t attr;
// pthread_attr_init(&attr);
// pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
// pthread_create(&_thread_id, &attr, &thread_func, &thread_params);
    // pthread_create(&_thread_id, NULL, &thread_func, &_thread_params);
// pthread_attr_destroy(&attr);

    // home(AXIS_ALL);
    return 0;
}

int RampsModule::deinit(void) {
    // pthread_cancel(_thread_id);
    // pthread_join(_thread_id, NULL);
    printf("LIBRAMPS: Killed worker thread.\n");
    close(_usbfd);
    printf("LIBRAMPS: Closed socket connection to hardware.\n");
    return 0;
    // PRINT_ERROR("LIBRAMPS: Successfully shut down driver.\n");
}

int RampsModule::lconf(void) {

}

// void RampsModule::seterrfunc(void(*ef)(std::string s)) {
// PRINT_ERROR = ef;
// }

void RampsModule::dwell(int t) {
    std::string ret = "G4 P";
    ret += std::to_string(t);
    sendCommand(ret);
}

void RampsModule::emergencyStop(void) {
    std::string ret = "M112";
    sendCommand(ret);
}

void RampsModule::emergencyStopReset(void) {
    std::string ret = "M999";
    sendCommand(ret);
}

bool RampsModule::home(unsigned int axis) {
    std::string ret = "G28";
    switch (axis) {
    case AXIS_ALL:
        break;
    case AXIS_X:
        ret += "X";
        break;
    case AXIS_Y:
        ret += "Y";
        break;
    case AXIS_Z:
        ret += "Z";
        break;
    default:
        /* Jumps here if no argument is given. */
        break;
    }
    sendCommand(ret);
    return verifyPosition((AXIS_X | AXIS_Y), 0, 406.5, 0, 20);
}


int RampsModule::sendCommand(std::string s, bool wfr) {
    s += "\n";
    int n = 0;
    n = write(_usbfd, s.c_str(), s.length());
    PRINT_DEBUG("LIBRAMPS: Sent " + std::to_string(n) + " characters in command: " + s);
    usleep((n+25) * 100);
    usleep(100000);
    return n;
}

int RampsModule::readResponse(void) {
    int n = 0;
    boost::timer t;
    /* Receive date information and print it */

    /* TODO: nam - Do we need to memset() the buffer again here?
     * Thu 01 Dec 2016 01:59:16 PM MST */
    while((t.elapsed() * 1000) < _netTimeoutMS) {
        if((n = read(_usbfd, &_buffer, NETBUFSIZE - 1)) > 0) {
            _buffer[n] = '\0'; // Null terminate the string
            printf("%s", _buffer);
            break;
        }
        if(n < 0) {
            /* Socket read error */
            perror("read");
            break;
        }
    }
    return n;
}

int RampsModule::motorsDisable(unsigned int axis) {
    std::string ret = "M18";
    switch (axis) {
    case AXIS_X:
        ret += " X";
        break;
    case AXIS_Y:
        ret += " Y";
        break;
    case AXIS_Z:
        ret += " Z";
        break;
    default:
        /* Jumps here if no argument is given */
        break;
    }
    sendCommand(ret);
    return 0;
}

int RampsModule::motorsEnable(void) {
    std::string ret = "M17";
    sendCommand(ret);
    return 0;
}

int RampsModule::moveAbsolute(float x, float y, float z, bool movex, bool movey, bool movez) {
    std::string ret = "";
    double diff = 0.0f;
    int move_times[] = {0, 0, 0};
    double tvel = _traverse_velocity / 60.0f;
    if (_move_mode != MOVE_MODE_ABSOLUTE) {
        ret = "G90"; // Set absolute mode (modal)
        sendCommand(ret, 1);
        _move_mode = MOVE_MODE_ABSOLUTE;
    }
    ret = std::string("G0");
    if(movex) {
        ret += std::string(" X") + std::to_string(x);
        _pos[0] = x;
    }
    if(movey) {
        ret += std::string(" Y") + std::to_string(y);
        _pos[1] = y;
    }
    if(movez) {
        ret += std::string(" Z") + std::to_string(z);
        _pos[2] = z;
    }
    ret += std::string(" F") + std::to_string(_traverse_velocity);
    sendCommand(ret, 1);
    verifyPosition((AXIS_X | AXIS_Y), x,y,z);
    return 0;
}

int RampsModule::moveRelative(float x, float y, float z, bool movex, bool movey, bool movez) {
    std::string ret;
    if (_move_mode != MOVE_MODE_RELATIVE) {
        ret = "G91"; // Set absolute mode (modal)
        sendCommand(ret);
        _move_mode = MOVE_MODE_RELATIVE;
    }
    ret = std::string("G0");
    if(movex) {
        ret += std::string(" X") + std::to_string(x);
        _pos[0] += x;
    }
    if(movey) {
        ret += std::string(" Y") + std::to_string(y);
        _pos[1] += y;
    }
    if(movez) {
        ret += std::string(" Z") + std::to_string(z);
        _pos[2] += z;
    }
    ret += std::string(" F") + std::to_string(_traverse_velocity);
    sendCommand(ret, 1);
    verifyPosition(x,y,z);
    return 0;
}

int RampsModule::setUnits(unsigned int u) {
    std::string ret;
    switch(u) {
    case UNITS_MM:
        ret =  "G21";
        break;
    case UNITS_IN:
        ret = "G20";
        break;
    default:
        break;
    }
    sendCommand(ret);
    return 0;
}

int RampsModule::setAxisStepsPerUnit(unsigned int axis, unsigned int steps) {
    std::string ret = "M92";
    switch (axis) {
    case AXIS_X:
        ret += " X";
        ret += std::to_string(steps);
        _steps_per_unit[0] = steps;
        break;
    case AXIS_Y:
        ret += " Y";
        ret += std::to_string(steps);
        _steps_per_unit[1] = steps;
        break;
    case AXIS_Z:
        ret += " Z";
        ret += std::to_string(steps);
        _steps_per_unit[2] = steps;
        break;
    case AXIS_ALL:
        _steps_per_unit[0] = steps;
        _steps_per_unit[1] = steps;
        _steps_per_unit[2] = steps;
        ret += " X" + std::to_string(steps);
        ret += " Y" + std::to_string(steps);
        ret += " Z" + std::to_string(steps);
        break;
    default:
        return -1;
    }
    sendCommand(ret);
    return 0;
}
