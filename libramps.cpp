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

// extern "C" void *thread_func(void *arg) {
// int n = 0;
// thread_params_t *tp = (thread_params_t *)arg;
// char buffer[NETBUFSIZE];
// std::memset(buffer, 0, NETBUFSIZE);
// std::string s;
// while(1) {
// n = 0;
// std::memset(buffer, 0, NETBUFSIZE);
// if((n = read(tp->sockfd, buffer, NETBUFSIZE - 1) > 0)) {
// s += std::string(buffer);
// if(s.find("ok") != std::string::npos)  {
// printf("LIBRAMPS::WORKER THREAD: %s\n", s.c_str());
// s.clear();
// std::lock_guard<std::mutex> lock(ready_flag_mutex);
// ready_flag = 1;
// }
// }
// }
// }

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
                printf("%s%s%s\n", KBLU, s.c_str(), KNRM);
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
    while(1) {
        n = 0;
        std::memset(buffer, 0, NETBUFSIZE);
        if((n = read(_usbfd, buffer, NETBUFSIZE - 1) > 0)) {
            s += std::string(buffer);
            if(s.find("ok") != std::string::npos)  {
                printf("LIBRAMPS::WORKER THREAD: %s\n", s.c_str());
                s.clear();
                return;
            }
        }
    }
}

void RampsModule::waitForString(std::string s, unsigned int timeout) {
    int n = 0;
    std::string s2;
    char buffer[NETBUFSIZE];
    std::memset(buffer, 0, NETBUFSIZE);
    time_t start = time(NULL);
    while((time(NULL) - start) < timeout) {
        write(_usbfd, "M114\n", 5);
        n = 0;
        std::memset(buffer, 0, NETBUFSIZE);
        if((n = read(_usbfd, buffer, NETBUFSIZE - 1) > 0)) {
            s2 += std::string(buffer);
            if(s2.find(s) != std::string::npos)  {
                printf("LIBRAMPS::WORKER THREAD: %s\n", s2.c_str());
                s2.clear();
                return;
            }
        }
        usleep(100000);
    }
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
    std::cout << "Opened serial connection to USB device with file descriptor " << _usbfd << std::endl;
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
    pthread_cancel(_thread_id);
    pthread_join(_thread_id, NULL);
    printf("LIBRAMPS: Killed worker thread.\n");
    close(_usbfd);
    printf("LIBRAMPS: Closed socket connection to hardware.\n");
    return 0;
    // PRINT_ERROR("LIBRAMPS: Successfully shut down driver.\n");
}

int RampsModule::lconf(void) {

}

void RampsModule::seterrfunc(void(*ef)(std::string s)) {
    PRINT_ERROR = ef;
}

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

int RampsModule::home(unsigned int axis) {
    std::string ret = "G28";
    switch (axis) {
    case AXIS_ALL:
        break;
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
        /* Jumps here if no argument is given. */
        break;
    }
    sendCommand(ret);
    _pos[0] =  0;
    _pos[1] = 0;
    _pos[2] = 0;
    return 0;
}


int RampsModule::sendCommand(std::string s, bool wfr) {
    s += "\n";
    int n = 0;
    // n = write(_usbfd, s.c_str(), sizeof(s.c_str()) - 1);
    n = write(_usbfd, s.c_str(), s.length());
    std::cout << "LIBRAMPS: Sent " << n << " characters in command: " << s.c_str() << std::endl;
    // if(wfr) {
    waitForString("Count");
    // }
    usleep((n+25) * 100);
    // usleep(10000);
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

int RampsModule::moveAbsolute(float x, float y, float z) {

    /* TODO: nam - Modify function such that x,y,z destinations less than zero are
     * supported. Tue 28 Mar 2017 11:08:51 AM MDT */
    std::string ret;
    int move_times[] = {0, 0, 0};
    double tvel = _traverse_velocity / 60.0f;
    if (_move_mode != MOVE_MODE_ABSOLUTE) {
        ret = "G90"; // Set absolute mode (modal)
        sendCommand(ret, 1);
        _move_mode = MOVE_MODE_ABSOLUTE;
    }
    ret = std::string("G0");
    if(x > 0) {
        ret += std::string(" X") + std::to_string(x);
        double diff = std::abs(x - _pos[0]);
        move_times[0] = static_cast<int>((diff*1000000 / tvel) + 0.5f);
        _pos[0] = x;
    }
    if(y > 0) {
        ret += std::string(" Y") + std::to_string(y);
        double diff = std::abs(y - _pos[1]);
        move_times[1] = static_cast<int>((diff*1000000 / tvel) + 0.5f);
        _pos[1] = y;
    }
    if(z > 0) {
        ret += std::string(" Z") + std::to_string(z);
        double diff = std::abs(z - _pos[2]);
        move_times[2] = static_cast<int>((diff*1000000 / tvel) + 0.5f);
        _pos[2] = z;
    }
    // ret += std::string(" F") + std::to_string(_traverse_velocity);
    sendCommand(ret, 1);
    // int stime = *std::max_element(move_times, move_times+3);
    // printf("SLEEPING %d MICROSECONDS...\n", stime);
    // usleep(stime);
    return 0;
}

int RampsModule::moveRelative(float x, float y, float z) {
    std::string ret;
    if (_move_mode != MOVE_MODE_RELATIVE) {
        ret = "G91"; // Set absolute mode (modal)
        sendCommand(ret);
        _move_mode = MOVE_MODE_RELATIVE;
    }
    ret = std::string("G0");
    if(x > 0) {
        ret += std::string(" X") + std::to_string(x);
    }
    if(y > 0) {
        ret += std::string(" Y") + std::to_string(y);
    }
    if(z > 0) {
        ret += std::string(" Z") + std::to_string(z);
    }
    ret += std::string(" F") + std::to_string(_traverse_velocity);
    sendCommand(ret, 1);
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
    return 0;
}



// int loadConfig(const std::string file) {
// cfg_t *cfg;
// cfg_opt_t linearStageOpts[] = {
// CFG_FLOAT((char *)"steps_per_revolution", 0.0, CFGF_NONE),
// CFG_FLOAT((char *)"leadscrew_pitch", 0.0, CFGF_NONE),
// CFG_FLOAT((char *)"limit_min", 0.0, CFGF_NONE),
// CFG_FLOAT((char *)"limit_max", 0.0, CFGF_NONE),
// CFG_END(),
// };
// cfg_opt_t opts[] = {
// CFG_INT((char *)"id", 1, CFGF_NONE),
// CFG_STR((char *)"pipetter_type",(char *) "NONE", CFGF_NONE),
// CFG_STR((char *)"pipetter_path", (char *)"NONE", CFGF_NONE),
// CFG_STR((char *)"controller_type", (char *)"NONE", CFGF_NONE),
// CFG_STR((char *)"controller_interface", (char *)"NONE", CFGF_NONE),
// CFG_STR((char *)"controller_IP", (char *)"NONE", CFGF_NONE),
// CFG_INT((char *)"controller_port", 0, CFGF_NONE),
// CFG_INT((char *)"controller_net_recv_timeout", 0, CFGF_NONE),
// CFG_STR((char *)"controller_path", (char *)"NONE", CFGF_NONE),
// CFG_STR((char *)"controller_protocol", (char *)"NONE", CFGF_NONE),
// CFG_SEC((char *)"linear_stage", linearStageOpts, CFGF_MULTI | CFGF_TITLE),
// CFG_END(),
// };
// cfg = cfg_init(opts, CFGF_NOCASE);
// if(cfg_parse(cfg, file.c_str()) == CFG_PARSE_ERROR) {
// perror("Error parsing config file.\n");
// return -1;
// }
// _id = cfg_getint(cfg, "id");
// _pipetterType = cfg_getstr(cfg, "pipetter_type");
// std::cout << "pipetter type = " << _pipetterType << std::endl;
// _pipetterPath = cfg_getstr(cfg, "pipetter_path");
// std::cout << "pipetter path = " << _pipetterPath << std::endl;
// _controllerType = cfg_getstr(cfg, "controller_type");
// std::cout << "controller type = " << _controllerType << std::endl;
// _controllerPath = cfg_getstr(cfg, "controller_path");
// std::cout << "controller path = " << _controllerPath << std::endl;
// _comInterface = cfg_getstr(cfg, "controller_interface");
// std::cout << "comms interface = " << _comInterface << std::endl;
// _comProtocol = cfg_getstr(cfg, "controller_protocol");
// std::cout << "controller protocol = " << _comProtocol << std::endl;
// _host_ip  = cfg_getstr(cfg, "controller_IP");
// std::cout << "host IP = " << _host_ip << std::endl;
// _host_port = cfg_getint(cfg, "controller_port");
// std::cout << "host port = " << _host_port << std::endl;
// _netTimeoutMS = cfg_getint(cfg, "controller_net_recv_timeout");
// std::cout << "network timeout = " << _netTimeoutMS << "ms" << std::endl;

// int n = cfg_size(cfg, "linear_stage");
// std::cout << "Machine configured with " << n << " linear stages." << std::endl;
// for (unsigned int i = 0; i < n; i++) {
// cfg_t *ls = cfg_getnsec(cfg, "linear_stage", i);
// std::cout << "Axis " << cfg_title(ls) << std::endl;
// std::cout << "Steps per revolution: " << \
// cfg_getfloat(ls,"steps_per_revolution") << std::endl;
// }
// cfg_free(cfg);
// return 0;
// }
