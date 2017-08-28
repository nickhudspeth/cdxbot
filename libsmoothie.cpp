/************************************************************************
Title:    libsmoothie.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libsmoothie.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libsmoothie.h.

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
#include "libsmoothie.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   maker()
* Purpose  :   Returns a pointer to a new GantryModule instance
* Input    :   void
* Returns  :   GantryModule*
*************************************************************************/


extern "C" GantryModule *create(void) {
    return new SmoothieModule;
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
// printf("LIBSMOOTHIE::WORKER THREAD: %s\n", s.c_str());
// s.clear();
// std::lock_guard<std::mutex> lock(ready_flag_mutex);
// ready_flag = 1;
// }
// }
// }
// }

extern "C" void *thread_func(void *arg) {
    char buffer[NETBUFSIZE];
    std::memset(buffer, 0, NETBUFSIZE);
    thread_params_t *tp = (thread_params_t *)arg;
    while(1) {
        read(tp->sockfd, buffer, (NETBUFSIZE - 1));
    }
}

void SmoothieModule::waitForOK(void) {
    int n = 0;
    std::string s;
    char buffer[NETBUFSIZE];
    std::memset(buffer, 0, NETBUFSIZE);
    while(1) {
        n = 0;
        std::memset(buffer, 0, NETBUFSIZE);
        if((n = read(_sockfd, buffer, NETBUFSIZE - 1) > 0)) {
            s += std::string(buffer);
            if(s.find("ok") != std::string::npos)  {
                printf("LIBSMOOTHIE::WORKER THREAD: %s\n", s.c_str());
                s.clear();
                // std::lock_guard<std::mutex> lock(ready_flag_mutex);
                return;
            }
        }
    }
}

void SmoothieModule::waitForString(std::string s, unsigned int timeout) {
    int n = 0;
    std::string s2;
    char buffer[NETBUFSIZE];
    std::memset(buffer, 0, NETBUFSIZE);
    time_t start = time(NULL);
    while((time(NULL) - start) < timeout) {
        n = 0;
        std::memset(buffer, 0, NETBUFSIZE);
        if((n = read(_sockfd, buffer, NETBUFSIZE - 1) > 0)) {
            s2 += std::string(buffer);
            if(s2.find(s) != std::string::npos)  {
                printf("LIBSMOOTHIE::WORKER THREAD: %s\n", s2.c_str());
                s2.clear();
                // std::lock_guard<std::mutex> lock(ready_flag_mutex);
                return;
            }
        }
    }
}



SmoothieModule::SmoothieModule() {}
SmoothieModule::~SmoothieModule() {}


int SmoothieModule::init(void) {
    struct sockaddr_in _remote;
    if(_connect == CONN_ETHER) {
        /* Clear out needed memory */
        std::memset(_buffer, 0, NETBUFSIZE);
        std::memset(&_remote, 0, sizeof(struct sockaddr_in));
        /* Fill in required details in the socket structure */
        _remote.sin_family = AF_INET;
        _remote.sin_port = htons(_port);
        _remote.sin_addr.s_addr = inet_addr(_ip_address.c_str());
        /* Create a socket */
        std::cout <<"LIBSMOOTHIE: Connecting to " << _ip_address << ":" << _port << std::endl;
        _sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if(_sockfd < 0) {
            std::cout << "err 1" << std::endl;
            perror("socket");
            return -1;
        }
        std::cout << "sockfd = " << _sockfd <<std::endl;
        /* Set socket options */
        int i = 1;
        setsockopt(_sockfd, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));
        /* Connect to remote host */
        if(connect(_sockfd, (struct sockaddr *) &_remote, sizeof(_remote)) < 0) {
            std::cout << "err 2" << std::endl;
            perror("connect");
            return -1;
        }
        std::cout <<"LIBSMOOTHIE: Connected to " << _ip_address << ":" << _port << std::endl;
    } else if (_connect == CONN_USB) {
        /* Open USB Device */
        _usbfd = open(_usb_addr.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        std::cout << "LIBSMOOTHIE: Opened serial connection to USB device with file descriptor " << _usbfd << std::endl;
        struct termios tty;
        struct termios tty_old;
        if(tcgetattr(_usbfd, &tty) != 0) {
            std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        }
        tty_old = tty;
        /* Set baud rate */
        cfsetospeed(&tty, (speed_t)_usb_baud);
        cfsetispeed(&tty, (speed_t)_usb_baud);

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
        tcflush(_usbfd, TCIFLUSH);
        if(tcsetattr(_usbfd, TCSANOW, &tty) !=0) {
            std::cout << "Error: " << errno << " from tcsetattr." << std::endl;
        }
    }
    /* Create a listener thread */
    _thread_params.sockfd = _sockfd;
    _thread_params.buffer = _buffer;
    _thread_params.timeout = _netTimeoutMS;
    _thread_params.remote = _remote;
    /* Configure attributes for creation of detached state thread. */
    // pthread_attr_t attr;
    // pthread_attr_init(&attr);
    // pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    // pthread_create(&_thread_id, &attr, &thread_func, &thread_params);
    pthread_create(&_thread_id, NULL, &thread_func, &_thread_params);
    // pthread_attr_destroy(&attr);
    usleep(100000);

    home(AXIS_ALL);

    return 0;
}

int SmoothieModule::deinit(void) {
    // if(_connect == CONN_ETHER) {
    // close(_sockfd);
    // } else {
    // close(_usbfd);
    // }
    pthread_cancel(_thread_id);
    pthread_join(_thread_id, NULL);
    printf("LIBSMOOTHIE: Killed worker thread.\n");
    close(_sockfd);
    printf("LIBSMOOTHIE: Closed socket connection to hardware.\n");
    return 0;
    // PRINT_ERROR("LIBSMOOTHIE: Successfully shut down driver.\n");
}

int SmoothieModule::lconf(void) {

}

void SmoothieModule::seterrfunc(void(*ef)(std::string s)) {
    PRINT_ERROR = ef;
}

void SmoothieModule::dwell(int t) {
    std::string ret = "G4 P";
    ret += std::to_string(t);
    sendCommand(ret);
}

void SmoothieModule::emergencyStop(void) {
    std::string ret = "M112";
    sendCommand(ret);
}

void SmoothieModule::emergencyStopReset(void) {
    std::string ret = "M999";
    sendCommand(ret);
}

int SmoothieModule::home(unsigned int axis) {
    std::string ret = "$H";
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


int SmoothieModule::sendCommand(std::string s, bool wfr) {
    s += "\r\n";
    // std::cout << "LIBSMOOTHIE: Sending command: " << s.c_str() << std::endl;
    int n = 0;
    // if(wfr) {
    // while(!ready_flag) {
    // usleep(1000);
    // }
    // }
    if(_connect == CONN_USB) {
        n = write(_sockfd, s.c_str(), sizeof(s.c_str()) - 1);
        usleep((n+25) * 100);
    } else if (_connect == CONN_ETHER) {
        // char buf[256];
        // memset(buf, 0, 256);
        // int len = s.size();
        // sprintf(buf, "%s", s.c_str());
        // n = write(_sockfd, buf, len);
        // fflush((FILE*)(&_sockfd));
        // waitForOK();
        std::string py = "python /home/cdx/catkin_ws/src/cdxbot/sendcommand.py ";
        py += " -i ";
        py += _ip_address;
        py += " -p 23 ";
        py += "\"";
        py += s;
        py += "\"";
        std::cout << " BASH: " << py << std::endl;
        system(py.c_str());

    }
    // if(n < 0) {
    // perror("Error writing to socket.\n");
    // return -1;
    // } else {
    // std::cout << n << " bytes written to device. " << std::endl;
    // }
    // if(wfr) {
    // std::lock_guard<std::mutex> lock(ready_flag_mutex);
    // ready_flag = 0;
    // }
    return n;
}

int SmoothieModule::readResponse(void) {
    int n = 0;
    boost::timer t;
    /* Receive date information and print it */

    /* TODO: nam - Do we need to memset() the buffer again here?
     * Thu 01 Dec 2016 01:59:16 PM MST */
    while((t.elapsed() * 1000) < _netTimeoutMS) {
        if((n = read(_sockfd, &_buffer, NETBUFSIZE - 1)) > 0) {
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

int SmoothieModule::motorsDisable(unsigned int axis) {
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

int SmoothieModule::motorsEnable(void) {
    std::string ret = "M17";
    sendCommand(ret);
    return 0;
}

int SmoothieModule::moveAbsolute(float x, float y, float z) {

    /* TODO: nam - Modify function such that x,y,z destinations less than zero are
     * supported. Tue 28 Mar 2017 11:08:51 AM MDT */
    std::string ret;
    int move_times[] = {0, 0, 0};
    double tvel = _traverse_velocity / 60.0f;
    if (_move_mode != MOVE_MODE_ABSOLUTE) {
        ret = "G90"; // Set absolute mode (modal)
        sendCommand(ret);
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
    ret += std::string(" F") + std::to_string(_traverse_velocity);
    sendCommand(ret);
    // int stime = *std::max_element(move_times, move_times+3);
    // printf("SLEEPING %d MICROSECONDS...\n", stime);
    // usleep(stime);
    return 0;
}

int SmoothieModule::moveRelative(float x, float y, float z) {
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
    sendCommand(ret);
    return 0;
}

int SmoothieModule::setUnits(unsigned int u) {
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

int SmoothieModule::setAxisStepsPerUnit(unsigned int axis, unsigned int steps) {
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
