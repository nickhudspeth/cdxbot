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

extern "C" void *thread_func(void *arg) {
    int n;
    thread_params_t *tp = (thread_params_t *)arg;
    while(1) {
        n = 0;
        std::memset(tp->buffer, 0, NETBUFSIZE);
        if((n = read(tp->sockfd, tp->buffer, NETBUFSIZE - 1) > 0)) {
            tp->buffer[n]= '\0';
            printf("LIBSMOOTHIE::WORKER THREAD: %s\n", tp->buffer);

        }
    }
}

SmoothieModule::SmoothieModule() {}
SmoothieModule::~SmoothieModule() {}

int SmoothieModule::init(void) {
    struct sockaddr_in _remote;
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
    /* Connect to remote host */
    if(connect(_sockfd, (struct sockaddr *) &_remote, sizeof(_remote)) < 0) {
        std::cout << "err 2" << std::endl;
        perror("connect");
        return -1;
    }
    std::cout <<"LIBSMOOTHIE: Connected to " << _ip_address << ":" << _port << std::endl;
    thread_params.sockfd = _sockfd;
    thread_params.buffer = _buffer;
    thread_params.timeout = _netTimeoutMS;
    thread_params.remote = _remote;
    /* Configure attributes for creation of detached state thread. */
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //pthread_create(&thread_id, &attr, &thread_func, &thread_params);
    pthread_attr_destroy(&attr);
    home(AXIS_ALL);
    return 0;
}

int SmoothieModule::deinit(void) {
    close(_sockfd);
    printf("LIBSMOOTHIE: Closed socket connection to hardware.\n");
    pthread_exit(&thread_id);
    printf("LIBSMOOTHIE: Killed worker thread.\n");
    PRINT_ERROR("LIBSMOOTHIE: Successfully shut down driver.\n");
}

int SmoothieModule::lconf(void) {

}

void SmoothieModule::seterrfunc(void(*ef)(std::string s)) {
    PRINT_ERROR = ef;
}

void SmoothieModule::dwell(int t) {
    std::string ret = "G4 P";
    ret += t;
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
    return 0;
}


int SmoothieModule::sendCommand(std::string s) {
    s += "\r\n";
    std::cout << "LIBSMOOTHIE: Sending command: " << s.c_str() << std::endl;
    char buf[128];
    memset(buf, 0, 128);
    int len = s.size();
    sprintf(buf, "%s", s.c_str());
    int n = 0;
    n = write(_sockfd, buf, len);
    if(n < 0) {
        perror("Error writing to socket.\n");
        return -1;
    }
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

    std::string ret = "G90"; // Set absolute mode (modal)
    sendCommand(ret);
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

int SmoothieModule::moveRelative(float x, float y, float z) {
    std::string ret = "G91"; // Set relative mode (modal)
    sendCommand(ret);
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

int SmoothieModule::setAxisStepsPerMM(unsigned int axis, unsigned int steps) {
    std::string ret = "M92";
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
        return -1;
    }
    ret += std::to_string(steps);
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
