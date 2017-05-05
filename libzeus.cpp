/************************************************************************
Title:    libzeus.cpp - C++ Wrapper for Hamilton Zeus Pipetter Python Driver
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libzeus.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libzeus.h.

LICENSE:
    Copyright (C) 2016 Nicholas Morrow

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
#include "libzeus.h"
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   maker()
* Purpose  :   Returns a pointer to a new Module instance
* Input    :   void
* Returns  :   PiModule*
*************************************************************************/


extern "C" PipetterModule *create(void) {
    return new ZeusModule;
}

extern "C" void destroy(PipetterModule *gc) {
    delete gc;
}

extern "C" void *thread_func(void *arg) {
    char retbuf[8];
    // printf("Entered thread function.\n");
    /* Check to see if there is any incoming data on the CAN bus.
     * If so, push the data to a FIFO.*/
    struct can_frame ret;
    int n = 0;  // Number of received bytes
    ZeusModule *zm = static_cast<ZeusModule *>(arg);
    std::string msg;
    int soc = zm->getSockFD();
    // printf("ZM socket filedescriptor = %d\n", soc);
    zm->_read_can_port = 1;
    memset(&ret, 0, sizeof(struct can_frame));
    while(zm->_read_can_port) {
        struct timeval timeout = {1,0};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(soc, &readSet);
        if(select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0) {
            if(!(zm->_read_can_port)) {
                break;
            }
            if(FD_ISSET(soc, &readSet)) {
                memset(&ret, 0, sizeof(struct can_frame));
                n = read(soc, &ret, sizeof(struct can_frame));
                if(n != 0) {
                    if(n < 0) {
                        printf("LIBZEUS: Error reading from CAN socket.\n");
                    } else if(n < sizeof(struct can_frame)) {
                        printf("LIBZEUS: Incomplete frame read from CAN socket.\n");
                    } else {
                        if(ret.can_id & KICK_MASK) {
                            /* Received kick frame. */
                            //zm->setKickFlag(1);
                            zm->sendRemoteFrame(1);
                            // printf("Received kick frame with byte %04X.\n", ret.data[7]);
                            // } else if(!strcmp((const char*)ret.data,"")) {
                        } else if(ret.can_id & CAN_RTR_FLAG) {
                            /* Data field is empty, but this is not a kick frame.
                             *  This must be  a remote request.*/
                            // printf("Received remote request with byte %04X.\n", ret.data[7]);
                            zm->setRemoteFlag(1);
                        } else {
                            // printf("Received data frame with byte %04X\n", ret.data[7]);
                            memset(&retbuf, 0, sizeof(retbuf));
                            snprintf(retbuf, 7,"%s", ret.data);
                            msg.append(retbuf);
                            // printf("Received message: %s\n", msg.c_str());

                            /*If frame data contains EOM flag, process the returned string. */
                            if(ret.data[7] & EOM_MASK) {
                                zm->setReceivedMsg(msg);
                                zm->setMsgReadyFlag(1);
                                zm->setWaitingForMsgFlag(0);
                                std::string err_msg = zm->parseErrors(msg);
                                if(PRINT_OUTPUT) {
                                    printf("Received message: %s -> ", msg.c_str());
                                    printf("%s\n", err_msg.c_str());
                                }
                                msg.clear();
                            } else {
                                zm->sendRemoteFrame(8);
                            }
                            // printf("Read a frame from da socket with DLC %d\n and data %s\n", ret.can_dlc, ret.data);
                        }
                        // (zm->getFIFO()).push(ret);
                        // for(int i = 0; i < ret.can_dlc; i++) {
                        // printf("%d", ret.data[i]);
                        // msg.append(reinterpret_cast<const char*>(ret.data[i]));
                        // }
                        // printf("\n");
                    }
                }
            }
        }
        if(zm->_error_flag) {
            printf("Error flag set. Getting last faulty parameter.\n");
            zm->getLastFaultyParameter();
            zm->_error_flag = 0;
        }
    }
    printf("Exiting thread loop.\n");
    return 0;
}


extern "C" std::string zfill(std::string s, int len) {
    if(!(s.length() < len)) {
        return s;
    }
    s.insert(s.begin(), len - s.length(), '0');
    return s;
}



ZeusModule::ZeusModule(int id) {
}
ZeusModule::~ZeusModule() {
}


/* TODO: nam - Is there a good reason not to move this to the class
 * constructor? Mon 06 Mar 2017 11:06:21 AM MST */

int ZeusModule::init(void) {
    /* TODO: nam - Start thread_func() here using pthread_create().
     * Mon 06 Mar 2017 11:07:02 AM MST */
    // _id = id;
    struct can_frame f;
    memset(&f, 0, sizeof(struct can_frame));
    setLastFrame(f);
    // if(pthread_mutex_init(&_lock_msg, NULL) != 0) {
        // printf("ERROR: LibZeus: Could not create mutex.\n");
        // Unable to create mutex. Throw error.
    // }
    initCANBus();
    pthread_create(&_thread_id, NULL, &thread_func, this);
    // printf("Thread function created.\n");
    initZDrive();
    initDosingDrive();
    // getInitializationStatus();
    // getFirmwareVersion();
    struct deck_geometry_t d;
    d.index = 0;
    d.min_traverse_height = 10;
    d.botpp = 1140;
    d.eotpp = d.botpp + 70;
    d.potdp = 900;
    setDeckGeometryParams(d);
    getDeckGeometryParams(0);
    return 0;
}

int ZeusModule::deinit(void) {
    // Send switchOff command
    std::string cmd = cmdHeader("AV");
    sendCommand(cmd);
    close(_sockfd);
    // printf("Closed socket.\n");
    pthread_cancel(_thread_id);
    pthread_join(_thread_id, NULL);
    // printf("Killed thread function.\n");
    // pthread_mutex_destroy(&_lock_msg);
    // printf("Destroyed mutex.\n");
    // printf("fin.\n");
    return 0;
}

int ZeusModule::lconf(void) {
    return 0;
}

void ZeusModule::moveZ(double pos, double vel) {
    printf("Moving pipetter head to: %f mm.", pos);
    std::string cmd = cmdHeader("GZ");
    pos = (1800 - 10*pos) + 370;
    if((_zpos > ZPOS_MAX) || (_zpos < ZPOS_MIN)) {
        printf("LIBZEUS: Requested z-position %f out of range.\
                Valid range for z-position is [%f,%f]",\
               _zpos, ZPOS_MIN, ZPOS_MAX);
    }
    if(vel < 1.0) {
        vel = 0.0;
    }
    if(vel > 1.0) {
        vel = 1.0;
    }
    // printf("LIBZEUS: Moving z-axis from position %f to position %f.", _zpos, pos);
    cmd += "gy" + zfill(std::to_string(static_cast<unsigned int>(pos + 0.5)),4) + "gw" + std::to_string(int(vel));
    _zpos = pos;
    sendCommand(cmd);
}

void ZeusModule::pickUpTip(struct container_cell c) {
    if((_tt_index > 9) || (_dg_index > 99)) {
        //throw error
        return;
    }

    /* Create a deck geometry struct from the tip properties in the container
      cell structure and send these parameters to the pipetter */

    /* TODO: nam - Maintain an internal index of deck geometry definitions and
     * tip types so that new parameters don't have to be set with each call to
     * this function. Currently, new parameters are set and stored at index 0
     * each time this function is called. Since the Zeus' internal EEPROM has a
     * finite number of write cycles, this may cause issues with time unless
     * the Zeus implements some wear-leveling strategy internally (unknown.)
     * - Tue 02 May 2017 11:00:23 AM MDT */

    /* NOTE: The ZEUS only supports the use of predefined tip-types, as listed
     * in section 5.12.4 of the ZEUS (II) Integrator Manual. Custom tip type
     * definitions are not supported. */
    struct deck_geometry_t d;
    d.index = 0;
    d.min_traverse_height = c.min_traverse_height;
    d.botpp = c.botpp;
    d.eotpp = c.eotpp;
    d.potdp = d.potdp;
    setDeckGeometryParams(d);

    std::string cmd = cmdHeader("GT");
    cmd += "tt" + zfill(std::to_string(c.tt_index), 1) + \
           "go" + zfill(std::to_string(0), 2) + \
           "mt" + std::to_string(0);
    sendCommand(cmd);

    /* Here we should wait for the pipetter to indicate that a tip has been
     * picked up before returning. */
    return;
}

void ZeusModule::discardTip(void) {
    if((_dg_index > 99)) {
        //throw error
        return;
    }
    std::string cmd = cmdHeader("GU");
    cmd += "go" + zfill(std::to_string(_dg_index), 2);
    sendCommand(cmd);
}

void ZeusModule::aspirate(double vol, bool mode) {

    /* TODO: nam - Modify function to accept a liquid container structure
     * reference and fetch all paramaters from the struct. On completion, parse
     * out the liquid height from the returned string and update the
     * corresponding struct element before returning from function. - Tue 14 Mar 2017 09:40:08 AM MDT */

    int _check_height = 100;
    if((vol > 11000) || (_cgt_index > 99) || (_dgt_index > 99) \
            || (_lct_index > 99) || (_qpm > 1) || (_lld > 1) \
            || (_lld_search_pos > 1800) || (_liquid_surface > 1800) \
            || (_check_height > 1800)) {
        //throw error
        return;
    }

    std::string cmd = cmdHeader("GA");
    cmd += "ai" + zfill(std::to_string(static_cast<unsigned int>(vol)), 5) +\
           "ge" + zfill(std::to_string(_cgt_index), 2) +\
           "go" + zfill(std::to_string(_dgt_index), 2) +\
           "lq" + zfill(std::to_string(_lct_index), 2) +\
           "gq" + std::to_string(_qpm) +\
           "lb" + std::to_string(_lld) +\
           "zp" + zfill(std::to_string(_lld_search_pos), 4) +\
           "cg" + zfill(std::to_string(_check_height), 4) + \
           "cf" + zfill(std::to_string(_liquid_surface), 4);
    sendCommand(cmd);
}

void ZeusModule::dispense(double vol, bool mode) {

    /* TODO: nam - Modify function to accept a liquid container structure
     * reference and fetch all paramaters from the struct. On completion, parse
     * out the liquid height from the returned string and update the
     * corresponding struct element before returning from function. - Tue 14 Mar 2017 09:40:08 AM MDT */

    std::string cmd = cmdHeader("GD");
    cmd += "di" + zfill(std::to_string(static_cast<unsigned int>(vol)), 4) +\
           "ge" + zfill(std::to_string(_cgt_index), 2) +\
           "go" + zfill(std::to_string(_dgt_index), 2) +\
           "lq" + zfill(std::to_string(_lct_index), 2) +\
           "gq" + std::to_string(_qpm) +\
           "lb" + std::to_string(_lld) +\
           "zp" + zfill(std::to_string(_lld_search_pos), 4) +\
           "cf" + zfill(std::to_string(_liquid_surface), 4) +\
           "zm" + std::to_string(_search_bottom_mode);
    sendCommand(cmd);
}

void ZeusModule::getLastFaultyParameter(void) {
    printf("Getting last faulty parameter.\n");
    std::string cmd = cmdHeader("VP");
    sendCommand(cmd);
}

int ZeusModule::initCANBus(void) {
    /*
     * We first need to run the init_can.sh script to set up the CAN bus
     * for communications. Check if the script exists, and if so, run it. Else,
     * throw an error and exit.
     */
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;

    char cwd[1024];
    struct stat buffer;
    if(getcwd(cwd, sizeof(cwd)) != NULL) {
        // std::string cur_dir(cwd);
        std::string cur_dir = "/home/cdx/catkin_ws/src/cdxbot";
        cur_dir += "/init_can.sh";
        printf("cur_dir = %s \n", cur_dir.c_str());
        std::string cmd = "sudo bash " + cur_dir;
        if(stat(cur_dir.c_str(), &buffer) == 0) {
            if(system(cmd.c_str()) != 0) {
                perror("error running init_can.sh");
                exit(1);
            }
        }
    } else {
        perror("getcwd() error");
        exit(1);
    }

    /* Open and bind to a socket for CAN bus communications */
    if((_sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("ERROR:LIBZEUS - Error while opening socket.\n");
        exit(1);
    }
    printf("Opened CAN socket with file descriptor %d\n", _sockfd);

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, _interface.c_str());
    if(ioctl(_sockfd, SIOCGIFINDEX, &ifr) < 0) {
        perror("IOCTL error.");
        exit(1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);
    sprintf(cwd, "sockfd = %d\n", _sockfd);
    perror(cwd);
    if(bind(_sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("ERROR:LIBZEUS - Error while attempting to bind to socket.\n");
        close(_sockfd);
        exit(2);
    }
    return 0;
}

std::string ZeusModule::cmdHeader(std::string cmd) {
    std::string ret;
    ret = cmd + "id" + zfill("1", 4);
    return ret;
}

int ZeusModule::initDosingDrive(void) {
    std::string cmd = cmdHeader("DI");
    sendCommand(cmd);
    return 0;
}

int ZeusModule::initZDrive(void) {
    std::string cmd = cmdHeader("ZI");
    _zpos = ZPOS_MIN;
    sendCommand(cmd);
    return 0;
}

unsigned int ZeusModule::getInitializationStatus(void) {
    std::string cmd = cmdHeader("QW");
    setWaitingForMsgFlag(1);
    sendCommand(cmd);
    while(getWaitingForMsgFlag() == 1) {}
    std::string ret = getReceivedMsg().substr(getReceivedMsg().find("qw")+2, 1);
    printf("Init status = %s\n", ret.c_str());
    if(stoi(ret) == 1) {
        return INIT_SUCCESS;
    }
    return INIT_FAILURE;
}

void ZeusModule::getFirmwareVersion(void) {
    // std::string cmd = cmdHeader("RF");
    sendCommand("RF");
}

canid_t ZeusModule::assembleIdentifier(unsigned int type) {
    canid_t identifier = 0;
    identifier |= _id;
    if(_master_id > 0) {
        identifier |= (_master_id << 5);
    }
    if(type == CAN_MSG_KICK) {
        identifier |= 1 << 10;
    }
    return identifier;
}

void ZeusModule::sendRemoteFrame(unsigned int dlc) {
    // printf("Sending remote frame.\n");
    struct can_frame f;
    memset(&f, 0, sizeof(struct can_frame));
    // f.can_id = assembleIdentifier(CAN_MSG_DATA);
    //f.can_id = 20;
    f.can_id |= _id;
    f.can_id = 0x20;
    if(_master_id > 0) {
        f.can_id |= (_master_id << 5);
    }
    f.can_id |= CAN_RTR_FLAG;
    f.can_dlc = dlc;
    sendFrame(f);
}

bool ZeusModule::waitForRemoteFrame(void) {
    unsigned int n = 0;
    /* Wait REMOTE_TIMEOUT seconds for remote frame. If no remote frame is
     * received within the allotted time, resend the previously sent frame. Try
     * this up to _transmission_retries times. */
    for(n = 0; n < _transmission_retries; n++) {
        time_t start = time(NULL);
        while((time(NULL) - start) < REMOTE_TIMEOUT) {
            if(getRemoteFlag()) {
                setRemoteFlag(0);
                return 1;
            }
        }
        /* Resend the last sent frame if no remote frame is received */
        printf("LIBZEUS: Timeout waiting for remote response.\
                    issuing retry %u of %d.\n", n+1, _transmission_retries);
        sendFrame(_last_sent_frame);
    }
    printf("ERROR: No remote frame received from device.\n");
    exit(1);
    return 0;
}

void ZeusModule::sendKickFrame(void) {
    // printf("Sending kick frame. \n");
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = assembleIdentifier(CAN_MSG_KICK);
    frame.can_dlc = 8;
    sendFrame(frame);
}

int ZeusModule::sendFrame(struct can_frame f) {
    int nbytes = write(_sockfd, &f, sizeof(struct can_frame));
    if(nbytes < 0) {
        perror("Error writing to CAN bus.");
        printf("%d\n", errno);
        exit(1);
    }
    setLastFrame(f);
    return nbytes;
}

bool ZeusModule::waitForKickFrame(void) {
    unsigned int n = 0;
    /* Wait REMOTE_TIMEOUT seconds for remote frame. If no remote frame is
     * received within the allotted time, resend the previously sent frame. Try
     * this up to _transmission_retries times. */
    for(n = 0; n < _transmission_retries; n++) {
        time_t start = time(NULL);
        while((time(NULL) - start) < REMOTE_TIMEOUT) {
            if(getKickFlag()) {
                setKickFlag(0);
                return 1;
            }
        }
        /* Resend the last sent frame if no remote frame is received */
        printf("LIBZEUS: Timeout waiting for remote response.\
                    issuing retry %u of %d.\n", n+1, _transmission_retries);
        sendFrame(getLastFrame());
    }
    printf("ERROR: No remote frame received from device.\n");
    exit(1);
    return 0;
}

void ZeusModule::setLastFrame( struct can_frame &f ) {
    _last_sent_frame = f;
}

struct can_frame & ZeusModule::getLastFrame(void) {
    return _last_sent_frame;
}

void ZeusModule::sendCommand(std::string cmd) {
    /* At the end of this function, we will block until a return string is sent
     * from the pipetter to avoid sending simultaneous commands. */
    _ready_for_new_command = 0;
    if(PRINT_OUTPUT) {
        printf("Sending command: %s. -> ", cmd.c_str());
    }
    std::vector<std::string> substrings;
    size_t sbegin = 0;

    /* Split incoming string into substrings of length 7 bytes or less. */
    do {
        std::string s = cmd.substr(sbegin, 7);
        substrings.push_back(s);
        sbegin += 7;
    } while(sbegin < cmd.size());
    /* Transmit each 7 bytes as a separate CAN frame */
    sendKickFrame();
    waitForRemoteFrame();
    for(size_t i = 0; i < substrings.size(); i++) {
        char byte = 0;
        // printf("writing CAN frame with content %s\n",substrings[i].data());
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));
        frame.can_id = assembleIdentifier(CAN_MSG_DATA);
        frame.can_dlc = substrings[i].size();
        /* Copy string data byte-by-byte into can data field. */
        for(size_t j = 0; j < substrings[i].size(); j++) {
            frame.data[j] = substrings[i][j];
        }
        if(frame.can_dlc < 8) {
            for(int k = frame.can_dlc; k < 7; k++) {
                frame.data[k] = 0;
            }
        }
        /* Append EOM bit to byte if this is the last frame in the message.*/
        if(i == (substrings.size() - 1)) {
            // printf("Appending EOM bit to byte\n");
            byte |= (1 << 7);
        }
        /* Append message length and index data to byte */
        byte |= substrings[i].size() << 4;
        byte |= ((i + 1) %31);
        frame.data[7] = byte;
        // frame.can_dlc++;
        frame.can_dlc = 8;
        sendFrame(frame);
        if(byte & (1 << 7)) {
            /* Instead of a blind delay, set a 'ready for new command' flag after error
             * parsing the returned string in thread_func() */

            /* Wait until command completes, up to a maximum of 1 second. */
            // time_t start = time(NULL);
            // while((!_ready_for_new_command) && (time(NULL) < start + 1)) {
            /* Sleep for 10ms between checks so that this section doesn't
             * peg the CPU with calls to time() */
            // usleep(10000);
            // }
            usleep(1000000);
            _ready_for_new_command = 1;
            return;
        }
        waitForRemoteFrame();
    }
}

bool ZeusModule::getTipStatus(void) {
    std::string cmd = cmdHeader("RT");
    setWaitingForMsgFlag(1);
    sendCommand(cmd);
    while(getWaitingForMsgFlag() == 1) {}
    std::string ret = getReceivedMsg().substr(getReceivedMsg().find("rt")+2, 1);

    return stoi(ret);
}

double ZeusModule::getZPos(void) {
    std::string cmd = cmdHeader("RZ");
    setWaitingForMsgFlag(1);
    sendCommand(cmd);
    while(getWaitingForMsgFlag() == 1) {}
    std::string z_str = getReceivedMsg().substr(getReceivedMsg().find("gy")+2, 3);
    // printf("ZPOS is: %d\n", stoi(z_str));
    return stod(z_str);
}

void ZeusModule::emergencyStop(void) {
    std::string cmd = "AB";
    sendCommand(cmd);
}

void ZeusModule::emergencyStopReset(void) {
    std::string cmd = "AW";
    sendCommand(cmd);
}

void ZeusModule::setDeckGeometryParams(struct deck_geometry_t d) {
    /* Validate struct members */
    if((d.index > 9999) || (d.min_traverse_height > 1800) || (d.min_z_pos > 1800)
            || (d.botpp > 1800) || (d.eotpp > 1800) || (d.potdp > 1800)) {
        printf("ERROR: Deck geometry definition contains out of range\
               parameter.\n");
        return;
    }
    std::string cmd = cmdHeader("GO") +
                      "go" + zfill(std::to_string(d.index), 2) + \
                      "th" + zfill(std::to_string(d.min_traverse_height), 4) + \
                      "te" + zfill(std::to_string(d.min_z_pos), 4) + \
                      "tm" + zfill(std::to_string(d.botpp), 4) + \
                      "tn" + zfill(std::to_string(d.eotpp), 4) + \
                      "tr" + zfill(std::to_string(d.potdp), 4);
    sendCommand(cmd);
}

void ZeusModule::getDeckGeometryParams(unsigned int index) {
    if(index > 99) {
        printf("ERROR: Requested deck geometry table index is out of range.\n");
        return;
    }
    std::string cmd = cmdHeader("GR") + \
                      "go" + zfill(std::to_string(index), 2);
    sendCommand(cmd);
}

void ZeusModule::setContainerGeometryParams(struct container_geometry_t c) {
    /* Validate bounds on struct members*/
    if((c.index > 99) || (c.diameter > 999)
            || (c.x_length > 999) ||(c.y_length > 999)
            || (c.second_height > 1800) || (c.second_section > 10000)
            || (c.minimum_height > 1800) || (c.sohbs > 1800)
            || (c.dhabs > 1800)) {
        printf("ERROR: Container geometry definition contains out of range\
               parameter.\n");
    }
    std::string cmd = cmdHeader("GC") + \
                      "ge" + zfill(std::to_string(c.index), 2) + \
                      "ca" + zfill(std::to_string(c.geometry), 1) + \
                      "cb" + zfill(std::to_string(c.diameter), 3) + \
                      "cc" + zfill(std::to_string(c.x_length), 3) + \
                      "cd" + zfill(std::to_string(c.y_length), 3) + \
                      "bg" + zfill(std::to_string(c.second_height), 4) + \
                      "gx" + zfill(std::to_string(c.second_section), 5) + \
                      "ce" + zfill(std::to_string(c.minimum_height), 4) + \
                      "ch" + zfill(std::to_string(c.sohbs), 4) + \
                      "ic" + zfill(std::to_string(c.dhabs), 4);
    sendCommand(cmd);
}

void ZeusModule::getContainerGeometryParams(unsigned int index) {
    if(index > 99) {
        printf("ERROR: Requested deck geometry table index is out of range.\n");
        return;
    }
    std::string cmd = cmdHeader("GB") + \
                      "ge" + zfill(std::to_string(index), 2);
    sendCommand(cmd);

}

void ZeusModule::setLiquidClassParams(struct liquid_class_t l) {
    /* Validate bounds on struct members*/
    if((l.index > 99) || (l.lcfft > 1) || (l.aspiration_mode > 2) ||
            (l.aspiration_flow_rate > 25000) || (l.over_aspirated_vol > 9999) ||
            (l.aspiration_transport_vol > 10000) || (l.blowout_air_vol > 10000) ||
            (l.aspiration_swap_speed > 3000) || (l.aspiration_settling_time > 999)
            || (l.lld > 1) || (l.clld_sens > 4) || (l.plld_sens > 4) || (l.adc > 0)
            || (l.dispensing_mode > 3) || (l.dispensing_flow_rate > 25000) ||
            (l.stop_flow_rate > 25000) || (l.stop_back_vol > 325) ||
            (l.dispensing_transport_vol > 10000) || (l.acceleration > 200) ||
            (l.dispensing_swap_speed > 3000) || (l.dispensing_settling_time > 999)
            || (l.flow_rate_transport_vol > 25000)) {
        printf("ERROR: Liquid class definition contains out of range\
               parameter.\n");
    }
    std::string cmd = cmdHeader("GL") + \
                      "iq" + zfill(std::to_string(l.index), 2) + \
                      "uu" + std::to_string(l.lcfft) + \
                      std::to_string(l.aspiration_mode) + \
                      zfill(std::to_string(l.aspiration_flow_rate), 5) + \
                      zfill(std::to_string(l.over_aspirated_vol), 4) + \
                      zfill(std::to_string(l.aspiration_transport_vol), 5) + \
                      zfill(std::to_string(l.blowout_air_vol), 5) + \
                      zfill(std::to_string(l.aspiration_swap_speed), 4) + \
                      zfill(std::to_string(l.aspiration_settling_time), 3) + \
                      std::to_string(l.lld) + \
                      zfill(std::to_string(l.clld_sens), 4) + \
                      zfill(std::to_string(l.plld_sens), 4) + \
                      std::to_string(l.adc) + \
                      std::to_string(l.dispensing_mode) + \
                      zfill(std::to_string(l.dispensing_flow_rate), 5) + \
                      zfill(std::to_string(l.stop_flow_rate), 5) + \
                      zfill(std::to_string(l.stop_back_vol), 3) + \
                      zfill(std::to_string(l.dispensing_transport_vol), 5) + \
                      zfill(std::to_string(l.acceleration), 3) + \
                      zfill(std::to_string(l.dispensing_swap_speed), 4) + \
                      zfill(std::to_string(l.dispensing_settling_time), 3) + \
                      zfill(std::to_string(l.flow_rate_transport_vol), 5);
    sendCommand(cmd);
}

void ZeusModule::getLiquidClassParams(unsigned int index) {
    if(index > 99) {
        printf("ERROR: Requested deck geometry table index is out of range.\n");
        return;
    }
    std::string cmd = cmdHeader("GM") + \
                      "iq" + zfill(std::to_string(index), 2);
    sendCommand(cmd);
}

std::string ZeusModule::parseErrors(std::string error) {
    /* If we're in this function, the last sent command completed and returned
     * a string, so we can allow new commands to be sent as of now. */
    _ready_for_new_command = 1;
    std::string cmd, ec;
    std:: string default_error = "Unknown error code returned.";
    size_t eidx;
    boost::erase_all(error," ");
    if(error.length() == 0) {
        return "NONE";
    }
    cmd = error.substr(0,2);
    if(cmd == "VP") {
        eidx = error.find("vp");
        return cmd + ": Parameter " + error.substr(eidx+2, error.size() - eidx+1) + " is invalid. ";
    }

    eidx = error.find("er");
    if(eidx == std::string::npos) {
        return cmd + ": " + error;
    }
    ec = error.substr((eidx + 2), 2);

    if(ec == "00") {
        return cmd + ": " + "NONE";
    }
    if(ec == "31") {
        // getLastFaultyParameter();
        _error_flag = 1;
    }
    if(cmd == "DI") {
        std::vector<std::string> codes = {"00", "30", "35", "36", "40", "50",
                                          "52"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return  cmd + ": " + _error_table[*it];
        } else {
            return  cmd + ": " + default_error + "(" + ec + ")";
        }
    }

    else if(cmd == "ZI") {
        std::vector<std::string> codes = {"00", "30", "35", "36", "40", "60",
                                          "62"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return  cmd + ": " + _error_table[*it];
        } else {
            return  cmd + ": " + default_error;
        }
    }

    else if(cmd == "GZ") {
        std::vector<std::string> codes = {"00", "31", "32", "35", "36", "40",
                                          "61", "62", "64"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return  cmd + ": " + _error_table[*it];
        } else {
            return  cmd + ": " + default_error;
        }
    }

    else if(cmd == "GT") {
        std::vector<std::string> codes = {"00", "31", "32", "35", "36", "40",
                                          "51", "52", "61", "62", "65", "75",
                                          "76"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return  cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GU") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "36",
                                          "40", "51", "52", "61", "62", "65",
                                          "69", "75", "77"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GA") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "36",
                                          "38", "40", "51", "52", "53", "54",
                                          "55", "56", "57", "61", "62", "65",
                                          "66", "67", "68", "70", "71", "72",
                                          "74", "75", "80", "81", "82", "85"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GD") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "36",
                                          "38", "40", "51", "52", "53", "54",
                                          "55", "56", "57", "61", "62", "65",
                                          "66", "67", "68", "70", "71", "72",
                                          "74", "75", "80", "81", "82", "85"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GJ") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "36",
                                          "38", "40", "51", "52", "56", "57",
                                          "61", "62", "65", "66", "67", "68",
                                          "70", "72", "74", "85"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "AB") {
        std::vector<std::string> codes = {"00", "30"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "AW") {
        std::vector<std::string> codes = {"00", "30"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "XA") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GK") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "51",
                                          "52", "54"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GC") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }


    else if(cmd == "GO") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GB") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GR") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GL") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GM") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GQ") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GS") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GV") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GW") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GG") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GE") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GH") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else if(cmd == "GI") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return cmd + ": " + _error_table[*it];
        } else {
            return cmd + ": " + default_error;
        }
    }

    else {
        return  cmd + ": " + "Error code returned corresponds to unknown command.";
    }

}

struct can_frame ZeusModule::getNextMessage(void) {
    struct can_frame ret = _fifo.front();
    _msg_ready_flag = 0;
    return ret;
}
