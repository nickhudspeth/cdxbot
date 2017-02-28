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
    /* Check to see if there is any incoming data on the CAN bus.
     * If so, push the data to a FIFO.*/
    struct can_frame ret;
    int n = 0;  // Number of received bytes
    ZeusModule *zm = (ZeusModule *)arg;
    int soc = zm->getSockFD();
    zm-> _read_can_port = 1;
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
                n = read(soc, &ret, sizeof(struct can_frame));
                if(n) {
                    if(n < 0) {
                        printf("LIBZEUS: Error reading from CAN socket.\n");
                    } else if(n < sizeof(struct can_frame)) {
                        printf("LIBZEUS: Incomplete frame read from CAN socket.\n");
                    } else {
                        (zm->getFIFO()).push(ret);
                        zm->setMsgReadyFlag(1);
                    }

                }
            }
        }

    }
    return 0;
}

extern "C" std::string zfill(std::string s, int len) {
    s.insert(s.begin(), len - s.length(), '0');
    return s;
}


ZeusModule::ZeusModule() {}
ZeusModule::~ZeusModule() {}

int ZeusModule::init(void) {
    initCANBus();
    initZDrive();
    initDosingDrive();

    return 0;
}

int ZeusModule::deinit(void) {
    // Send switchOff command
    std::string cmd = cmdHeader("AV");
    sendCommand(cmd);

    return 0;
}

int ZeusModule::lconf(void) {
    return 0;
}

void ZeusModule::moveZDrive(double pos, double vel) {
    std::string cmd = cmdHeader("GZ");
    if((_zpos > _zpos_max) || (_zpos < _zpos_min)) {
        printf("LIBZEUS: Requested z-position %f out of range.\
                Valid range for z-position is [%f,%f]",\
               _zpos, _zpos_min, _zpos_max);
    }
    if(vel < 1.0) {
        vel = 0.0;
    }
    if(vel > 1.0) {
        vel = 1.0;
    }
    printf("LIBZEUS: Moving z-axis from position %f to position %f.", _zpos, pos);
    cmd += "gy" + zfill(std::to_string(pos),4) + "gw" + std::to_string(int(vel));
    _zpos = pos;
    sendCommand(cmd);
}

void ZeusModule::pickUpTip(void) {
    std::string cmd = cmdHeader("GT");
    cmd += "gy" + zfill(std::to_string(_tt_index), 2) + "go" +\
           zfill(std::to_string(_dg_index), 2);
    sendCommand(cmd);
}

void ZeusModule::discardTip(void) {
    std::string cmd = cmdHeader("GU");
    cmd += "go" + zfill(std::to_string(_dg_index), 2);
    sendCommand(cmd);
}

void ZeusModule::aspirate(double vol) {
    std::string cmd = cmdHeader("GA");
    cmd += "ai" + zfill(std::to_string(vol), 5) +\
           "ge" + zfill(std::to_string(_cgt_index), 2) +\
           "go" + zfill(std::to_string(_dgt_index), 2) +\
           "lq" + zfill(std::to_string(_lct_index), 2) +\
           "gq" + std::to_string(_qpm) +\
           "lb" + std::to_string(_lld) +\
           "zp" + zfill(std::to_string(_lld_search_pos), 4) +\
           "cf" + zfill(std::to_string(_liquid_surface), 4) +\
           "ma" + zfill(std::to_string(_mix_vol), 5) +\
           "mb" + zfill(std::to_string(_mix_flow_rate), 5) +\
           "dn" + zfill(std::to_string(_mix_cycles), 2);
    sendCommand(cmd);
}

void ZeusModule::dispense(double vol) {
    std::string cmd = cmdHeader("GD");
    cmd += "di" + zfill(std::to_string(vol), 4) +\
           "ge" + zfill(std::to_string(_cgt_index), 2) +\
           "go" + zfill(std::to_string(_dgt_index), 2) +\
           "gq" + std::to_string(_qpm) +\
           "lq" + zfill(std::to_string(_lct_index), 2) +\
           "lb" + std::to_string(_lld) +\
           "zp" + zfill(std::to_string(_lld_search_pos), 4) +\
           "cf" + zfill(std::to_string(_liquid_surface), 4) +\
           "zm" + std::to_string(_search_bottom_mode) +\
           "ma" + zfill(std::to_string(_mix_vol), 5) +\
           "mb" + zfill(std::to_string(_mix_flow_rate), 5) +\
           "dn" + zfill(std::to_string(_mix_cycles), 2);
    sendCommand(cmd);
}

int ZeusModule::initCANBus(void) {
    int nbytes;
    struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
    const char *ifname =_interface.c_str();

    if((_sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        /* TODO: nam -   Mon 23 Jan 2017 12:05:08 PM MST */
        // Maybe consider placing return from socket() call in function-local
        // variable first to check if return < 0, and only assigning to
        // _sockfd if the return value is valid. This way _sockfd will never
        // be assigned an invalid value other than the initialization value.
        perror("LIBZEUS: Error while opening socket.\n");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(_sockfd, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(_sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("LIBZEUS: Error while attempting to bind to socket.\n");
        return -2;
    }
    return 0;
}

std::string ZeusModule::cmdHeader(std::string cmd) {
    std::string ret;
    ret = cmd + "id" + zfill(std::to_string(_id), 4);
    return ret;
}

int ZeusModule::initDosingDrive(void) {
    std::string cmd = cmdHeader("DI");
    sendCommand(cmd);
    return 0;
}

int ZeusModule::initZDrive(void) {
    std::string cmd = cmdHeader("ZI");
    _zpos = _zpos_min;
    sendCommand(cmd);
    return 0;
}

void ZeusModule::setAutoResponse(void) {}


int ZeusModule::assembleIdentifier(unsigned int type) {
    int identifier = 0;
    identifier |= _id;
    if(_master_id > 0) {
        identifier |= (_master_id << 5);
    }
    if(type == CAN_MSG_KICK) {
        identifier |= 1 << 10;
    }
    return identifier;
}

void ZeusModule::sendRemoteFrame(unsigned int dlc) {}

bool ZeusModule::waitForRemoteFrame(void) {}

void ZeusModule::sendKickFrame(void) {}

bool ZeusModule::waitForKickFrame(void) {}

void ZeusModule::sendDataObject(unsigned int i, unsigned int cmd_len, int data) {}

void ZeusModule::sendCommand(std::string cmd) {
    std::vector<std::string> substrings;
    size_t sbegin = 0;
    char byte = 0;
    size_t nbytes = 0;
    //Split incoming string into substrings of length 7 bytes or less.
    do {
        std::string s = cmd.substr(sbegin, 7);
        substrings.push_back(s);
        sbegin += 7;
    } while(sbegin < cmd.size());
    // Transmit each 7 bytes as a separate CAN frame
    for(size_t i = 0; i < (substrings.size() - 1); i++) {
        struct can_frame frame;
        frame.can_id = assembleIdentifier(CAN_MSG_DATA);
        for(size_t j = 0; j < (substrings[i].size() - 1); j++) {
            frame.data[j] = substrings[i][j];
        }
        if(i == (cmd.size() - 1)) {
            byte |= (1 << 7);
        }
        byte |= substrings[i].size() << 4;
        byte |= ((i + 1) %31);
        frame.data[7] = byte;

        sendKickFrame();
        for(size_t j = 0; j < _transmission_retries; j++) {
            nbytes = write(_sockfd, &frame, sizeof(struct can_frame));
            if((frame.data[7] & EOM_MASK) == 1) {
                return;
            }
            if(waitForRemoteFrame() == 1) {
                break;
            } else {
                printf("LIBZEUS: Timeout waiting for remote response.\
                    issuing retry %lu of %d.\n", j+1, _transmission_retries);
            }
            waitForKickFrame();
        }
    }
}

bool ZeusModule::getTipStatus(void) {
    std::string cmd = cmdHeader("RT");
    sendCommand(cmd);
    // Grab and return parameter value from pipetter
}

double ZeusModule::getZPos(void) {
    std::string cmd = cmdHeader("RZ");
    sendCommand(cmd);
    // Grab and return parameter value from pipetter
}

void ZeusModule::emergencyStop(void) {
    std::string cmd = "AB";
    sendCommand(cmd);
}

void ZeusModule::emergencyStopReset(void) {
    std::string cmd = "AW";
    sendCommand(cmd);
}

std::string ZeusModule::parseErrors(std::string error) {
    std::string cmd, ec;
    std:: string default_error = "Unknown error code returned.";
    size_t eidx;
    boost::erase_all(error," ");
    if(error.length() == 0) {
        return "NONE";
    }
    cmd = error.substr(0,1);
    eidx = error.find("er");
    if(eidx == std::string::npos) {
        return "NONE";
    }
    ec = error.substr((eidx + 2), (eidx + 4));

    if(ec == "00") {
        return "NONE";
    }
    if(cmd == "DI") {
        std::vector<std::string> codes = {"00", "30", "35", "36", "40", "50",
                                          "52"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "ZI") {
        std::vector<std::string> codes = {"00", "30", "35", "36", "40", "60",
                                          "62"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GZ") {
        std::vector<std::string> codes = {"00", "31", "32", "35", "36", "40",
                                          "61", "62", "64"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GT") {
        std::vector<std::string> codes = {"00", "31", "32", "35", "36", "40",
                                          "51", "52", "61", "62", "65", "75",
                                          "76"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GU") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "36",
                                          "40", "51", "52", "61", "62", "65",
                                          "69", "75", "77"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
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
            return _error_table[*it];
        } else {
            return default_error;
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
            return _error_table[*it];
        } else {
            return default_error;
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
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "AB") {
        std::vector<std::string> codes = {"00", "30"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "AW") {
        std::vector<std::string> codes = {"00", "30"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "XA") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GK") {
        std::vector<std::string> codes = {"00", "30", "31", "32", "35", "51",
                                          "52", "54"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GC") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }


    else if(cmd == "GO") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GB") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GR") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GL") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"};
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GM") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GQ") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GS") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GV") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GW") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GG") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GE") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GH") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else if(cmd == "GI") {
        std::vector<std::string> codes = {"00", "20", "30", "31", "32"
                                         };
        std::vector<std::string>::iterator it = find(codes.begin(), codes.end(), ec);
        if(it != codes.end()) {
            return _error_table[*it];
        } else {
            return default_error;
        }
    }

    else {
        return "Error code returned corresponds to unknown command.";
    }

}

struct can_frame ZeusModule::getNextMessage(void) {
    struct can_frame ret = _fifo.front();
    _msg_ready_flag = 0;
    return ret;
}

void ZeusModule::CANOpenPort() {

}

void ZeusModule::CANClosePort() {

}

void ZeusModule::CANReadPort() {
    struct can_frame ret;
    int n = 0;  // Number of received bytes

    int read_can_port = 1;
    while(_read_can_port) {
        struct timeval timeout = {1,0};
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(_sockfd, &readSet);
        if(select((_sockfd + 1), &readSet, NULL, NULL, &timeout) >= 0) {
            if(!_read_can_port) {
                break;
            }
            if(FD_ISSET(_sockfd, &readSet)) {
                n = read(_sockfd, &ret, sizeof(struct can_frame));
                if(n) {
                    if(n < 0) {
                        printf("LIBZEUS: Error reading from CAN socket.\n");
                    } else if(n < sizeof(struct can_frame)) {
                        printf("LIBZEUS: Incomplete frame read from CAN socket.\n");
                    } else {
                        _fifo.push(ret);
                        _msg_ready_flag = 1;
                    }

                }
            }
        }

    }

}

void ZeusModule::CANWritePort() {

}
