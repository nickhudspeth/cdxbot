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
#define TO_ZEUS_COORDS(pos, offset) (1800 - (10*(pos + offset)))
#define TO_DECK_COORDS(pos, offset) (0.1*(1800 - pos) - offset)
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
                        zm->PRINT_ERROR("LIBZEUS: Error reading from CAN socket.");
                    } else if(n < sizeof(struct can_frame)) {
                        zm->PRINT_ERROR("LIBZEUS: Incomplete frame read from CAN socket.");
                    } else {
                        if(ret.can_id & KICK_MASK) {
                            /* Received kick frame. */
                            //zm->setKickFlag(1);
                            zm->sendRemoteFrame(1);
                            // PRINT_DEBUG("LIBZEUS: Received kick frame with byte %04X.\n", ret.data[7]);
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
                            // zm->PRINT_DEBUG("LIBZEUS: Received message: " + msg);

                            /*If frame data contains EOM flag, process the returned string. */
                            if(ret.data[7] & EOM_MASK) {
                                zm->setReceivedMsg(msg);
                                zm->setMsgReadyFlag(1);
                                // zm->PRINT_DEBUG("LIBZEUS: Set msgreadyflag to 1.");
                                zm->setWaitingForMsgFlag(0);
                                // std::string err_msg = zm->parseErrors(msg);
                                // zm->PRINT_DEBUG("LIBZEUS: Received message: " + msg);
                                msg.clear();
                            } else {
                                zm->sendRemoteFrame(8);
                            }
                            // zm->PRINT_DEBUG("LIBZEUS: Read a frame from da socket with DLC " + std::to_string(ret.can_dlc) + " and data " + std::to_string(ret.data));
                        }
                    }
                }
            }
        }
        if(zm->_error_flag) {
            zm->PRINT_ERROR("LIBZEUS: Error flag set. Getting last faulty parameter.\n");
            zm->getLastFaultyParameter();
            zm->_error_flag = 0;
        }
    }
    zm->PRINT_DEBUG("LIBZEUS: Exiting thread loop.\n");
    return 0;
}


extern "C" std::string zfill(std::string s, int len) {
    if(s.length() == len) {
        return s;
    }
    if(s.length() > len) {
        return s.substr(0, len); // Trim trailing characters from string that exceeds specified length.
    }
    s.insert(s.begin(), len - s.length(), '0');
    return s;
}



ZeusModule::ZeusModule(int id) {
    for(unsigned int i = 0; i < 100; i++) {
        _liquid_classes.emplace_back(new LiquidClass(""));
    }
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
    initCANBus();
    pthread_create(&_thread_id, NULL, &thread_func, this);
    PRINT_DEBUG("LIBZEUS: Thread function created.");
    // initZDrive();
    // initDosingDrive();

    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 5");
    // std::string cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(5), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 7");
    // cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(7), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 8");
    // cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(8), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 9");
    // cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(9), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 17");
    // std::string cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(17), 2);
    // sendCommand(cmd);
    // cmd = cmdHeader("GS") + "gv" + zfill(std::to_string(17), 2);
    // sendCommand(cmd);
    // cmd = cmdHeader("GW") + "gp" + zfill(std::to_string(17), 2);
    // sendCommand(cmd);
    // cmd = cmdHeader("GE") + "gg" + zfill(std::to_string(17), 2);
    // sendCommand(cmd);
    // cmd = cmdHeader("GI") + "gh" + zfill(std::to_string(17), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 19");
    // cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(19), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 20");
    // cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(20), 2);
    // sendCommand(cmd);
    // PRINT_WARNING("LIBZEUS: Liquid Class Parameters for index 21");
    // cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(21), 2);
    // sendCommand(cmd);

    // getInitializationStatus();
    // getFirmwareVersion();
    // struct deck_geometry_t d;
    // d.index = 0;
    // d.min_traverse_height = 10;
    // d.botpp = 1140;
    // d.eotpp = d.botpp + 70;
    // d.potdp = 900;
    // setDeckGeometryParams(d);
    // getDeckGeometryParams(0);
    return 0;
}

int ZeusModule::deinit(void) {
    // Send switchOff command
    std::string cmd = cmdHeader("AV");
    sendCommand(cmd);
    close(_sockfd);
    PRINT_DEBUG("LIBZEUS: Closed socket.");
    pthread_cancel(_thread_id);
    pthread_join(_thread_id, NULL);
    PRINT_DEBUG("LIBZEUS: Terminated worker thread.");
    return 0;
}

int ZeusModule::lconf(void) {
    return 0;
}

bool ZeusModule::moveZ(double pos, double vel) {
    // printf("Moving pipetter head to: %f mm.\n", pos);
    std::string cmd = cmdHeader("GZ");
    // if((pos > ZPOS_MAX) || (pos < ZPOS_MIN)) {
    // PRINT_ERROR("LIBZEUS: Requested z-position " + std::to_string(pos) \
    // + " out of range. Valid range for z-position is [" \
    // + std::to_string(ZPOS_MIN/10.0) + ", " \
    // + std::to_string(ZPOS_MAX/10.0) + "]");
    // return 0;
    // }
    if(vel < 1.0) {
        vel = 0.0;
    }
    if(vel > 1.0) {
        vel = 1.0;
    }
    cmd += "gy" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(pos, TIP_INSTALLED_OFFSET))),4) + "gw" + std::to_string(int(vel));
    return sendCommand(cmd);
    // double cur_z_pos = getZPos();
    // double err = 99.0;
    // time_t start = time(NULL);
    // while((err < MAX_Z_ERROR) && ((time(NULL) - start) < 1)) {
    // cur_z_pos = getZPos();
    // err = abs(cur_z_pos - pos);
    // }
    // if(err < MAX_Z_ERROR) {
    // _zpos = pos;
    // return 1;
    // } else {
    // printf("Pipetter z-pos = %f\n", getZPos());
    // return 0;
    // }
}


bool ZeusModule::makeDeckGeometry(unsigned int index, double traverse_height,\
                                  double min_end_cmd_height, \
                                  double container_offset_z, \
                                  double tip_engagement_len, \
                                  double tip_deposit_height) {
    /* It's only necessary to perform range check on the difference of these two
     * variables. Error checking on all other parameters is performed by the
     * Zeus module.*/
    if(container_offset_z < tip_engagement_len) {
        return false;
    }
    // pos = (1800 - 10*traverse_height) + 370;

    PRINT_DEBUG("LIBZEUS: Making deck geometry at index " + std::to_string(index)\
                + " with traverse_height " + std::to_string(traverse_height) + " and container z-offset " + std::to_string(container_offset_z) + " and tip engagement length " + std::to_string(tip_engagement_len) + " and tip deposit height " + std::to_string(tip_deposit_height));
    std::string cmd = cmdHeader("GO");
    cmd +=  "go" + zfill(std::to_string(index), 2) + \
            /*"th" + zfill(std::to_string(static_cast<unsigned int>(traverse_height * 10)), 4) + \ */
            "th" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(min_end_cmd_height, TIP_INSTALLED_OFFSET))), 4) + \
            // "th" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(traverse_height, TIP_INSTALLED_OFFSET))), 4) + \
            /*"te" + zfill(std::to_string(static_cast<unsigned int>((1800 - 10*(container_offset_z - tip_engagement_len)))), 4) + \*/
            "te" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(min_end_cmd_height, TIP_INSTALLED_OFFSET))), 4) + \
            /*"tm" + zfill(std::to_string(static_cast<unsigned int>(10*container_offset_z)), 4 )+ \ */
            "tm" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(container_offset_z, 0))), 4 )+ \
            /* "tm" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(traverse_height, 0))), 4 )+ \ */
            "tn" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(container_offset_z - tip_engagement_len, 0))), 4) + \
            "tr" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(tip_deposit_height, 0))), 4);
    _current_dg_index = index;
    return sendCommand(cmd);
}

bool ZeusModule::getDeckGeometryParams(unsigned int index) {
    if(index > 99) {
        printf("ERROR: Requested deck geometry table index is out of range.\n");
        return false;
    }
    std::string cmd = cmdHeader("GR") + \
                      "go" + zfill(std::to_string(index), 2);
    sendCommand(cmd);
    return true;
}

bool ZeusModule::makeContainerGeometry(unsigned int index, bool geometry,
                                       double diameter, double len_x, double len_y,
                                       double second_section_height,
                                       double second_section, double max_depth,
                                       double bottom_search_offset,
                                       double dispense_offset) {
    std::string cmd = cmdHeader("GC") + \
                      "ge" + zfill(std::to_string(index), 2) + \
                      "ca" + zfill(std::to_string(geometry), 1) + \
                      "cb" + zfill(std::to_string(static_cast<unsigned int>(diameter * 10)), 3) + \
                      "cc" + zfill(std::to_string(static_cast<unsigned int>(len_x * 10)), 3) + \
                      "cd" + zfill(std::to_string(static_cast<unsigned int>(len_y * 10)), 3) + \
                      "bg" + zfill(std::to_string(static_cast<unsigned int>(second_section_height * 10)), 4) + \
                      "gx" + zfill(std::to_string(static_cast<unsigned int>(second_section)), 5) + \
                      "ce" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(max_depth, TIP_INSTALLED_OFFSET))), 4) + \
                      "ch" + zfill(std::to_string(static_cast<unsigned int>(bottom_search_offset * 10)), 4) + \
                      "ci" + zfill(std::to_string(static_cast<unsigned int>(dispense_offset * 10)), 4);
    return sendCommand(cmd);
}

bool ZeusModule::getContainerGeometryParams(unsigned int index) {
    if(index > 99) {
        printf("ERROR: Requested deck geometry table index is out of range.\n");
        return false;
    }
    std::string cmd = cmdHeader("GB") + \
                      "ge" + zfill(std::to_string(index), 2);
    sendCommand(cmd);
    return true;

}

bool ZeusModule::setLiquidClass(unsigned int index) {
    LiquidClass l = *(_liquid_classes[index]);
    std::string cmd = cmdHeader("GL") + \
                      "lq" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2) + \
                      "uu" + zfill(std::to_string(l.getAspirateTypeRef()), 1) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getAspirateSpeedRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getPrewetVolumeRef() * 10)), 4) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getAspirateTransportAirVolumeRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getAspirateBlowoutVolumeRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getAspirateSwapSpeedRef() * 10)), 4) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getAspirateSettlingTimeRef() * 10)), 3) + " " + \
                      zfill(std::to_string(l.getLLDModeRef()), 1) + " " + \
                      zfill(std::to_string(l.getCLLDSensitivityRef()), 1) + " " + \
                      zfill(std::to_string(l.getPLLDSensitivityRef()), 1) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getLLDHeightDifferenceRef() * 10)), 2) + " " + \
                      zfill(std::to_string(l.getADCRef()), 1) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getImmersionDepthRef() * 10)), 4) + " " + \
                      zfill(std::to_string(l.getImmersionDirectionRef()), 1) + " " + \
                      zfill(std::to_string(l.getDispenseTypeRef()), 1) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getDispenseSpeedRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getCutoffSpeedRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getStopBackVolumeRef() * 10)), 3) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getDispenseTransportAirVolumeRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getDispenseBlowoutVolumeRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getDispenseSwapSpeedRef() * 10)), 4) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getDispenseSettlingTimeRef() * 10)), 3) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getTransportSpeedRef() * 10)), 5) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getLeavingHeightRef() * 10)), 3) + " " + \
                      zfill(std::to_string(static_cast<unsigned int>(l.getDispenseHeightRef() * 10)), 3);
    sendCommand(cmd);

    // Set and request liquid class parameters
    cmd = cmdHeader("GM") + "lq" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2);
    sendCommand(cmd);

    // Set and request QPM parameters for aspiration
    cmd = cmdHeader("GQ") + "gv" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2) + "vv0310 0005 0010 0 0770 0005 0015 1 0713 0005 0015 1 0615 0005 0015 1 0081 0000 0015 0 0531 0005 0005 1 0570 0005 0015 1 0485 0005 0010 1 0010 0000 0000";
    sendCommand(cmd);
    cmd = cmdHeader("GS") + "gv" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2);
    sendCommand(cmd);

    // Set and request QPM parameters for dispensing
    cmd = cmdHeader("GV") + "gp" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2) + "ww0300 0015 1 0046 0010 1 0420 0015 1 0500 0015 1 0550 0005 1 0640 0015 1 0644 0015 1 0662 0015 1 0000 0000";
    sendCommand(cmd);
    cmd = cmdHeader("GW") + "gp" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2);
    sendCommand(cmd);

    // Set and request gravimetric calibration parameters for aspiration
    cmd = cmdHeader("GG") + "gg" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2) + "ck00050 00065 00100 00220 00200 00280 00500 00551 00750 08160 00100 01020 01500 01614 02000 02151";
    sendCommand(cmd);
    cmd = cmdHeader("GE") + "gg" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2);
    sendCommand(cmd);

    // Set and request gravimetric calibration parameters for aspiration
    cmd = cmdHeader("GH") + "gh" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2) + "cl00050 00065 00100 00220 00200 00280 00500 00551 00750 08160 00100 01020 01500 01614 02000 02151";
    sendCommand(cmd);
    cmd = cmdHeader("GI") + "gh" + zfill(std::to_string(index + LIQUID_CLASS_OFFSET), 2);
    return sendCommand(cmd);
}

// LiquidClass *ZeusModule::getLiquidClass(unsigned int index){
// return _liquid_classes.at(index + LIQUID_CLASS_OFFSET);
// }

bool ZeusModule::getLiquidClassParams(unsigned int index) {
    if(index > 99) {
        printf("ERROR: Requested deck geometry table index is out of range.\n");
        return false;
    }
    std::string cmd = cmdHeader("GM") + \
                      "iq" + zfill(std::to_string(index), 2);
    sendCommand(cmd);
    return false;
}

// bool ZeusModule::getLiquidLevel(unsigned int cg_idx, unsigned int dg_idx,
// unsigned int lc_idx, )

bool ZeusModule::pickUpTip(unsigned int tt_idx, unsigned int dg_idx, bool speed) {
    if((tt_idx > 9) || (dg_idx > 99)) {
        // ROS_ERROR_STREAM("Tip type index (" << tt_idx << ") or deck geometry index (" << dg_idx << ") out of range.")
        return false;
    }
    // PRINT_DEBUG("LIBZEUS: PipetterControllerNode: Picking up tip with tt_idx " + std::to_string(tt_idx)+ " and dg_idx " + std::to_string(dg_idx));
    std::string cmd = cmdHeader("GT");
    cmd += "tt" + zfill(std::to_string(tt_idx), 1) + \
           "go" + zfill(std::to_string(dg_idx), 2) + \
           "mt" + std::to_string(static_cast<unsigned int>(speed));
    return sendCommand(cmd);

    /* Here we should wait for the pipetter to indicate that a tip has been
     * picked up before returning. */
    // return true;
    // if(!getTipStatus() ) {
    // return false;
    // }
}

bool ZeusModule::ejectTip(void) {
    std::string cmd = cmdHeader("GU");
    cmd += "go" + zfill(std::to_string(_current_dg_index), 2);
    /* TODO: nam - Check if has been successfully discarded
     * before returning. - Thu 24 Aug 2017 04:34:57 PM MDT */
    return sendCommand(cmd);
}

bool ZeusModule::home(bool init_z, bool init_dosing) {
    // moveZ(TO_ZEUS_COORDS(0, 0,),1);
    if(init_z == true) {
        initZDrive();
    }
    if(init_dosing == true) {
        initDosingDrive();
    }
    return true;
}

bool ZeusModule::aspirate(double vol, unsigned int gc_idx, unsigned int dg_idx,
                          unsigned int lc_idx, double liquid_surface) {

    if((vol < 0) || (vol > 11000) || (liquid_surface < 0) || (liquid_surface > 1800)) {
        //throw error
        return false;
    }

    std::string cmd = cmdHeader("GA");
    cmd += "ai" + zfill(std::to_string(static_cast<unsigned int>(vol*10)), 5) +\
           "ge" + zfill(std::to_string(gc_idx), 2) +\
           "go" + zfill(std::to_string(dg_idx), 2) +\
           /*"lq" + zfill(std::to_string(lc_idx), 2) +\ */
           "lq" + zfill(std::to_string(lc_idx + LIQUID_CLASS_OFFSET), 2) +\
           "gq" + std::to_string(_qpm) +\
           "lb" + std::to_string(_lld_active) +\
           "zp" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(66, TIP_INSTALLED_OFFSET))), 4) +\
           "cg" + zfill(std::to_string(static_cast<unsigned int>(_check_height* 10)), 4) + \
           "cf" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(liquid_surface, TIP_INSTALLED_OFFSET))), 4);

    return sendCommand(cmd);
}

bool ZeusModule::dispense(double vol, unsigned int gc_idx, unsigned int dg_idx,
                          unsigned int lc_idx, double liquid_surface) {
    std::string cmd = cmdHeader("GD");
    cmd += "di" + zfill(std::to_string(static_cast<unsigned int>(vol*10)), 5) +\
           "ge" + zfill(std::to_string(gc_idx), 2) +\
           "go" + zfill(std::to_string(dg_idx), 2) +\
           "lq" + zfill(std::to_string(lc_idx + LIQUID_CLASS_OFFSET), 2) +\
           /*"lq" + zfill(std::to_string(lc_idx), 2) +\*/
           "gq" + std::to_string(_qpm) +\
           "lb" + std::to_string(_lld_active) +\
           "zp" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(66, TIP_INSTALLED_OFFSET))), 4) +\
           "cf" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(liquid_surface, TIP_INSTALLED_OFFSET))), 4) +\
           "zm" + std::to_string(_search_bottom_mode);
    return sendCommand(cmd);
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
        perror("LIBZEUS: Error while opening socket.\n");
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
        perror("LIBZEUS: - Error while attempting to bind to socket.\n");
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

bool ZeusModule::initDosingDrive(void) {
    std::string cmd = cmdHeader("DI");
    return sendCommand(cmd);
}

bool ZeusModule::initZDrive(void) {
    std::string cmd = cmdHeader("ZI");
    _zpos = ZPOS_MIN;
    return sendCommand(cmd);
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

bool ZeusModule::getFirmwareVersion(void) {
    // std::string cmd = cmdHeader("RF");
    sendCommand("RF");
    return true;
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

std::string ZeusModule::waitForResponse(void) {
    time_t start = time(NULL);
    while((time(NULL) - start) < 10) {
        if(_msg_ready_flag == 1) {
            setMsgReadyFlag(0);
            std::string ret = parseErrors(_received_msg);
            PRINT_DEBUG("LIBZEUS: Received message: " + _received_msg);
            // setReceivedMsg(std::string(""));
            // if(ret.find("GL") != std::string::npos){
            // std::string cmd = cmdHeader("VP");
            // sendCommand(cmd);
            // }
            return ret;
        }
    }
    PRINT_ERROR("LIBZEUS: Timeout in waitForResponse()");
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

bool ZeusModule::sendCommand(std::string cmd) {
    /* At the end of this function, we will block until a return string is sent
     * from the pipetter to avoid sending simultaneous commands. */
    _ready_for_new_command = 0;
    setMsgReadyFlag(0);
    PRINT_DEBUG("LIBZEUS: Sending command: " + cmd);
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
            std::string res = waitForResponse();
            if(res.find("NONE") != std::string::npos) {
                usleep(100000);
                _ready_for_new_command = 1;
                return true;
                // } else if(res.find("er") != std::string::npos) {
            } else {
                usleep(100000);
                _ready_for_new_command = 1;
                std::cout << "LIBZEUS:: WaitForResponse: Returned error \"" << res << "\"" << std::endl;
                return false;
            }
        }
        waitForRemoteFrame();
    }
}

bool ZeusModule::getTipStatus(void) {
    unsigned int timeout = 5;
    std::string cmd = cmdHeader("RT");
    setWaitingForMsgFlag(1);
    sendCommand(cmd);
    unsigned int start = time(NULL);
    while(getWaitingForMsgFlag() == 1 ) {
        if((time(NULL) - start) > timeout) {
            PRINT_ERROR("LIBZEUS::getTipStatus: Timeout waiting for response.");
            return 0;
        }
    }
    std::string ret = getReceivedMsg().substr(getReceivedMsg().find("rt") + 2, 1);

    return stoi(ret);
}

double ZeusModule::getZPos(void) {
    std::string cmd = cmdHeader("RZ");
    setWaitingForMsgFlag(1);
    sendCommand(cmd);
    while(getWaitingForMsgFlag() == 1) {}
    std::string z_str = getReceivedMsg().substr(getReceivedMsg().find("gy")+2, 3);
    // printf("ZPOS is: %d\n", stoi(z_str));
    return stod(z_str) * 0.1;
}

bool ZeusModule::emergencyStop(bool state) {
    std::string cmd;
    if(state == true) {
        cmd = cmdHeader("AB");
    } else {
        cmd = cmdHeader("AW");
    }
    return sendCommand(cmd);
}

bool ZeusModule::getContainerVolume(unsigned int gc_idx, unsigned int dg_idx,
                                    unsigned int lc_idx, double lld_search_pos,
                                    double liquid_surface, double &vol_result,
                                    double &level_result) {
    PRINT_DEBUG("LIBZEUS: Entered getContainerVolume()");
    bool ret = false;
    std::string cmd = cmdHeader("GJ");
    cmd += "ge" + zfill(std::to_string(gc_idx), 2) +\
           "go" + zfill(std::to_string(dg_idx), 2) +\
           "lq" + zfill(std::to_string(lc_idx + LIQUID_CLASS_OFFSET), 2) +\
           "lb" + zfill(std::to_string(static_cast<unsigned int>(_lld_active)), 1) +\
           "zp" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(lld_search_pos, TIP_INSTALLED_OFFSET))), 4) +\
           "cf" + zfill(std::to_string(static_cast<unsigned int>(TO_ZEUS_COORDS(liquid_surface, TIP_INSTALLED_OFFSET))), 4);
    ret = sendCommand(cmd);
    if(ret == true) {
        vol_result = stod(getReceivedMsg().substr(getReceivedMsg().find("aw") + 2, 5)) * 0.1;
        level_result = TO_DECK_COORDS(stod(getReceivedMsg().substr(getReceivedMsg().find("yl") + 2, 3)), TIP_INSTALLED_OFFSET);
    }
    PRINT_DEBUG("LIBZEUS: Leaving getContainerVolume()");
    return ret;
}

// bool ZeusModule::setDeckGeometryParams(struct deck_geometry_t d) {
// if((d.index > 9999) || (d.min_traverse_height > 1800) || (d.min_z_pos > 1800)
// || (d.botpp > 1800) || (d.eotpp > 1800) || (d.potdp > 1800)) {
// printf("ERROR: Deck geometry definition contains out of range\
// parameter.\n");
// return false;
// }
// std::string cmd = cmdHeader("GO") +
// "go" + zfill(std::to_string(d.index), 2) + \
// "th" + zfill(std::to_string(d.min_traverse_height), 4) + \
// "te" + zfill(std::to_string(d.min_z_pos), 4) + \
// "tm" + zfill(std::to_string(d.botpp), 4) + \
// "tn" + zfill(std::to_string(d.eotpp), 4) + \
// "tr" + zfill(std::to_string(d.potdp), 4);
// sendCommand(cmd);
// return true;
// }


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
        // return cmd + ": " + error;
        return cmd + ": " + "NONE";
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
