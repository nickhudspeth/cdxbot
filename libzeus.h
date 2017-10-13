/*************************************************************************
Title:    libzeus.h -
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libzeus.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:


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
#include <boost/algorithm/string.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <string>
#include <map>
#include <unistd.h>
#include <vector>
#include <queue>

#include <fcntl.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>

#include <pthread.h>
#include <sys/stat.h>
#include "PipetterModule.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define PRINT_OUTPUT 1


#define KICK_MASK 0b10000000000
#define SENDER_ID_MASK 0x03E0
#define RECEIVER_ID_MASK 0x001F
#define EOM_MASK 0b10000000
#define CAN_MSG_DATA 0
#define CAN_MSG_KICK 1

#define ZPOS_MIN 0
#define ZPOS_MAX 1800
#define NETBUFSIZE 64
#define REMOTE_TIMEOUT 1
#define INIT_SUCCESS 1
#define INIT_FAILURE 0

#define CONTAINER_GEOMETRY_ROUND  0
#define CONTAINER_GEOMETRY_SQUARE 1

#define TOP 0
#define BOTTOM 1

#define MAX_Z_ERROR 0.1

#define LIQUID_CLASS_OFFSET 30
#define TIP_INSTALLED_OFFSET 53

typedef struct {
    int sockfd;
    char *buffer;
    double timeout = 0.0;
    struct sockaddr_can remote;
} thread_params_t;



/* The 'extern "C"' keyword must be used here to force the compiler to use
 * C rather than C++ linkage. Otherwise, the compiler mangles the symbol
 * name and causes dlsym to not be able to locate any symbols in the library.*/
extern "C" {
    struct container_geometry_t {
        unsigned int index;
        bool geometry;
        unsigned int diameter;
        unsigned int x_length;
        unsigned int y_length;
        unsigned int second_height;
        unsigned int second_section;
        unsigned int minimum_height;
        unsigned int sohbs; // Start Of Height Bottom Search
        unsigned int dhabs; // Dispense Height After Bottom Search
    };

    struct deck_geometry_t {
        unsigned int index;
        unsigned int min_traverse_height;
        unsigned int min_z_pos;
        unsigned int botpp; // Beginning of Tip Picking Position
        unsigned int eotpp; // End of Tip Picking Position
        unsigned int potdp; // Position of Tip Deposit Process
    };

    struct liquid_class_t {
        unsigned int id;
        unsigned int index;
        bool lcfft; // Liquid Class For Filter Tips
        unsigned int aspiration_mode;
        unsigned int aspiration_flow_rate;
        unsigned int over_aspirated_vol;
        unsigned int aspiration_transport_vol;
        unsigned int blowout_air_vol;
        unsigned int aspiration_swap_speed;
        unsigned int aspiration_settling_time;
        unsigned int lld;
        unsigned int clld_sens;
        unsigned int plld_sens;
        unsigned int adc;
        unsigned int dispensing_mode;
        unsigned int dispensing_flow_rate;
        unsigned int stop_flow_rate;
        unsigned int stop_back_vol;
        unsigned int dispensing_transport_vol;
        unsigned int acceleration;
        unsigned int dispensing_swap_speed;
        unsigned int dispensing_settling_time;
        unsigned int flow_rate_transport_vol;
    };

    std::string _last_error_msg = "";

    class ZeusModule : public PipetterModule {
      public:
        int _read_can_port = 1;

        ZeusModule (int id = 1);
        virtual ~ZeusModule ();
        int init(void);
        int deinit(void);
        int lconf(void);
        // void seterrfunc(void(*ef)(std::string s)) {
        // PRINT_ERROR = ef;
        // }
        bool getTipStatus(void);
        bool moveZ(double pos, double vel);
        double getZPos(void);
        unsigned int getInitializationStatus(void);
        bool aspirate(double vol, unsigned int gc_idx, unsigned int dg_idx,
                      unsigned int lc_idx, double liquid_surface);
        bool dispense(double vol, unsigned int gc_idx, unsigned int dg_idx,
                      unsigned int lc_idx, double liquid_surface);
        bool ejectTip(void);
        bool home(bool init_z, bool init_dosing);
        bool emergencyStop(void);
        bool emergencyStopReset(void);
        bool getContainerGeometryParams(unsigned int index);
        bool getDeckGeometryParams(unsigned int index);
        bool getFirmwareVersion(void);
        bool getLiquidClassParams(unsigned int index);
        bool makeDeckGeometry(unsigned int index, double feed_plane,\
                              double min_end_cmd_height, \
                              double container_offset_z, \
                              double tip_engagement_len, \
                              double tip_deposit_height);
        bool makeContainerGeometry(unsigned int index, bool geometry,
                                   double diameter, double len_x, double len_y,
                                   double second_section_height,
                                   double second_section, double max_depth,
                                   double bottom_search_offset,
                                   double dispense_offset);
        bool setLiquidClass(unsigned int index);
        bool pickUpTip(unsigned int tt_idx, unsigned int dg_idx, bool speed);
        // bool setContainerGeometryParams(struct container_geometry_t c);
        // bool setDeckGeometryParams(struct deck_geometry_t d);
        bool setLiquidClassParams(struct liquid_class_t l);
        bool getContainerVolume(unsigned int cg_idx, unsigned int dg_idx,
                                unsigned int lc_idx, double lld_search_pos,
                                double liquid_surface, double &vol_result,
                                double &level_result);
        int getSockFD(void) {
            return _sockfd;
        }
        void getLastFaultyParameter(void);
        std::queue<struct can_frame>& getFIFO(void) {
            return _fifo;
        }

        void setMsgReadyFlag(int i) {
            _msg_ready_flag = i;
        }

        /* This method is accessed from *thread_func(), so data access
         * must be protected from collisions by a mutex.*/
        void setReceivedMsg(std::string s) {
            // pthread_mutex_lock(&_lock_msg);
            _received_msg = s;
            // pthread_mutex_unlock(&_lock_msg);
        }
        std::string getReceivedMsg(void) {
            // pthread_mutex_lock(&_lock_msg);
            std::string ret = _received_msg;
            // pthread_mutex_unlock(&_lock_msg);
            return ret;
        }
        void setRemoteFlag(bool state) {
            _remote_flag = state;
        };
        void setKickFlag(bool state) {
            _kick_flag = state;
        };
        bool getRemoteFlag(void) {
            return _remote_flag;
        };
        bool getKickFlag(void) {
            return _kick_flag;
        }
        void setWaitingForMsgFlag(bool s) {
            _waiting_for_msg_flag = s;
        }
        bool getWaitingForMsgFlag(void) {
            return _waiting_for_msg_flag;
        }

        void sendRemoteFrame(unsigned int dlc);
        std::string parseErrors(std::string error);

        unsigned int _error_flag = 0;
        bool initDosingDrive(void );
        bool initZDrive(void);
      private:
        int initCANBus(void);

        std::string cmdHeader(std::string hdr);
        canid_t assembleIdentifier(unsigned int type);

        bool waitForRemoteFrame(void);
        void sendKickFrame(void);
        bool waitForKickFrame(void);
        bool sendCommand(std::string cmd);
        int sendFrame(struct can_frame f);

        std::string on_message_received();
        bool remoteReceived(void);
        void setLastTransmited(struct can_frame lt);
        struct can_frame getLastTransmitted(void);
        bool kickReceived(void);
        bool dataReceived(void);
        bool msgIsLast(struct can_frame f);
        bool parseMsgID(int id, const char frame);
        struct can_frame getNextMessage(void);
        void setLastFrame(struct can_frame &f);
        struct can_frame & getLastFrame(void);
        std::string waitForResponse(void);
        /* data */
        const int _id = 1;
        int _sockfd; // SocketCAN file descriptor
        std::string _interface = "can0";
        unsigned int _remote_timeout = 1000;
        unsigned int _transmission_retries = 5;
        std::map<std::string, std::string> _error_table = {
            {"20", "No communication to EEPROM."},
            {"30", "Undefined command."},
            {"31", "Undefined parameter."},
            {"32", "Parameter out of range."},
            {"35", "Voltage outside the permitted range."},
            {"36", "Emergency stop is active or was sent during action."},
            {"38", "Empty liquid class."},
            {"39", "Liquid class write protected."},
            {"40", "Parallel processes not permitted."},
            {"50", "Initialization failed."},
            {"51", "Pipetting drive not initialized."},
            {"52", "Movement error on pipetting drive."},
            {"53", "Maximum volume of the tip reached."},
            {"54", "Maximum volume in pipetting drive reached."},
            {"55", "Volume check failed."},
            {"56", "Conductivity check failed."},
            {"57", "Filter check failed."},
            {"60", "Initialization failed."},
            {"61", "Z-drive is not initialized."},
            {"62", "Movement error on the z-drive."},
            {"63", "Container bottom search failed."},
            {"64", "Z-position not possible."},
            {"65", "Z-position not possible."},
            {"66", "Z-position not possible."},
            {"67", "Z-position not possible."},
            {"68", "Z-position not possible."},
            {"69", "Z-position not possible."},
            {"70", "Liquid level not detected."},
            {"71", "Not enough liquid present."},
            {"72", "Auto calibration of the pressure sensor not possible."},
            {"74", "Early liquid level detection."},
            {"75", "No tip picked up or no tip present."},
            {"76", "Tip already picked up."},
            {"77", "Tip not discarded."},
            {"80", "Clot detected during aspiration."},
            {"81", "Empty tube detected during aspiration."},
            {"82", "Foam detected during aspiration."},
            {"83", "Clot detected during dispensing."},
            {"84", "Foam detected during dispensing."},
            {"85", "No communication to the digital potentiometer."},
        };

        unsigned int _cgt_index = 1;
        unsigned int _lct_index = 1;
        unsigned int _current_dg_index = 0;
        bool _qpm = 0;
        bool _lld = 1;
        unsigned int _lld_search_height = 0;
        unsigned int _check_height = 5;
        int _search_bottom_mode = 0;
        // Mixing parameters
        unsigned int _mix_vol = 0;
        unsigned int _mix_flow_rate = 0;
        unsigned int _mix_cycles = 0;
        unsigned int _master_id = 0;

        void *parent;
        bool _remote_flag = 0;
        bool _waiting_for_remote_flag = 0;
        bool _kick_flag = 0;
        bool waiting_for_kick_flag = 0;
        bool data_flag = 0;
        bool _msg_complete_flag = 0;
        bool _msg_ready_flag = 0;
        bool _waiting_for_msg_flag = 0;

        bool _ready_for_new_command = 1;
        std::string _received_msg;
        struct can_frame _last_transmitted;
        std::queue<struct can_frame> _fifo;
        struct can_frame _last_sent_frame;
        // pthread_mutex_t _lock_msg;
        pthread_t _thread_id;
        thread_params_t _thread_params;
    };
}
/***********************    FUNCTION PROTOTYPES    ***********************/
