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

#include <pthread.h>
#include <sys/stat.h>
#include "PipetterModule.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define KICK_MASK 0b10000000000
#define SENDER_ID_MASK 0x03E0
#define RECEIVER_ID_MASK 0x001F
#define EOM_MASK 0b10000000
#define CAN_MSG_DATA 0
#define CAN_MSG_KICK 1

#define NETBUFSIZE 64
#define REMOTE_TIMEOUT 1

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

    typedef struct {
        unsigned int index;
        double diameter;
        double bottom_height;
        double bottom_pos;
        double immersion_depth;
        double leaving_height;
        double jet_height;
        double sohbs; // Start Of Height Bottom Search
        double dhabs; // Dispense Height After Bottom Search
    } container_geometry_t;

    typedef struct {
        unsigned int indes;
        double end_traverse_pos;
        double botpp; // Beginning of Tip Picking Position
        double potdp; // Position of Tip Deposit Process
    } deck_geometry_t;

    typedef struct {
        unsigned int id;
        unsigned int index;
        bool lcfft; // Liquid Class For Filter Tips
        unsigned int aspiration_mode;
        double aspiration_flow_rate;
        double over_aspirated_vol;
        double aspiration_transport_vol;
        double blowout_air_vol;
        double aspiration_swap_speed;
        double aspiration_settling_time;
        double lld;
        double clld_sens;
        double plld_sens;
        double ads;
        double dispensing_mode;
        double dispensing_flow_rate;
        double stop_flow_rate;
        double stop_back_vol;
        double dispensing_trnasport_vol;
        double acceleration;
        double dispensing_swap_speed;
        double dispensing_settling_time;
        double flow_rate_transport_vol;
    } liquid_class_t;


    class remoteFrameListener {
      public:
        remoteFrameListener (void);
        virtual ~remoteFrameListener ();
        std::string on_message_received();
        bool remoteReceived(void);
        void setLastTransmited(struct can_frame lt);
        struct can_frame getLastTransmitted(void);
        bool kickReceived(void);
        bool dataReceived(void);
        bool msgIsLast(struct can_frame f);
        bool parseMsgID(int id, const char frame);
        void setRemoteFlag(bool state);
        void setKickFlag(bool state);
        bool getRemoteFlag(void);
        bool getKickFlag(void);
        struct can_frame getNextMessage(void);
        std::string _received_msg;

      private:
        void *parent;
        bool _remote_flag;
        bool _waiting_for_remote_flag;
        bool kick_flag;
        bool waiting_for_kick_flag;
        bool data_flag;
        bool _msg_complete_flag;
        bool _msg_ready_flag;
        struct can_frame _last_transmitted;
        std::queue<struct can_frame> fifo;
    };

    class ZeusModule : public PipetterModule {
      public:
        int _read_can_port = 1;

        ZeusModule (int id = 1);
        virtual ~ZeusModule ();
        int init(void);
        int deinit(void);
        int lconf(void);
        void seterrfunc(void(*ef)(std::string s)) {
            PRINT_ERROR = ef;
        }
        void moveZDrive(double pos, double vel);
        void pickUpTip(void);
        void discardTip(void);
        void aspirate(double vol);
        void dispense(double vol);
        bool getTipStatus(void);
        void getFirmwareVersion(void);
        double getZPos(void);
        void emergencyStop(void);
        void emergencyStopReset(void);
        void CANOpenPort(void);
        void CANClosePort(void);
        void CANReadPort(void);
        void CANWritePort(void);
        int getSockFD(void) {
            return _sockfd;
        }
        std::queue<struct can_frame>& getFIFO(void) {
            return _fifo;
        }

        void setMsgReadyFlag(int i) {
            _msg_ready_flag = i;
        }

        /* This method is accessed from *thread_func(), so data access
         * must be protected from collisions by a mutex.*/
        void setReceivedMsg(std::string s) {
            pthread_mutex_lock(&_lock_msg);
            _received_msg = s;
            pthread_mutex_unlock(&_lock_msg);
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

        void sendRemoteFrame(unsigned int dlc);
        std::string parseErrors(std::string error);

      private:
        int initCANBus(void);
        int initDosingDrive(void );
        int initZDrive(void);

        void setAutoResponse(void);
        std::string cmdHeader(std::string hdr);
        canid_t assembleIdentifier(unsigned int type);

        bool waitForRemoteFrame(void);
        void sendKickFrame(void);
        bool waitForKickFrame(void);
        void sendDataObject(unsigned int i, unsigned int cmd_len, int data);
        void sendCommand(std::string cmd);
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

        /* data */
        int _id = 1;
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

        int _tt_index = 0;
        int _dg_index = 0;
        int _cgt_index = 0;
        int _dgt_index = 0;
        int _lct_index = 0;
        bool _gpm = 0;
        int _qpm = 0;
        bool _lld = 1;
        unsigned int _lld_search_pos = 0;
        unsigned int _liquid_surface = 500;
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
        std::string _received_msg;
        struct can_frame _last_transmitted;
        std::queue<struct can_frame> _fifo;
        struct can_frame _last_sent_frame;
        pthread_mutex_t _lock_msg;
        pthread_t _thread_id;
        thread_params_t _thread_params;
    };

}
/***********************    FUNCTION PROTOTYPES    ***********************/
