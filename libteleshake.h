/*************************************************************************
Title:    libteleshake.h - CDXBot Shaker Controller Driver for Thermo Scientific
                           Teleshake module.
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libteleshake.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:
    This driver expects the teleshake module to be available at /dev/teleshakeN,
    where N is the device address. Therefore, an appropriate udev rule must be
    set for the USB<->RS232 converter used between the host PC and the module.
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
#include <iomanip>
#include <iostream>
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
#include "ShakerModule.h"
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define PRINT_OUTPUT 0
/* All CB_****_MASK definitions must be ORed with the device address */
#define CB_ADDRESS_MASK         0b01100000
#define CB_BROADCAST_ADDRESS    0b01101111
#define CMD_CLOSE_CLAMP         0x58
#define CMD_GET_CYCLE_TIME      0x32
#define CMD_GET_INFO            0x23
#define CMD_GET_LAST_ERROR      0x25
#define CMD_GET_NUM3            0x3d
#define CMD_GET_NUM_SEQ1        0x3a
#define CMD_GET_NUM_SEQ2        0x3b
#define CMD_GET_POWER           0x3f
#define CMD_GET_SENCE1          0x36
#define CMD_GET_SENCE2          0x37
#define CMD_OPEN_CLAMP          0x57
#define CMD_QUERY_ALL           0x20
#define CMD_RESET_ALL           0x21
#define CMD_RESET_DEVICE        0x22
#define CMD_RESET_PROG          0x51
#define CMD_SET_CYCLE_TIME      0x33
#define CMD_SET_NUM3            0x3c
#define CMD_SET_NUM_SEQ1        0x38
#define CMD_SET_NUM_SEQ2        0x39
#define CMD_SET_POWER           0x3e
#define CMD_SET_PROG            0x50
#define CMD_SET_SENCE1          0x34
#define CMD_SET_SENCE2          0x35
#define CMD_START_DEVICE        0x30
#define CMD_STOP_DEVICE         0x31

#define ERR_BUFFER_OVERFLOW     1
#define ERR_CMD_NOT_ALLOWED     14
#define ERR_CRC                 4
#define ERR_DEVICE_ALREADY_OFF  10
#define ERR_DEVICE_ALREADY_ON   9
#define ERR_DEV_ADDR_MISMATCH   5
#define ERR_N1_OVERFLOW         11
#define ERR_N2_OVERFLOW         12
#define ERR_N3_OVERFLOW         13
#define ERR_NOT_INITIALIZED     7
#define ERR_NOT_IN_REMOTE_MODE  8
#define ERR_NO_STOP_BIT         2
#define ERR_TIMEOUT             6
#define ERR_UNKNOWN_COMMAND     3

#define MASK_ADDR0_BIT          0b00000001
#define MASK_ADDR1_BIT          0b00000010
#define MASK_ADDR2_BIT          0b00000100
#define MASK_ADDR3_BIT          0b00001000
#define MASK_DIRTY_BIT          0b00100000
#define MASK_ERROR_BIT          0b00010000
#define MASK_LEN_BIT            0b10000000
#define MASK_MODE_BIT           0b01000000

#define SHAKE_FREQUENCY_MIN     100
#define SHAKE_FREQUENCY_MAX     2000.0
#define HAS_ERROR(X) (((X) & MASK_ERROR_BIT == 1) ? 1 : 0)

/* MACROS FOR OPERATING ON CONTROL BYTES */
#define CTRL_GET_ADDR0(X) (((X) &= (1UL << 7)) ? 1 : 0)
#define CTRL_GET_ADDR1(X) (((X) &= (1UL << 6)) ? 1 : 0)
#define CTRL_GET_ADDR2(X) (((X) &= (1UL << 5)) ? 1 : 0)
#define CTRL_GET_ADDR3(X) (((X) &= (1UL << 4)) ? 1 : 0)
#define CTRL_GET_DIRTY(X) (((X) &= (1UL << 2)) ? 1 : 0)
#define CTRL_GET_ERROR(X) (((X) &= (1UL << 3)) ? 1 : 0)
#define CTRL_GET_LEN(X) (((X) &= 1UL) ? 1 : 0)
#define CTRL_GET_MODE(X) (((X) &= (1UL << 1)) ? 1 : 0)
#define CTRL_SET_ADDR0(X) ((X) |= (1UL << 7))
#define CTRL_SET_ADDR1(X) ((X) |= (1UL << 6))
#define CTRL_SET_ADDR2(X) ((X) |= (1UL << 5))
#define CTRL_SET_ADDR3(X) ((X) |= (1UL << 4))
#define CTRL_SET_DIRTY(X) ((X) |= (1UL << 2))
#define CTRL_SET_ERROR(X) ((X) |= (1UL << 3))
#define CTRL_SET_LEN(X) ((X) |= (1UL))
#define CTRL_SET_MODE(X) ((X) |= (1UL << 1))


typedef struct {
    // control_byte_t control;
    uint8_t control              = 0;
    uint8_t command              = 0;
    uint8_t data_2               = 0;
    uint8_t data_1               = 0;
    uint8_t data_0               = 0;
    uint8_t checksum             = 0;
} command_telegram_t;

typedef struct {
    int usbfd;
    uint8_t *buffer;
    double timeout = 0.0;
    // struct sockaddr_in remote;
} thread_params_t;

bool ready_flag = 1;
// std::mutex ready_flag_mutex;
/* The 'extern "C"' keyword must be used here to force the compiler to use
 * C rather than C++ linkage. Otherwise, the compiler mangles the symbol
 * name and causes dlsym to not be able to locate any symbols in the library.*/
extern "C" {
    class TeleshakeModule : public ShakerModule {
      public:
        TeleshakeModule (void);
        virtual ~TeleshakeModule ();
        int init(void);
        int deinit(void);
        int lconf(void);
        void seterrfunc(void(*ef)(std::string s));
        uint8_t getPower(void);
        unsigned int queryAll();
        bool resetAll();
        bool reset();
        bool setFrequency(unsigned int f);
        bool setPower(float percent);
        bool start();
        bool stop();
      private:
        command_telegram_t readResponse(void);
        int sendCommand(command_telegram_t t);
        unsigned int getLastError(void);
        void calc_checksum(command_telegram_t *t);
        void parseErrors(unsigned int e);
        void waitForOK(void);
        void waitForString(std::string s, unsigned int timeout = 10);
        void setDeviceAddr(uint8_t addr) {
            /* Zero out upper four bits of address variable for safety */
            _device_addr = addr & 0b00001111;
        };
        uint8_t getDeviceAddr() {
            return _device_addr;
        };
        bool _ready_flag = 1;
        double _netTimeoutMS = 200.0;
        int _usb_baud = B9600;
        int _usbfd = 0;
        long d2b(long n);
        pthread_t _thread_id;
        std::string _usb_addr = "/dev/serial/by-id/usb-FTDI_UT232R_FTYO7XCW-if00-port0";
        thread_params_t _thread_params;
        uint8_t _buffer[6];
        uint8_t _device_addr = 0;
    };

}

/***********************    FUNCTION PROTOTYPES    ***********************/
