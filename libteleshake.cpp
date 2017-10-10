/************************************************************************
Title:    libteleshake.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libramps.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file libtelelshake.h.

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
#include "libteleshake.h"


/*********************    CONSTANTS AND MACROS    **********************/
#define KBLU  "\x1B[34m""]"
#define KNRM  "\x1B[0m""]"
/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   maker()
* Purpose  :   Returns a pointer to a new ShakerModule instance
* Input    :   void
* Returns  :   ShakerModule*
*************************************************************************/
extern "C" ShakerModule *create(void) {
    return new TeleshakeModule;
}

extern "C" void destroy(ShakerModule *sc) {
    delete sc;
}

extern "C" void *thread_func(void *arg) {
    int n = 0;
    std::string s;
    uint8_t buffer[6];
    std::memset(buffer, 0, 6);
    thread_params_t *tp = (thread_params_t *)arg;
    while(1) {
        n = 0;
        std::memset(buffer, 0, 6);
        if((n = read(tp->usbfd, buffer, 6) > 0)) {
            for(unsigned int i = 0; i < 6; i++) {
                s += std::to_string(buffer[i]);
            }
            if(s.find("\n") != std::string::npos)  {
                printf("%s%s%s\n", KBLU, s.c_str(), KNRM);
                s.clear();
            }
        }
    }
}


TeleshakeModule::TeleshakeModule() {}
TeleshakeModule::~TeleshakeModule() {}

int TeleshakeModule::init(void) {
    /* Open USB Device */
    _usbfd = open(_usb_addr.c_str(), O_RDWR | O_NOCTTY);
    if(_usbfd < 0) {
        PRINT_ERROR(std::string(__FILE__) + " " + std::string(__PRETTY_FUNCTION__) + " Error: " + std::to_string(errno) + " opening " + _usb_addr + " : " + strerror(errno));
        return -1;
    }
    PRINT_DEBUG("LIBTELESHAKE: Opened serial connection to USB device at " + _usb_addr + " with file descriptor " + std::to_string(_usbfd));
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(_usbfd, &tty) != 0) {
        PRINT_ERROR(std::string(__FILE__) + " " + std::string(__PRETTY_FUNCTION__) + " Error: " + std::to_string(errno) + " from tcgetattr: " + strerror(errno));
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
        PRINT_ERROR(std::string(__FILE__) + " " + std::string(__PRETTY_FUNCTION__) + "Error: " + std::to_string(errno) + " from tcsetattr: " + strerror(errno));
        return -1;
    }

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
    // resetAll();
    // usleep(100000);
    queryAll();
    return 0;
}

int TeleshakeModule::deinit(void) {
    pthread_cancel(_thread_id);
    pthread_join(_thread_id, NULL);
    close(_usbfd);
    PRINT_DEBUG("LIBTELESHAKE: Closed socket connection to hardware.");
    PRINT_DEBUG("LIBTELESHAKE: Successfully shut down driver.");
    return 0;
}

int TeleshakeModule::lconf(void) {

}


unsigned int TeleshakeModule::queryAll(void) {
    command_telegram_t t;
    t.control = CB_BROADCAST_ADDRESS;
    t.command = CMD_QUERY_ALL;
    sendCommand(t);
    t = readResponse();
    if(t.data_0 > 0) {
        PRINT_DEBUG("LIBTELESHAKE: Located " + std::to_string(t.data_0) + " shaker module(s)");
        _device_addr = t.data_0;
        return true;
    } else {
        PRINT_DEBUG("LIBTELESHAKE: Unable to locate shaker module(s)");
        return false;
    }
}

bool TeleshakeModule::resetAll(void) {
    command_telegram_t t;
    t.control = CB_BROADCAST_ADDRESS;
    t.command = CMD_RESET_ALL;
    sendCommand(t);
    t = readResponse();
    usleep(55 * 100000); /* Wait 5.5 seconds for module to reset */
    return true;
}

bool TeleshakeModule::reset(void) {
    command_telegram_t t;
    t.control = CB_ADDRESS_MASK | _device_addr;
    t.command = CMD_RESET_DEVICE;
    sendCommand(t);
    t = readResponse();
    usleep(55 * 100000); /* Wait 5.5 seconds for module to reset */
    return true;
}

bool TeleshakeModule::start(void) {
    command_telegram_t t;
    t.control = CB_ADDRESS_MASK | _device_addr;
    t.command = CMD_START_DEVICE;
    sendCommand(t);
    t = readResponse();
    return true;
}

bool TeleshakeModule::stop(void) {
    command_telegram_t t;
    t.control = CB_ADDRESS_MASK | _device_addr;
    t.command = CMD_STOP_DEVICE;
    sendCommand(t);
    t = readResponse();
    return true;
}

bool TeleshakeModule::setFrequency(unsigned int f) {
    command_telegram_t t;
    std::string s = "LIBTELESHAKE: Commanded frequency " + \
                    std::to_string(f) + " is out of range. Setting frequency \
                    to ";
    t.control = CB_ADDRESS_MASK | _device_addr;
    t.command = CMD_SET_CYCLE_TIME;
    if(f < SHAKE_FREQUENCY_MIN ) {
        s += " " + std::to_string(SHAKE_FREQUENCY_MIN);
        PRINT_WARNING(s);
        f = SHAKE_FREQUENCY_MIN;
    } else if(f >  SHAKE_FREQUENCY_MAX) {
        s += " " + std::to_string(SHAKE_FREQUENCY_MAX);
        PRINT_WARNING(s);
        f = SHAKE_FREQUENCY_MAX;
    }
    /* Convert frequency to cycle time. Round to nearest integer */
    unsigned int c_time = (unsigned int)((60000000.0f / (float)f) + 0.5f);
    /* data_2, data_1, data_0 represent the cycle time as a 24-bit integer
     * Here, we  split the 32-bit c_time variable into three 8-bit values.*/
    t.data_2 = (uint8_t)(c_time >> 16);
    t.data_1 = (uint8_t)(c_time >> 8);
    t.data_0 = (uint8_t)(c_time);
    PRINT_DEBUG("LIBTELESHAKE: Setting Teleshake vibratory frequency to: " + f);
    sendCommand(t);
    t = readResponse();
    return true;
}

bool TeleshakeModule::setPower(float percent) {
    command_telegram_t t;
    float pwr = 255.0f;
    std::string s = "LIBTELESHAKE: Commanded power level percentage " + \
                    std::to_string(percent) + " is out of range. Setting power level \
                    to ";
    t.control = CB_ADDRESS_MASK | _device_addr;
    t.command = CMD_SET_POWER;
    if(percent < 0.0f) {
        s += " " + std::to_string(0.0f);
        PRINT_WARNING(s);
        percent = 0.0f;
    } else if(percent > 100.0f) {
        s += " " + std::to_string(100.0f);
        PRINT_WARNING(s);
        percent = 100.0f;
    }
    pwr *= percent;
    t.data_0 = (uint8_t)(pwr + 0.5f);
    sendCommand(t);
    t = readResponse();
    return true;
}

uint8_t TeleshakeModule::getPower(void) {
    command_telegram_t t;
    t.control = CB_ADDRESS_MASK | _device_addr;
    t.command = CMD_GET_POWER;
    sendCommand(t);
    t = readResponse();
    return t.data_0;
}

void TeleshakeModule::calc_checksum(command_telegram_t *t) {
    unsigned int n = 0;
    n += t->control;
    n += t->command;
    n += t->data_2;
    n += t->data_1;
    n += t->data_0;
    n %= 8;
    t->checksum = n;
}

unsigned int TeleshakeModule::getLastError(void) {
    command_telegram_t t;
    t.control = (CB_ADDRESS_MASK | _device_addr);
    t.command = CMD_GET_LAST_ERROR;
    sendCommand(t);
    t = readResponse();
    return t.data_0;
}

void TeleshakeModule::parseErrors(unsigned int e) {
    std::string s = "LIBTELESHAKE: Returned error - ";
    switch (e) {
    case ERR_BUFFER_OVERFLOW:
        s += "Buffer overflow.";
        break;
    case ERR_CMD_NOT_ALLOWED:
        s += "Command not allowed.";
        break;
    case ERR_CRC:
        s += "Invalid checksum";
        break;
    case ERR_DEVICE_ALREADY_OFF:
        s += "Device already off.";
        break;
    case ERR_DEVICE_ALREADY_ON:
        s += "Device already on.";
        break;
    case ERR_DEV_ADDR_MISMATCH:
        s += "Device address mismatch";
        break;
    case ERR_N1_OVERFLOW:
        s += "N1 overflow.";
        break;
    case ERR_N2_OVERFLOW:
        s += "N2 overflow.";
        break;
    case ERR_N3_OVERFLOW:
        s += "N3 overflow.";
        break;
    case ERR_NOT_INITIALIZED:
        s += "Module is not initialized.";
        break;
    case ERR_NOT_IN_REMOTE_MODE:
        s += "Module is not in remote mode.";
        break;
    case ERR_NO_STOP_BIT:
        s += "No stop bit.";
        break;
    case ERR_TIMEOUT:
        s += "Timeout.";
        break;
    case ERR_UNKNOWN_COMMAND:
        s += "Unknown command received.";
        break;
    default:
        s += "Module returned unknown error code '" + std::to_string(e) +"'";
        break;
    }
    PRINT_ERROR(s);
}

int TeleshakeModule::sendCommand(command_telegram_t t) {
    int n = 0;
    uint8_t out[6];
    calc_checksum(&t);

    out[0] = (uint8_t)t.control;
    out[1] = (uint8_t)t.command;
    out[2] = (uint8_t)t.data_2;
    out[3] = (uint8_t)t.data_1;
    out[4] = (uint8_t)t.data_0;
    out[5] = (uint8_t)t.checksum;
    // PRINT_DEBUG("\nSent command telegram with data:");
    // PRINT_DEBUG("\t" + std::setfill('0') + std::setw(8) + d2b(t.control)  + " (" + (int)t.control + "),");
    // PRINT_DEBUG("\t" + std::setfill('0') + std::setw(8) + d2b(t.command)  + " (" + (int)t.command + "),");
    // PRINT_DEBUG("\t" + std::setfill('0') + std::setw(8) + d2b(t.data_2)   + " (" + (int)t.data_2 + "),");
    // PRINT_DEBUG("\t" + std::setfill('0') + std::setw(8) + d2b(t.data_1)   + " (" + (int)t.data_1 + "),");
    // PRINT_DEBUG("\t" + std::setfill('0') + std::setw(8) + d2b(t.data_0)   + " (" + (int)t.data_0 + "),");
    // PRINT_DEBUG("\t" + std::setfill('0') + std::setw(8) + d2b(t.checksum) + " (" + (int)t.checksum + ")");
    n = write(_usbfd, out, 6*sizeof(uint8_t));
    PRINT_DEBUG("LIBTELESHAKE: Sent " + std::to_string(n) +  " characters.");
    usleep(500); // Wait 500 us for characters to transmit
    usleep(300000); // Wait 200ms for shaker to interpret command
    return n;
}

command_telegram_t TeleshakeModule::readResponse(void) {
    int n = 0;
    boost::timer tmr;
    command_telegram_t t;
    bool success = false;
    time_t start = time(NULL);

    while((time(NULL) - start) < 2) {
        if((n = read(_usbfd, _buffer, 6)) > 0) {
            // PRINT_DEBUG("Received " + std::string(_buffer));
            success = true;
            break;
        }
        if(n < 0) {
            PRINT_ERROR(std::string(__FILE__) + " " + std::string(__PRETTY_FUNCTION__) +" Socket read error.");
        }
    }
    if(success) {
        t.control  = _buffer[0];
        t.command  = _buffer[1];
        t.data_2   = _buffer[2];
        t.data_1   = _buffer[3];
        t.data_0   = _buffer[4];
        t.checksum = _buffer[5];
        // if(PRINT_OUTPUT) {
            // std::cout << "\nReceived command telegram with data:" << std::endl;
            // std::cout << "\t" << std::setfill('0') << std::setw(8) << d2b(t.control)  << " (" << (int)t.control << "),\n";
            // std::cout << "\t" << std::setfill('0') << std::setw(8) << d2b(t.command)  << " (" << (int)t.command << "),\n";
            // std::cout << "\t" << std::setfill('0') << std::setw(8) << d2b(t.data_2)   << " (" << (int)t.data_2 << "),\n";
            // std::cout << "\t" << std::setfill('0') << std::setw(8) << d2b(t.data_1)   << " (" << (int)t.data_1 << "),\n";
            // std::cout << "\t" << std::setfill('0') << std::setw(8) << d2b(t.data_0)   << " (" << (int)t.data_0 << "),\n";
            // std::cout << "\t" << std::setfill('0') << std::setw(8) << d2b(t.checksum) << " (" << (int)t.checksum << ")" << std::endl;
        // }
        if(HAS_ERROR(t.control)) {
            parseErrors(getLastError());
        }
    } else {
        PRINT_ERROR("LIBTELESHAKE: Error - Timeout waiting for response from Teleshake module.");
    }
    return t;
}


long TeleshakeModule::d2b(long n) {
    int remainder;
    long binary = 0, i = 1;

    while(n != 0) {
        remainder = n%2;
        n = n/2;
        binary= binary + (remainder*i);
        i = i*10;
    }
    return binary;
}
