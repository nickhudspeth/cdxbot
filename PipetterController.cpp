/************************************************************************
Title:    PipetterController.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     PipetterController.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file PipetterController.h.

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
#include "PipetterController.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/

int PipetterController::loadDriver(std::string file) {

    char *error;
    /* ATTEMPT TO LOAD DRIVER FROM FILE */
    dlerror(); // Clear error code
    /* Load without verifying symbols so that symbols may be resolved
     * manually below. */
    _driver_handle = dlopen(file.c_str(), RTLD_LAZY);
    if(!_driver_handle) {
        /* Handle error */
        ROS_ERROR_STREAM("Failed to open driver at location " << file.c_str());
        ROS_ERROR_STREAM(dlerror());
        return -1;
    }
    /* VERIFY THAT DRIVER PROVIDES ALL THE REQUIRED METHODS */
    dlerror(); // Clear error code
    _driver_init = reinterpret_cast<int(*)()>(dlsym(_driver_handle, "init")); // cast me to fn ptr
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Function driver_init() not found in specified \
                    driver at " << file.c_str());
        ROS_ERROR_STREAM(error);
        return -1;
    }

    dlerror(); // Clear error code
    _driver_deinit = reinterpret_cast<int(*)()>(dlsym(_driver_handle, "deinit")); // cast me to fn ptr
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Function driver_deinit() not found in specified \
                    driver at " << file.c_str());
        ROS_ERROR_STREAM(error);
        return -1;
    }

    dlerror(); // Clear error code
    _driver_lconf = reinterpret_cast<int(*)()>(dlsym(_driver_handle, "lconf")); // cast me to fn ptr
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Function driver_lconf() not found in specified \
                    driver at " << file.c_str());
        ROS_ERROR_STREAM(error);
        return -1;
    }

    dlerror(); // Clear error code
    _driver_seterrfunc = reinterpret_cast<void(*)()>(dlsym(_driver_handle, "seterrfunc")); // cast me to fn ptr
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Function driver_seterrfunc() not found in specified \
                    driver at " << file.c_str());
        ROS_ERROR_STREAM(error);
        return -1;
    }
    return 0; // All required functions exist in the specified driver.
}

double PipetterController::getZPos(){
    return 0;
}
