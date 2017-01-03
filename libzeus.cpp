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
// #include <string>
//#include <python2.7/Python.h>
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
char python_driver[] = "zeus.py";
//PyObject *pName, *pModule, *pDict, *pFunc;
//PyObject *pArgs, *pValue;
/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   init()
* Purpose  :   Performs all initialization routines for the Zeus Module.
* Input    :   void
* Returns  :   int
*************************************************************************/
int init(void) {
//    Py_Initialize();
//    PyObject * pName = PyString_FromString(python_driver);
//    pModule =  PyImport_Import(pName);
//    Py_DECREF(pName); // We don't need pName anymore here, so free this memory.
//    if (pModule != NULL) {
    /* Do we have to call __init__ manually, or is it done automatically
     * on importing the module? Don't forget to pass the module number
     * to the pipetter.*/
//        pFunc = pyObject_GetAttrString(pModule, "__init__");
//        if (pFunc &&PyCallableCheck(pFunc )) {

//        } else {
    //  COULD NOT FIND FUNCTION. THROW ERROR.
//        }

//    } else {
    // COULD NOT FIND PYTHON MODULE. THROW ERROR.
//    }


}

/*************************************************************************
* Function :   deinit()
* Purpose  :   Performs all shutdown and cleanup routines on unlozding the
*              driver
* Input    :   void
* Returns  :   int
*************************************************************************/
int deinit(void) {

}

/*************************************************************************
* Function :   lconf()
* Purpose  :   Loads all configuration parameters used by the driver.
* Input    :   void
* Returns  :   int
*************************************************************************/
int lconf(void) {

}

/*************************************************************************
* Function :   seterrfunc()
* Purpose  :   Specifies an error reporting function to call to provide
*              a mechanism for the driver to report errors to the system.
* Input    :   void(*ef)(std::string)
* Returns  :   void
*************************************************************************/
void seterrfunc(void(*ef)(std::string s)) {
    PRINT_ERROR = ef;
}
