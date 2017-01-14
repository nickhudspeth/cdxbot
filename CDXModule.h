/*************************************************************************
Title:    CDXModule.h - CDXBot Module Base Class
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     CDXModule.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    Provides a base class implementation from which all CDXBot hardware drivers
    are derived.

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
#include <stdlib.h>
#include <stdio.h>
#include <string>
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/


/***********************    FUNCTION PROTOTYPES    ***********************/
class CDXModule {
  public:
    CDXModule (void){};
    virtual ~CDXModule (void) {};
    virtual int init(void) {
        return 0;
    }
    virtual int deinit(void) {
        return 0;
    }
    virtual int lconf(void) {
        return 0;
    }
    virtual void seterrfunc(void(*ef)(std::string s)) {
        PRINT_ERROR = ef;
    }
  protected:
    void(*PRINT_ERROR)(std::string s);

};
