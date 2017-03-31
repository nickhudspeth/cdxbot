/************************************************************************
Title:    Container.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     Container.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file Container.h.

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
#include "Container.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/


/*******************    FUNCTION IMPLEMENTATIONS    ********************/


double Container::getGlobalCoords(const char axis, unsigned int row, unsigned int col) {
    // printf("Getting global coords for container axis %c with %d rows and %d cols.\n", axis, _rows, _cols);
    double ret = 0;
    if(row > _rows) {
        printf("ERROR: Specified row index \"%d\" is out of range for container.\n", row);
        // Throw error: specified row index out of range for container.
        return 0;
    }
    if(col > _cols) {
        // Throw error: specified column index out of range for container.
        printf("ERROR: Specified column index \"%d\" is out of range for container.\n", col);
        return 0;
    }

    switch(axis) {
    case 'x':
        ret = _offset_x + row * _row_spacing;
        break;

    case 'y':
        ret = _offset_y + col * _col_spacing;
        break;

    case 'z':
        ret = _height;
        break;

    default:
        // Throw error: unspecified axis selected
        printf("ERROR: Specified axis \"%c\" is invalid.\n", axis);
        break;
    }
    return ret;

}

