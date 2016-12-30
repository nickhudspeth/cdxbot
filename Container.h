/*************************************************************************
Title:    Container.h - Generic class for pipette containers
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     Container.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:


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

*************************************************************************/
//#pragma once

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
struct container_cell {
    std::string test_type;
    unsigned int used; // BOOL
    double vol;            // Well volume (mL)
    double filled_vol;
    double depth;
};

class Container {
  public:
    Container () {}
    virtual ~Container () {}

    /*************************************************************************
    * Function :   setCellProperties()
    * Purpose  :   What does this function do?
    * Input    :   const unsigned int row,
                   const unsigned int col,
                   const std::string type = ""
    * Returns  :   void
    *************************************************************************/
    void setCellProperties(const unsigned int row,
                           const unsigned int col,
                           const std::string type = ""
                          );

    /*************************************************************************
    * Function :    getCell()
    * Purpose  :   Returns a struct containing the properties of a cell at a
    *              given index.
    * Input    :   const unsigned int row,
                   const unsigned int col
    * Returns  :   struct container_cell
    *************************************************************************/
    struct container_cell getCell(const unsigned int row,
                                  const unsigned int col) {
        return _cells[row][col];
    }

    /*************************************************************************
    * Function :   getGlobalCoords()
    * Purpose  :   Returns the XYZ coordinates for a specified cell
    * Input    :   const char axis,
    *              unsigned int row
    *              unsigned int col
    * Returns  :   double
    *************************************************************************/
    double getGlobalCoords(const char axis, unsigned int row, unsigned int col);

  private:
    double _length;
    double _width;
    double _height;
    unsigned int _rows;
    unsigned int _cols;
    double _row_spacing;
    double _col_spacing;
    std::string _type;      // Type of container.(tip rack, well plate, etc.)
    double _well_depth;     // Well depth measured from top of deck (mm)
    double _offset_x;       // Container origin offset from gantry origin (mm)
    double _offset_y;
    double _offset_z;
    std::vector<std::vector<struct container_cell>> _cells; // Vector of cell properties;
};
