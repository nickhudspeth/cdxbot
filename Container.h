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
    * Input    :   unsigned int row,
                   unsigned int col,
                   std::string type = ""
    * Returns  :   void
    *************************************************************************/
    void setCellProperties(unsigned int row,
                           unsigned int col,
                           std::string type = ""
                          );

    /*************************************************************************
    * Function :    getCell()
    * Purpose  :   Returns a struct containing the properties of a cell at a
    *              given index.
    * Input    :   unsigned int row,
                   unsigned int col
    * Returns  :   struct container_cell
    *************************************************************************/
    struct container_cell getCell(unsigned int row,
                                  unsigned int col) {
        return _cells[row][col];
    }

    /*************************************************************************
    * Function :   getGlobalCoords()
    * Purpose  :   Returns the XYZ coordinates for a specified cell
    * Input    :   char axis,
    *              unsigned int row
    *              unsigned int col
    * Returns  :   double
    *************************************************************************/
    double getGlobalCoords(char axis, unsigned int row, unsigned int col);

    /*************************************************************************
    * Function :   getType()
    * Purpose  :   Getter for the variable _type
    * Input    :   void
    * Returns  :       std::string
    *************************************************************************/
    std::string getType(void) {
        return _type;
    }

    /*************************************************************************
    * Function :   getTypeRef()
    * Purpose  :   Returns a reference to the variable Container::_type
    * Input    :   void
    * Returns  :   std::string&
    *************************************************************************/
    std::string& getTypeRef (void) {
        return _type;
    }

    /*************************************************************************
    * Function :   setType()
    * Purpose  :   Setter for the variable _type
    * Input    :   std::string t
    * Returns  :   void
    *************************************************************************/
    void setType(std::string t) {
        _type = t;
    }

    /*************************************************************************
    * Function :   getLength()
    * Purpose  :   Getter for the variable _length
    * Input    :   void
    * Returns  :       double
    *************************************************************************/
    double getLength(void) {
        return _length;
    }

    /*************************************************************************
    * Function :   getLengthRef()
    * Purpose  :   Returns a reference to the variable Container::_length
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getLengthRef (void) {
        return _length;
    }

    /*************************************************************************
    * Function :   setLength()
    * Purpose  :   Setter for the variable _length
    * Input    :   double l
    * Returns  :   void
    *************************************************************************/
    void setLength(double l) {
        _length = l;
    }

    /*************************************************************************
    * Function :   getWidth()
    * Purpose  :   Getter for the variable _width
    * Input    :   void
    * Returns  :       double
    *************************************************************************/
    double getWidth(void) {
        return _width;
    }

    /*************************************************************************
    * Function :   getWidthRef()
    * Purpose  :   Returns a reference to the variable Container::_width
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getWidthRef (void) {
        return _width;
    }

    /*************************************************************************
    * Function :   setWidth()
    * Purpose  :   Setter for the variable _width
    * Input    :   double w
    * Returns  :   void
    *************************************************************************/
    void setWidth(double w) {
        _width = w;
    }

    /*************************************************************************
    * Function :   getHeight()
    * Purpose  :   Getter for the variable _height
    * Input    :   void
    * Returns  :       double
    *************************************************************************/
    double getHeight(void) {
        return _height;
    }

    /*************************************************************************
    * Function :   getHeightRef()
    * Purpose  :   Returns a reference to the variable Container::_height
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getHeightRef (void) {
        return _height;
    }

    /*************************************************************************
    * Function :   setHeight()
    * Purpose  :   Setter for the variable _height
    * Input    :   double h
    * Returns  :   void
    *************************************************************************/
    void setHeight(double h) {
        _height = h;
    }

    /*************************************************************************
    * Function :   getRows()
    * Purpose  :   Getter for the variable _rows
    * Input    :   void
    * Returns  :   int
    *************************************************************************/
    int getRows(void) {
        return _rows;
    }

    /*************************************************************************
    * Function :   getRowsRef()
    * Purpose  :   Returns a reference to the variable Container::_rows
    * Input    :   void
    * Returns  :   int&
    *************************************************************************/
    int& getRowsRef (void) {
        return _rows;
    }

    /*************************************************************************
    * Function :   setRows()
    * Purpose  :   Setter for the variable _rows
    * Input    :   unsigned int r
    * Returns  :   void
    *************************************************************************/
    void setRows(unsigned int r) {
        _rows = r;
    }

    /*************************************************************************
    * Function :   getCols()
    * Purpose  :   Getter for the variable _cols
    * Input    :   void
    * Returns  :   int
    *************************************************************************/
    int getCols(void) {
        return _cols;
    }

    /*************************************************************************
    * Function :   getColsRef()
    * Purpose  :   Returns a reference to the variable Container::_cols
    * Input    :   void
    * Returns  :   int&
    *************************************************************************/
    int& getColsRef (void) {
        return _cols;
    }

    /*************************************************************************
    * Function :   setCols()
    * Purpose  :   Setter for the variable _cols
    * Input    :   unsigned int c
    * Returns  :   void
    *************************************************************************/
    void setCols(unsigned int c) {
        _cols = c;
    }

    /*************************************************************************
    * Function :   getRowSpacing()
    * Purpose  :   Getter for the variable _row_spacing
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getRowSpacing(void) {
        return _row_spacing;
    }

    /*************************************************************************
    * Function :   getRowSpacingRef()
    * Purpose  :   Returns a reference to the variable Container::_row_spacing
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getRowSpacingRef (void) {
        return _row_spacing;
    }

    /*************************************************************************
    * Function :   setRowSpacing()
    * Purpose  :   Setter for the variable _row_spacing
    * Input    :   double r
    * Returns  :   void
    *************************************************************************/
    void setRowSpacing(double r) {
        _row_spacing = r;
    }

    /*************************************************************************
    * Function :   getColSpacing()
    * Purpose  :   Getter for the variable _col_spacing
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getColSpacing(void) {
        return _col_spacing;
    }

    /*************************************************************************
    * Function :   getColSpacingRef()
    * Purpose  :   Returns a reference to the variable Container::_col_spacing
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getColSpacingRef (void) {
        return _col_spacing;
    }

    /*************************************************************************
    * Function :   setColSpacing()
    * Purpose  :   Setter for the variable _col_spacing
    * Input    :   double c
    * Returns  :   void
    *************************************************************************/
    void setColSpacing(double c) {
        _col_spacing = c;
    }

    /*************************************************************************
    * Function :   getWellDepth()
    * Purpose  :   Getter for the variable _well_depth
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getWellDepth(void) {
        return _well_depth;
    }

    /*************************************************************************
    * Function :   getWellDepthRef()
    * Purpose  :   Returns a reference to the variable Container::_well_depth
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getWellDepthRef (void) {
        return _well_depth;
    }

    /*************************************************************************
    * Function :   setWellDepth()
    * Purpose  :   Setter for the variable _well_depth
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setWellDepth(double d) {
        _well_depth = d;
    }

    /*************************************************************************
    * Function :   getOffsetX()
    * Purpose  :   Getter for the variable _offset_x
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getOffsetX(void) {
        return _offset_x;
    }

    /*************************************************************************
    * Function :   getOffsetXRef()
    * Purpose  :   Returns a reference to the variable Container::_offset_x
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getOffsetXRef (void) {
        return _offset_x;
    }

    /*************************************************************************
    * Function :   setOffsetX()
    * Purpose  :   Setter for the variable _offset_x
    * Input    :   double o
    * Returns  :   void
    *************************************************************************/
    void setOffsetX(double o) {
        _offset_x = o;
    }

    /*************************************************************************
    * Function :   getOffsetY()
    * Purpose  :   Getter for the variable _offset_y
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getOffsetY(void) {
        return _offset_y;
    }

    /*************************************************************************
    * Function :   getOffsetYRef()
    * Purpose  :   Returns a reference to the variable Container::_offset_y
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getOffsetYRef (void) {
        return _offset_y;
    }

    /*************************************************************************
    * Function :   setOffsetY()
    * Purpose  :   Setter for the variable _offset_y
    * Input    :   double o
    * Returns  :   void
    *************************************************************************/
    void setOffsetY(double o) {
        _offset_y = o;
    }

    /*************************************************************************
    * Function :   getOffsetZ()
    * Purpose  :   Getter for the variable _offset_z
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getOffsetZ(void) {
        return _offset_z;
    }

    /*************************************************************************
    * Function :   getOffsetZRef()
    * Purpose  :   Returns a reference to the variable Container::_offset_z
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getOffsetZRef (void) {
        return _offset_z;
    }

    /*************************************************************************
    * Function :   setOffsetZ()
    * Purpose  :   Setter for the variable _offset_z
    * Input    :   double o
    * Returns  :   void
    *************************************************************************/
    void setOffsetZ(double o) {
        _offset_z = o;
    }

    /*************************************************************************
    * Function :   getCellsVecRef()
    * Purpose  :   Returns a reference to the variable Container::cells
    * Input    :   void
    * Returns  :   std::vector<std::vector<struct container_cell>>&
    *************************************************************************/
    std::vector<std::vector<struct container_cell>>& getCellsVecRef (void) {
        return _cells;
    }

  private:
    std::string _type = "tip";      // Type of container.(tip rack, well plate, etc.)
    double _length = 0;
    double _width = 0;
    double _height = 0;
    int _rows = 0;
    int _cols = 0;
    double _row_spacing = 0;
    double _col_spacing = 0;
    double _well_depth = 0;     // Well depth measured from top of deck (mm)
    double _offset_x = 0;       // Container origin offset from gantry origin (mm)
    double _offset_y = 0;
    double _offset_z = 0;
    std::vector<std::vector<struct container_cell>> _cells; // Vector of cell properties;
};
