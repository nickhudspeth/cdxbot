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
#include <stdbool.h>
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
struct container_cell {
    std::string test_type;
    bool used;
    double vol;            // Well volume (mL)
    double filled_vol;
    double depth;
    /* Pipette tip properties */
    int tt_index; // tip type table index.
    double min_traverse_height;
    double min_z_pos;
    double botpp;
    double eotpp;
    double potdp;
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
    * Function :   getLength(const char axis)
    * Purpose  :   Getter for the variable _length_{axis}
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getLength(const char axis) {
        switch(axis) {
        case 'x':
            return _len_x;
            break;
        case 'y':
            return _len_y;
            break;
        case 'z':
            return _len_z;
            break;
        }
        return -1;
    }

    /*************************************************************************
    * Function :   getLengthRef()
    * Purpose  :   Returns a reference to the variable Container::_length_{axis}
    * Input    :   const char axis
    * Returns  :   double&
    *************************************************************************/
    double& getLengthRef (const char axis) {
        switch(axis) {
        case 'x':
            return _len_x;
            break;
        case 'y':
            return _len_y;
            break;
        case 'z':
            return _len_z;
            break;
        }
    }

    /*************************************************************************
    * Function :   setLength()
    * Purpose  :   Setter for the variable _len_{axis}
    * Input    :   const char axis, double l
    * Returns  :   void
    *************************************************************************/
    void setLength(const char axis, double l) {
        switch(axis) {
        case 'x':
            _len_x = l;
            break;
        case 'y':
            _len_y = l;
            break;
        case 'z':
            _len_z = l;
            break;
        }
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
    * Function :   getTipEngagementLen()
    * Purpose  :   Getter for the variable _tip_engagement_len
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getTipEngagementLen(void) {
        return _tip_engagement_len;
    }

    /*************************************************************************
    * Function :   getTipEngagementLenRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_tip_engagement_len
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getTipEngagementLenRef (void) {
        return _tip_engagement_len;
    }

    /*************************************************************************
    * Function :   setTipEngagementLen()
    * Purpose  :   Setter for the variable _tip_engagement_len
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setTipEngagementLen(double d) {
        _tip_engagement_len = d;
    }


    /*************************************************************************
    * Function :   getTipDepositHeight()
    * Purpose  :   Getter for the variable _tip_deposit_height
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getTipDepositHeight(void) {
        return _tip_deposit_height;
    }

    /*************************************************************************
    * Function :   getTipDepositHeightRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_tip_deposit_height
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getTipDepositHeightRef (void) {
        return _tip_deposit_height;
    }

    /*************************************************************************
    * Function :   setTipDepositHeight()
    * Purpose  :   Setter for the variable _tip_deposit_height
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setTipDepositHeight(double d) {
        _tip_deposit_height = d;
    }


    /*************************************************************************
    * Function :   getWellGeometry()
    * Purpose  :   Getter for the variable _well_geometry
    * Input    :   void
    * Returns  :   std::string
    *************************************************************************/
    std::string getWellGeometry(void) {
        return _well_geometry;
    }

    /*************************************************************************
    * Function :   getWellGeometryRef()
    * Purpose  :   Returns a reference to the variable Container::_well_geometry
    * Input    :   void
    * Returns  :   std::string&
    *************************************************************************/
    std::string& getWellGeometryRef (void) {
        return _well_geometry;
    }

    /*************************************************************************
    * Function :   setWellGeometry()
    * Purpose  :   Setter for the variable _well_geometry
    * Input    :   std::string g
    * Returns  :   void
    *************************************************************************/
    void setWellGeometry(std::string g) {
        _well_geometry = g;
    }


    /*************************************************************************
    * Function :   getWellDiameter()
    * Purpose  :   Getter for the variable _well_diameter
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getWellDiameter(void) {
        return _well_diameter;
    }

    /*************************************************************************
    * Function :   getWellDiameterRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_well_diameter
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getWellDiameterRef (void) {
        return _well_diameter;
    }

    /*************************************************************************
    * Function :   setWellDiameter()
    * Purpose  :   Setter for the variable _well_diameter
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setWellDiameter(double d) {
        _well_diameter = d;
    }


    /*************************************************************************
    * Function :   getWellLen()
    * Purpose  :   Getter for the variable _well_len_{axis}
    * Input    :   const char axis
    * Returns  :   double
    *************************************************************************/
    double getWellLen(const char axis) {
        switch(axis) {
        case 'x':
            return _well_len_x;
        case 'y':
            return _well_len_y;
        default:
            /* Throw error. Invalid axis specified */
            return 0.0f;
        }
    }

    /*************************************************************************
    * Function :   getWellLenRef()
    * Purpose  :   Returns a reference to the variable Container::_well_len_{axis}
    * Input    :   const char axis
    * Returns  :   double&
    *************************************************************************/
    double& getWellLenRef (const char axis) {
        switch(axis) {
        case 'x':
            return _well_len_x;
        case 'y':
            return _well_len_y;
        default:
            /* Throw error. Invalid axis specified */
            break;
        }
    }

    /*************************************************************************
    * Function :   setWellLen()
    * Purpose  :   Setter for the variable _well_len_{axis}
    * Input    :   const char axis, double d
    * Returns  :   void
    *************************************************************************/
    void setWellLen(const char axis, double d) {
        switch(axis) {
        case 'x':
            _well_len_x = d;
        case 'y':
            _well_len_y = d;
        }
    }


    /*************************************************************************
    * Function :   getSecondSectionHeight()
    * Purpose  :   Getter for the variable _second_section_height
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getSecondSectionHeight(void) {
        return _second_section_height;
    }

    /*************************************************************************
    * Function :   getSecondSectionHeightRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_second_section_height
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getSecondSectionHeightRef (void) {
        return _second_section_height;
    }

    /*************************************************************************
    * Function :   setSecondSectionHeight()
    * Purpose  :   Setter for the variable _second_section_height
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setSecondSectionHeight(double d) {
        _second_section_height = d;
    }


    /*************************************************************************
    * Function :   getSecondSection()
    * Purpose  :   Getter for the variable _second_section
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getSecondSection(void) {
        return _second_section;
    }

    /*************************************************************************
    * Function :   getSecondSectionRef()
    * Purpose  :   Returns a reference to the variable Container::_second_section
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getSecondSectionRef (void) {
        return _second_section;
    }

    /*************************************************************************
    * Function :   setSecondSection()
    * Purpose  :   Setter for the variable _second_section
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setSecondSection(double d) {
        _second_section = d;
    }


    /*************************************************************************
    * Function :   getMinHeight()
    * Purpose  :   Getter for the variable _min_height
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getMinHeight(void) {
        return _min_height;
    }

    /*************************************************************************
    * Function :   getMinHeightRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_min_height
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getMinHeightRef (void) {
        return _min_height;
    }

    /*************************************************************************
    * Function :   setMinHeight()
    * Purpose  :   Setter for the variable _min_height
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setMinHeight(double d) {
        _min_height = d;
    }


    /*************************************************************************
    * Function :   getStartPositionOffset()
    * Purpose  :   Getter for the variable _start_pos_offset
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getStartPositionOffset(void) {
        return _start_pos_offset;
    }

    /*************************************************************************
    * Function :   getStartPositionOffsetRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_start_pos_offset
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getStartPositionOffsetRef (void) {
        return _start_pos_offset;
    }

    /*************************************************************************
    * Function :   setStartPositionOffset()
    * Purpose  :   Setter for the variable _start_pos_offset
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setStartPositionOffset(double d) {
        _start_pos_offset = d;
    }


    /*************************************************************************
    * Function :   getDispensePosition()
    * Purpose  :   Getter for the variable _dispense_pos
    * Input    :   void
    * Returns  :   double
    *************************************************************************/
    double getDispensePosition(void) {
        return _dispense_pos;
    }

    /*************************************************************************
    * Function :   getDispensePositionRef()
    * Purpose  :   Returns a reference to the variable
    * Container::_dispense_pos
    * Input    :   void
    * Returns  :   double&
    *************************************************************************/
    double& getDispensePositionRef (void) {
        return _dispense_pos;
    }

    /*************************************************************************
    * Function :   setDispensePosition()
    * Purpose  :   Setter for the variable _dispense_pos
    * Input    :   double d
    * Returns  :   void
    *************************************************************************/
    void setDispensePosition(double d) {
        _dispense_pos = d;
    }

    /*************************************************************************
    * Function :   getOffset()
    * Purpose  :   Getter for the variable _offset_{axis}
    * Input    :   const char axis
    * Returns  :   double
    *************************************************************************/
    double getOffset(const char axis) {
        switch(axis) {
        case 'x':
            return _offset_x;
            break;
        case 'y':
            return _offset_y;
            break;
        case 'z':
            return _offset_z;
            break;
        }
        return -1;
    }

    /*************************************************************************
    * Function :   getOffsetRef()
    * Purpose  :   Returns a reference to the variable Container::_offset_{axis}
    * Input    :   const char axis
    * Returns  :   double&
    *************************************************************************/
    double& getOffsetRef (const char axis) {
        switch(axis) {
        case 'x':
            return _offset_x;
            break;
        case 'y':
            return _offset_y;
            break;
        case 'z':
            return _offset_z;
            break;
        }
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

    /*************************************************************************
    * Function :   getTipTypeTableIndex()
    * Purpose  :   Returns the tip type table index for this container
    * Input    :   void
    * Returns  :   unsigned int
    *************************************************************************/
    unsigned int getTipTypeTableIndex(void) {
        return _tip_type_table_index;
    }

    /*************************************************************************
    * Function :   getDeckGeometryTableIndex()
    * Purpose  :   Returns the deck geometry table index for this container
    * Input    :   void
    * Returns  :   unsigned int
    *************************************************************************/
    unsigned int getDeckGeometryTableIndex(void) {
        return _deck_geometry_table_index;
    }


  private:
    std::string _type = "tip";      // Type of container.(tip rack, well plate, etc.)
    double _len_x = 0.0f;
    double _len_y = 0.0f;
    double _len_z = 0.0f;
    int _rows = 0;
    int _cols = 0;
    double _offset_x = 0.0f;       // Container origin offset from gantry origin (mm)
    double _offset_y = 0.0f;
    double _offset_z = 0.0f;
    double _row_spacing = 0.0f;
    double _col_spacing = 0.0f;
    std::vector<std::vector<struct container_cell>> _cells; // Vector of cell properties;
    unsigned int _deck_geometry_table_index = 0;
    /* TIP TRAY-SPECIFIC PROPERTIES */
    double _tip_engagement_len;
    double _tip_deposit_height;
    unsigned int _tip_type_table_index = 0;
    /* WELL-SPECIFIC PROPERTIES */
    std::string _well_geometry = "";
    double _well_diameter = 0.0f;
    double _well_len_x = 0.0f;
    double _well_len_y = 0.0f;
    double _second_section_height = 0.0f;
    double _second_section = 0.0f;
    double _min_height = 0.0f;
    double _start_pos_offset = 0.0f;
    double _dispense_pos = 0.0f;
    double _well_depth = 0.0f;     // Well depth measured from top of deck (mm)
    unsigned int _container_geometry_table_index = 0;
};
