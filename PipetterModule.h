/*************************************************************************
Title:    PipetterModule.h - Interface Class for CDXBot Pipetter Drivers
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     PipetterModule.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:


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
#pragma once

/**********************    INCLUDE DIRECTIVES    ***********************/
#include "CDXModule.h"
#include <errno.h>
#include <fcntl.h>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
class LiquidClass {
  public:
    LiquidClass(std::string name) {
        _name = name;
        _adc = 0;
        _aspirate_blowout_volume = 0.0f;
        _aspirate_settling_time = 0.0f;
        _aspirate_speed = 0.0f;
        _aspirate_swap_speed = 0.0f;
        _aspirate_type = 0;
        _clld_sensitivity = 0;
        _immersion_depth = 0.0f;
        _immersion_direction = 0;
        _lld_height_difference = 0.0f;
        _lld_mode = 0;
        _plld_sensitivity = 0;
        _prewet_volume = 0.0f;
        _transport_volume = 0.0f;
        _dispense_blowout_volume = 0.0f;
        _cutoff_speed = 0.0f;
        _dispense_height = 0.0f;
        _dispense_settling_time = 0.0f;
        _dispense_speed = 0.0f;
        _dispense_swap_speed = 0.0f;
        _dispense_type = 0;
        _leaving_height = 0.0f;
        _stop_back_volume = 0.0f;
        _transport_air_volume = 0.0f;
        _transport_speed = 0.0f;
    };
    ~LiquidClass();
    std::string &getNameRef(void) {
        return _name;
    }
    bool &getADCRef(void) {
        return _adc;
    }
    double &getAspirateBlowoutVolumeRef(void) {
        return _aspirate_blowout_volume;
    }
    double &getAspirateSettlingTimeRef(void) {
        return _aspirate_settling_time;
    }
    double &getAspirateSpeedRef(void) {
        return _aspirate_speed;
    }
    double &getAspirateSwapSpeedRef(void) {
        return _aspirate_swap_speed;
    }
    unsigned int &getAspirateTypeRef(void) {
        return _aspirate_type;
    }
    unsigned int &getCLLDSensitivityRef(void) {
        return _clld_sensitivity;
    }
    double &getImmersionDepthRef(void) {
        return _immersion_depth;
    }
    bool &getImmersionDirectionRef(void) {
        return _immersion_direction;
    }
    double & getLLDHeightDifferenceRef(void) {
        return _lld_height_difference;
    }
    unsigned int &getLLDModeRef(void) {
        return _lld_mode;
    }
    unsigned int &getPLLDSensitivityRef(void) {
        return _plld_sensitivity;
    }
    double &getPrewetVolumeRef(void) {
        return _prewet_volume;
    }
    double &transport_volumeRef(void) {
        return _transport_volume;
    }
    double &getDispenseBlowoutVolumeRef(void) {
        return _dispense_blowout_volume;
    }
    double &getCutoffSpeedRef(void) {
        return _cutoff_speed;
    }
    double &getDispenseHeightRef(void) {
        return _dispense_height;
    }
    double &getDispenseSettlingTimeRef(void) {
        return _dispense_settling_time;
    }
    double &getDispenseSpeedRef(void) {
        return _dispense_speed;
    }
    double &getDispenseSwapSpeedRef(void) {
        return _dispense_swap_speed;
    }
    unsigned int &getDispenseTypeRef(void) {
        return _dispense_type;
    }
    double getLeavingHeightRef(void) {
        return _leaving_height;
    }
    double getStopBackVolumeRef(void) {
        return _stop_back_volume;
    }
    double getTransportAirVolumeRef(void) {
        return _transport_air_volume;
    }
    double &getTransportSpeedRef(void) {
        return _transport_speed;
    }
  private:
    std::string _name;
    bool _adc;
    double _aspirate_blowout_volume;
    double _aspirate_settling_time;
    double _aspirate_speed;
    double _aspirate_swap_speed;
    unsigned int _aspirate_type;
    unsigned int _clld_sensitivity;
    double _immersion_depth;
    bool _immersion_direction;
    double _lld_height_difference;
    unsigned int _lld_mode;
    unsigned int _plld_sensitivity;
    double _prewet_volume;
    double _transport_volume;
    double _dispense_blowout_volume;
    double _cutoff_speed;
    double _dispense_height;
    double _dispense_settling_time;
    double _dispense_speed;
    double _dispense_swap_speed;
    unsigned int _dispense_type;
    double _leaving_height;
    double _stop_back_volume;
    double _transport_air_volume;
    double _transport_speed;
};

class PipetterModule : public CDXModule {
  public:
    PipetterModule (void) {};
    virtual ~PipetterModule (void) {};
    virtual bool aspirate(double vol, unsigned int gc_idx, unsigned int dg_idx,
                          unsigned int lc_idx, double liquid_surface) {};
    virtual bool dispense(double vol, unsigned int gc_idx, unsigned int dg_idx,
                          unsigned int lc_idx, double liquid_surface) {};
    virtual bool ejectTip(void) {};
    virtual bool home(void) {};
    virtual bool makeDeckGeometry(unsigned int index, double feed_plane,
                                  double container_offset_z,
                                  double tip_engagement_len,
                                  double tip_deposit_height) {};
    virtual bool makeContainerGeometry(unsigned int index, bool geometry,
                                       double diameter, double len_x, double len_y,
                                       double second_section_height,
                                       double second_section, double max_depth,
                                       double bottom_search_offset,
                                       double dispense_offset) {};
    LiquidClass *makeLiquidClass(std::string name, unsigned int index) {
        LiquidClass *p = new LiquidClass(name);
        _liquid_classes.insert(_liquid_classes.begin() + index, p);
        return p;
    }
    virtual bool setLiquidClass(unsigned int index, std::unique_ptr<LiquidClass>) {};
    virtual bool moveZ(double pos, double vel) {};
    virtual bool pickUpTip(unsigned int tt_idx, unsigned int dg_idx, bool speed) {};
    double getZPos(void) {
        return _zpos;
    }
    double &getZPosRef(void) {
        return _zpos;
    }
    bool getZAxisEnabled(void) {
        return _z_axis_enabled;
    }
    bool &getZAxisEnabledRef(void) {
        return _z_axis_enabled;
    }
    double getFeedPlane(void) {
        return _feed_plane;
    }
    double &getFeedPlaneRef(void) {
        return _feed_plane;
    }
    double getTipPickupSpeed(void) {
        return _tip_pickup_speed;
    }
    double &getTipPickupSpeedRef(void) {
        return _tip_pickup_speed;
    }

    std::string type;
    std::string driver_name;
    std::string driver_path;
    double _tip_pickup_speed;

  protected:
    double _zpos;
    double _zpos_min;
    double _zpos_max;
    bool _z_axis_enabled = 1;
    double _feed_plane = 1800;
    std::vector<LiquidClass*> _liquid_classes;
};

/***********************    FUNCTION PROTOTYPES    ***********************/
typedef PipetterModule *create_t();
typedef void destroy_t(PipetterModule *);
