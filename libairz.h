/*************************************************************************
Title:    libairz.h - CDXBot Driver for TriContinent Air-Z Premier Pipetter
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     libairz.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

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

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
#define N0 0 // Normal increment mode
#define N1 1 // Micro-increment mode
#define N2 2 // Microliter mode
#define NORMAL_INCREMENT_MAX 3143 // Maximum displacement in normal increment mode
#define MICRO_INCREMENT_MAX 50288 // Maximum displacement in micro-increment mode
#define MICROLITER_INCREMENT_MAX 1100.050 // Maximum displacement in microliter increment mode
#define TIMEOUT_DEFAULT 10
#define IDLE 0
#define BUSY 1
class AirZModule : public PipetterModule {
  public:
    airZModule (void);
    virtual ~airZModule ();
    int init(void);
    int deinit(void);
    int lconf(void);
    // void seterrfunc(void(*ef)(std::string s)) {
    // PRINT_ERROR = ef;
    // }
    void moveZ(double pos, double vel) {};
    void pickUpTip(int index) {};
    void pickUpTip(struct container_cell c) {};
    void aspirate(double vol) {};
    void dispense(double vol) {};
    void ejectTip(void) {};
    void home(void) {};
    // void setTipParams(struct tip_params t){
    // _tp.min_traverse_height = t.min_traverse_height;
    // _

    // }

    double getZPos(void) {
        return _zpos;
    }
    double &getZPosRef(void) {
        return _zpos;
    }
    double getFeedPlaneHeight(void) {
        return _feed_plane_height;
    }
    double &getFeedPlaneHeightRef(void) {
        return _feed_plane_height;
    }
    int setID(unsigned int id) {
        if((id < 1) || (id > 16)) {
            /* ERROR: ID out of valid range [1, 16] */
            return -1;
        }
        _id = id;
    }
    unsigned int getID(void) {
        return _id;
    }
    unsigned int &getIDRef(void) {
        return _id;
    }


  private:
    unsigned int _id;
    float _volume;
    unsigned int _capacitance_change_threshold;
    /*************************************************************************
    * Function :   makeCommandString()
    * Purpose  :   Returns a formatted command string ready for transmission.
    * Input    :   std::string cmd - command string to be formatted
    *              bool run - should a run character be appended to end of
    *              string?
    * Returns  :       std::string
    *************************************************************************/
    std::string makeCommandString(std::string cmd, bool run);
    /*************************************************************************
    * Function :   waitForReadyStatus()
    * Purpose  :   Polls the pipetter after a command is issued to check if the
    *              pipetter is ready to accept a new command.
    * Input    :   unsigned int timeout
    * Returns  :   bool
    *************************************************************************/
    bool waitForReadyStatus(unsigned int timeout = TIMEOUT_DEFAULT);
    /*************************************************************************
    * Function :   movePumpToAbsolutePosition()
    * Purpose  :   What does this function do?
    * Input    :   double pos
    * Returns  :   bool
    *************************************************************************/
    bool movePumpToAbsolutePosition(double pos);
    /*************************************************************************
    * Function :   relativePickup()
    * Purpose  :   Moves the pump up (aspirates) the distance specified by [d]
    * Input    :   double d
    * Returns  :   bool
    *************************************************************************/
    bool relativePickup(double d);
    /*************************************************************************
    * Function :   relativeDispense()
    * Purpose  :   Moves the pump down (dispenses) the distance specified by [d]
    * Input    :   double d
    * Returns  :   bool
    *************************************************************************/
    bool relativeDispense(double d);
    /*************************************************************************
    * Function :   setAccelerationSlopes()
    * Purpose  :   Sets the velocity ramp-up and ramp-down slopes.
    * Input    :   double accelup, double acceldown
    * Returns  :   bool
    *************************************************************************/
    bool setAccelerationSlopes(double accelup, double acceldown);
    /*************************************************************************
    * Function :   setVelocityStart()
    * Purpose  :   Sets teh velocity at which the pump begins its movement
    * Input    :   double vel
    * Returns  :   bool
    *************************************************************************/
    bool setVelocityStart(double vel);
    /*************************************************************************
    * Function :   setVelocityTop()
    * Purpose  :   Sets the top speed of the punp
    * Input    :   double vel
    * Returns  :   bool
    *************************************************************************/
    bool setVelocityTop(double vel);
    /*************************************************************************
    * Function :   setVelocityCutoff()
    * Purpose  :   Sets the velocity at which the pump ends its movement
    * Input    :   double vel
    * Returns  :   bool
    *************************************************************************/
    bool setVelocityCutoff(double vel);
    /*************************************************************************
    * Function :   setIncrementMode()
    * Purpose  :   Sets the increment mode that determines how all affected
    * motion command inputs will be interpreted.
    * Input    :   unsigned int mode
    * Returns  :   bool
    *************************************************************************/
    bool setIncrementMode(unsigned int mode);
    /*************************************************************************
    * Function :   setAuxiliaryOutput()
    * Purpose  :   Sets I/O 1 (connector P4 pin 5 ) to [state]
    * Input    :   bool state
    * Returns  :   bool
    *************************************************************************/
    bool setAuxiliaryOutput(bool state);
    /*************************************************************************
    * Function :   terminate()
    * Purpose  :   Immediately terminates any executing command string
    * Input    :   void
    * Returns  :   bool
    *************************************************************************/
    bool terminate(void);
    /*************************************************************************
    * Function :   ejectTip()
    * Purpose  :   Ejects any tip currently installed, returning to home
    *              position afterward.
    * Input    :   bool use_tip_sensor
    * Returns  :   bool
    *************************************************************************/
    bool ejectTip(bool use_tip_sensor);
    /*************************************************************************
    * Function :   checkForUnexpectedTipLoss()
    * Purpose  :   Configures the pump to continually check the tip sensor
    * presence. If a tip is not in place, an error code "k" will be set. Issuing
    * the ejectTip command will cancel lost tip reporting, if active.
    * Input    :   void
    * Returns  :   bool
    *************************************************************************/
    bool checkForUnexpectedTipLoss(void);
    /*************************************************************************
    * Function :   activatePLLD()
    * Purpose  :   Runs the pump in aspirate (dir = 0) or dispense (dir = 1)
    * mode until the column pressure exceeds (threshold)
    * Input    :   double threshold
    *              bool dir
    * Returns  :   bool
    *************************************************************************/
    bool activatePLLD(double threshold, bool dir);
    /*************************************************************************
    * Function :   activateCLLD()
    * Purpose  :   Initially measures the column capacitance and continually
    *              measures column capacitance until
    *              (measurement -cstart) > threshold.
    * Input    :   unsigned int threshold
    * Returns  :   bool
    *************************************************************************/
    bool activateCLLD(unsigned int threshold);
    /*************************************************************************
    * Function :   activateHLLD()
    * Purpose  :   Activates both the PLLD (in dispense mode) and CLLD
    *              functions described above.
    * Input    :   unsigned int c_threshold, unsigned int p_threshold
    * Returns  :   bool
    *************************************************************************/
    bool activateHLLD(unsigned int c_threshold, unsigned int p_threshold);
    /*************************************************************************
    * Function :   togglePressureDataStreaming()
    * Purpose  :   Enables or disables streaming pressure data from pipetter.
    * Input    :   bool state
    * Returns  :   bool
    *************************************************************************/
    bool togglePressureDataStreaming(bool state);
    /*************************************************************************
    * Function :   setPressureTransducerGain()
    * Purpose  :   Sets the pressure transducer gain to n:{0, 1, 2, 3,
    *              4=default}
    * Input    :   unsigned int gain
    * Returns  :   bool
    *************************************************************************/
    bool setPressureTransducerGain(unsigned int gain);
    




    bool sendCommand(std::string s);
    bool liquidLevelDetect(int timeout);
    bool getStatus(void);


    /* data */
};

/***********************    FUNCTION PROTOTYPES    ***********************/
