#include "ShakerModule.h"
#include "cdxbot/gc_cmd.h"
#include "cdxbot/nodeInit.h"
#include "cdxbot/nodeShutdown.h"
#include "cdxbot/pc_cmd.h"
#include "cdxbot/shakerReset.h"
#include "cdxbot/shakerSetFreq.h"
#include "cdxbot/shakerSetPower.h"
#include "cdxbot/shakerStart.h"
#include "cdxbot/shakerStop.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <stdlib.h>
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
ShakerModule *sm;
std::string type;
std::string driver_path, driver_name;
void *driver_handle;
create_t *create_sm;
destroy_t *destroy_sm;
bool timeout_flag = false;
float timeout_dur = 0.0;

bool node_should_idle = false;
/*******************    FUNCTION IMPLEMENTATIONS    ********************/
void handleDebugMessages(const std::string &msg) {
    ROS_DEBUG("ShakerControllerNode: %s", msg.c_str());
}
void handleInfoMessages(const std::string &msg) {
    ROS_INFO("ShakerControllerNode: %s", msg.c_str());
}
void handleWarningMessages(const std::string &msg) {
    ROS_WARN("ShakerControllerNode: %s", msg.c_str());
}
void handleErrorMessages(const std::string &msg) {
    ROS_ERROR("ShakerControllerNode: %s", msg.c_str());
}

ShakerModule * loadDriver(std::string file) {
    char *error;
    void *driver_handle = dlopen(file.c_str(), RTLD_LAZY);
    if(driver_handle == NULL) {
        ROS_ERROR_STREAM("ShakerControllerNode: Error loading shaker controller driver from " <<\
                         file << dlerror());
        sm->deinit();
        // ros::shutdown();
    }
    dlerror();
    ROS_INFO_STREAM("ShakerControllerNode: Successfully loaded shaker controller driver from " <<\
                    file);
    create_sm = (create_t*)dlsym(driver_handle, "create");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("ShakerControllerNode: Error loading maker function.\n " << error);
        dlerror();
    }
    ShakerModule *s = create_sm();

    destroy_sm = (destroy_t*)dlsym(driver_handle, "destroy");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("ShakerControllerNode: Error loading destroy function.\n " << error);
        dlerror();
    }
    s->setDebugMsgCallback(handleDebugMessages);
    s->setInfoMsgCallback(handleInfoMessages);
    s->setWarningMsgCallback(handleWarningMessages);
    s->setErrorMsgCallback(handleErrorMessages);

    return s;
}


void loadParams(ros::NodeHandle &nh) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/sc_conf/type", type)) {
        nh.getParam("/sc_defaults/type", type);
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing pipetter with default value %s",\
                 type);
    }
    if(!nh.getParam("/sc_conf/driver_name", driver_name)) {
        nh.getParam("/sc_defaults/driver_name", driver_name);
        ROS_WARN_STREAM("No parameter \"driver_name\" found in configuration file.\
                            Initializing pipetter with default value " <<\
                        driver_name);
    }
    if(!nh.getParam("/sc_conf/driver_path", driver_path)) {
        nh.getParam("/sc_defaults/driver_path", driver_path);
        ROS_WARN_STREAM("No parameter \"driver_path\" found in configuration file.\
                                Initializing pipetter with default value " <<\
                        driver_path);
    }
    sm = loadDriver(driver_path + driver_name);
}

bool initServiceCallback(cdxbot::nodeInit::Request & req,
                               cdxbot::nodeInit::Response &resp) {
    int init_ret;
    bool success = false;
    if((init_ret = sm->init()) < 0) {
        ROS_ERROR_STREAM("shakerControllerNode: Could not initialize pipetter. sm->init() return code = " << init_ret);
    } else {
        node_should_idle = false;
        success = true;
    }
    return success;
}


void shutdownCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_WARN_STREAM("ShakerControllerNode: Received shutdown directive.");
    sm->deinit();
    ros::shutdown();
}

bool shutdownServiceCallback(cdxbot::nodeShutdown::Request & req,
                             cdxbot::nodeShutdown::Response &resp) {
    ROS_WARN_STREAM("shakerControllerNode: Received shutdown directive.");
    sm->deinit();
    ros::shutdown();
    return true;
}

void stopFromTimerCallback(const ros::TimerEvent &e) {
    ROS_DEBUG_STREAM("ShakerControllerNode: Entered stopFromTimerCallback().");
    ROS_INFO_STREAM("Stopping shaker.");
    sm->stop();
    ROS_DEBUG_STREAM("ShakerControllerNode: Exiting stopFromTimerCallback().");
}

bool stopServiceCallback(cdxbot::shakerStop::Request &req,
                  cdxbot::shakerStop::Response &resp) {
    ROS_DEBUG_STREAM("ShakerControllerNode: Entered stopServiceCallback().");
    if(sm->stop() == 1) {
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
    ROS_DEBUG_STREAM("ShakerControllerNode: Exiting stopServiceCallback().");
}

bool startServiceCallback(cdxbot::shakerStart::Request &req,
                   cdxbot::shakerStart::Response &resp) {
    ROS_DEBUG_STREAM("ShakerControllerNode: Entered startServiceCallback().");
    /*Start the shaker and run for the amount of time specified in req->time.*/
    if(sm->start() == 1) {
        // if(req.time > 0.0){
            // timeout_dur = req.time;
            // timeout_flag = true;
        // }
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
    ROS_DEBUG_STREAM("ShakerControllerNode: Exiting startServiceCallback().");
}

bool setFreqServiceCallback(cdxbot::shakerSetFreq::Request &req,
                     cdxbot::shakerSetFreq::Response &resp) {
    ROS_DEBUG_STREAM("ShakerControllerNode: Entered setFreqServiceCallback().");
    if(sm->setFrequency(req.freq) == 1) {
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
    ROS_DEBUG_STREAM("ShakerControllerNode: Exiting setFreqServiceCallback().");
}

bool setPowerServiceCallback(cdxbot::shakerSetPower::Request &req,
                      cdxbot::shakerSetPower::Response &resp) {
    ROS_DEBUG_STREAM("ShakerControllerNode: Entered setPowerServiceCallback().");
    if(sm->setFrequency(req.pwr) == 1) {
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
    ROS_DEBUG_STREAM("ShakerControllerNode: Exiting setPowerServiceCallback().");
}

bool resetServiceCallback(cdxbot::shakerReset::Request &req,
                   cdxbot::shakerReset::Response &resp) {
    ROS_DEBUG_STREAM("ShakerControllerNode: Entered resetServiceCallback().");
    if(sm->reset() == 1) {
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
    ROS_DEBUG_STREAM("ShakerControllerNode: Exiting resetServiceCallback().");
}

int main(int argc, char **argv) {
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "shakerControllerNode");
    ros::NodeHandle nh;
    loadParams(nh);
    /* Instantiate publishers and subscribers*/
    ros::Subscriber shutdown = nh.subscribe("/sd_pub", 100, &shutdownCallback);
    ROS_DEBUG_STREAM("Initialized pc with addr: " << &sm);
    /* Instantiate service servers */
    ros::ServiceServer nodeInitServer = nh.advertiseService("shaker_init", &initServiceCallback);
    ros::ServiceServer nodeShutdownServer = nh.advertiseService("shaker_shutdown", &shutdownServiceCallback);
    ros::ServiceServer startServer = nh.advertiseService("shaker_start", &startServiceCallback);
    ros::ServiceServer stopServer = nh.advertiseService("shaker_stop", &stopServiceCallback);
    ros::ServiceServer setFreqServer = nh.advertiseService("shaker_set_freq", &setFreqServiceCallback);
    ros::ServiceServer setPowerServer = nh.advertiseService("shaker_set_power", &setPowerServiceCallback);
    ros::ServiceServer resetServer = nh.advertiseService("shaker_reset", &resetServiceCallback);

    ros::Rate rate(100);
    ROS_DEBUG_STREAM("shakerControllerNode: Node has started.");

    while(ros::ok()) {
        while(node_should_idle == true){
            ros::spinOnce();
            rate.sleep();
        }
        // if(timeout_flag){
            // ROS_INFO_STREAM("Running shaker with " << timeout_dur << " second timeout.");
            // ros::Timer timer = nh.createTimer(ros::Duration(timeout_dur), stopFromTimerCallback, true);
            // timeout_flag = false;
            // timeout_dur = 0.0;
        // }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
