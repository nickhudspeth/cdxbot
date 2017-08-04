#include "ShakerModule.h"
#include "cdxbot/gc_cmd.h"
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

ShakerModule *sm;
std::string type;
std::string driver_path, driver_name;
void *driver_handle;
create_t *create_sm;
destroy_t *destroy_sm;


ShakerModule * loadDriver(std::string file) {
    char *error;
    void *driver_handle = dlopen(file.c_str(), RTLD_LAZY);
    if(driver_handle == NULL) {
        ROS_ERROR_STREAM("Error loading shaker controller driver from " <<\
                         file);
        std::cerr << dlerror() << std::endl;
        //exit(-1);
        sm->deinit();
        ros::shutdown();
    }
    dlerror();
    ROS_INFO_STREAM("Successfully loaded shaker controller driver from " <<\
                    file);
    create_sm = (create_t*)dlsym(driver_handle, "create");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading maker function.\n " << error);
        dlerror();
    }
    ShakerModule *s = create_sm();

    destroy_sm = (destroy_t*)dlsym(driver_handle, "destroy");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading destroy function.\n " << error);
        dlerror();
    }

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

    // if(!nh.getParam("/sc_conf/z_axis_enabled", sm->getZAxisEnabledRef())) {

    // nh.getParam("/sc_defaults/z_axis_enabled", sm->getZAxisEnabledRef());
    // ROS_WARN_STREAM("No parameter \"_z_axis_enabled\" found in configuration file.\
    // Initializing pipetter with default value " <<\
    // sm->getZAxisEnabledRef());
    // }
}


void shutdownCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_WARN_STREAM("ShakerControllerNode: Received shutdown directive.");
    sm->deinit();
    ros::shutdown();
}

void scPubCallback(const std_msgs::String::ConstPtr &msg) {

}

bool startCallback(cdxbot::shakerStart::Request &req,
                   cdxbot::shakerStart::Response &resp) {
    if(sm->start() == 1) {
        return 1;
    } else {
        return 0;
    }
}

bool stopCallback(cdxbot::shakerStop::Request &req,
                  cdxbot::shakerStop::Response &resp) {
    if(sm->stop() == 1) {
        return 1;
    } else {
        return 0;
    }
}

bool setFreqCallback(cdxbot::shakerSetFreq::Request &req,
                     cdxbot::shakerSetFreq::Response &resp) {
    if(sm->setFrequency(req.freq) == 1) {
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
}

bool setPowerCallback(cdxbot::shakerSetPower::Request &req,
                      cdxbot::shakerSetPower::Response &resp) {
    if(sm->setFrequency(req.pwr) == 1) {
        resp.ok = true;
        return 1;
    } else {
        resp.ok = false;
        return 0;
    }
}

bool resetCallback(cdxbot::shakerReset::Request &req,
                   cdxbot::shakerReset::Response &resp) {
    if(sm->reset() == 1) {
        return 1;
    } else {
        return 0;
    }
}

int main(int argc, char **argv) {
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "shakerControllerNode");
    ros::NodeHandle nh;
    loadParams(nh);
    sm->init();
    /* Instantiate publishers and subscribers*/
    // ros::Subscriber sub_sc = nh.subscribe("/pc_pub", 100, &scPubCallback);
    ros::Subscriber shutdown = nh.subscribe("/sd_pub", 100, &shutdownCallback);
    ROS_DEBUG_STREAM("Initialized pc with addr: " << &sm);
    /* Instantiate service servers */
    ros::ServiceServer startServer = nh.advertiseService("shaker_start", &startCallback);
    ros::ServiceServer stopServer = nh.advertiseService("shaker_stop", &stopCallback);
    ros::ServiceServer setFreqServer = nh.advertiseService("shaker_set_freq", &setFreqCallback);
    ros::ServiceServer setPowerServer = nh.advertiseService("shaker_set_power", &setPowerCallback);
    ros::ServiceServer resetServer = nh.advertiseService("shaker_reset", &resetCallback);

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
