#include "PipetterModule.h"
#include "cdxbot/gc_cmd.h"
#include "cdxbot/pc_cmd.h"
#include "cdxbot/pipetterAspirate.h"
#include "cdxbot/pipetterDispense.h"
#include "cdxbot/pipetterEjectTip.h"
#include "cdxbot/pipetterMoveZ.h"
#include "cdxbot/pipetterPickUpTip.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <stdlib.h>
// #include "PipetterController.h"

PipetterModule *pc;
std::string type;
std::string driver_path, driver_name;
void *driver_handle;
create_t *create_pm;
destroy_t *destroy_pm;


PipetterModule * loadDriver(std::string file) {
    char *error;
    void *driver_handle = dlopen(file.c_str(), RTLD_LAZY);
    if(driver_handle == NULL) {
        ROS_ERROR_STREAM("Error loading pipetter controller driver from " <<\
                         file);
        std::cerr << dlerror() << std::endl;
        //exit(-1);
        pc->deinit();
        ros::shutdown();
    }
    dlerror();
    ROS_INFO_STREAM("Successfully loaded pipetter controller driver from " <<\
                    file);
    create_pm = (create_t*)dlsym(driver_handle, "create");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading maker function.\n " << error);
        dlerror();
    }
    PipetterModule *pm = create_pm();

    destroy_pm = (destroy_t*)dlsym(driver_handle, "destroy");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading destroy function.\n " << error);
        dlerror();
    }

    return pm;
}


void loadParams(ros::NodeHandle &nh) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/pc_conf/type", type)) {
        nh.getParam("/pc_defaults/type", type);
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing pipetter with default value %s",\
                 type);
    }
    if(!nh.getParam("/pc_conf/driver_name", driver_name)) {
        nh.getParam("/pc_defaults/driver_name", driver_name);
        ROS_WARN_STREAM("No parameter \"driver_name\" found in configuration file.\
                            Initializing pipetter with default value " <<\
                        driver_name);
    }
    if(!nh.getParam("/pc_conf/driver_path", driver_path)) {
        nh.getParam("/pc_defaults/driver_path", driver_path);
        ROS_WARN_STREAM("No parameter \"driver_path\" found in configuration file.\
                                Initializing pipetter with default value " <<\
                        driver_path);
    }
    pc = loadDriver(driver_path + driver_name);

    if(!nh.getParam("/pc_conf/z_axis_enabled", pc->getZAxisEnabledRef())) {

        nh.getParam("/pc_defaults/z_axis_enabled", pc->getZAxisEnabledRef());
        ROS_WARN_STREAM("No parameter \"_z_axis_enabled\" found in configuration file.\
                                Initializing pipetter with default value " <<\
                        pc->getZAxisEnabledRef());
    }
    if(!nh.getParam("/pc_conf/feed_plane_height", pc->getFeedPlaneHeightRef())) {
        nh.getParam("/pc_defaults/feed_plane_height", pc->getFeedPlaneHeightRef());
        ROS_WARN_STREAM("No paramater \"_feed_plane_height\" found in configuration file.\
               Initializing pipetter with default value " << pc->getFeedPlaneHeightRef());
    }
}

void gcPubCallback(const cdxbot::gc_cmd &msg) {
    ROS_DEBUG_STREAM("PipetterControllerNode:: Received gc_cmd - " << msg.cmd);
    if(msg.cmd == "movez") {
        // pc->moveZ((1800 - 219.075 - msg.z), 1);
        pc->moveZ(msg.z, 1);
    }
    /* Move Pipetter head to feed plane height at startup */
    else if(msg.cmd == "home") {
        // pc->moveZ((1800 - 219.75 - pc->getFeedPlaneHeight()), 1);
        pc->moveZ(pc->getFeedPlaneHeight(), 1);
    }
}

void pcPubCallback(const cdxbot::pc_cmd &msg) {
    if(msg.cmd == "aspirate") {
        pc->aspirate(msg.vol);
    } else if(msg.cmd == "dispense") {
        pc->dispense(msg.vol);
    } else if(msg.cmd == "eject") {
        pc->ejectTip();
    } else if(msg.cmd == "pickup") {
        pc->pickUpTip(msg.type);
    }
}

void shutdownCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO_STREAM("PipetterControllerNode: Received shutdown directive.");
    pc->deinit();
    ros::shutdown();
}

bool moveZCallback(cdxbot::pipetterMoveZ::Request &req,
                   cdxbot::pipetterMoveZ::Response &resp) {
    pc->moveZ(req.pos, 1);

}

bool pickUpTipCallback(cdxbot::pipetterPickUpTip::Request &req,
                       cdxbot::pipetterPickUpTip::Response &resp) {

}

bool ejectTipCallback(cdxbot::pipetterEjectTip::Request & req,
                      cdxbot::pipetterEjectTip::Response &resp) {

}

bool aspirateCallback(cdxbot::pipetterAspirate::Request & req,
                      cdxbot::pipetterAspirate::Response &resp) {

}

bool dispenseCallback(cdxbot::pipetterDispense::Request & req,
                      cdxbot::pipetterDispense::Response &resp) {

}




int main(int argc, char **argv) {
    // const char *pcfile = "..pipetterConfig.conf";
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "pipetterControllerNode");
    ros::NodeHandle nh;
    loadParams(nh);
    pc->init();
    /* Instantiate publishers and subscribers*/
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>(\
                         "cdxbot/pipetter_zpos", 100);
    ros::Subscriber sub_pc = nh.subscribe("/pc_pub", 100, &pcPubCallback);
    ros::Subscriber shutdown = nh.subscribe("/sd_pub", 1000, &shutdownCallback);
    ros::Subscriber sub_gc = nh.subscribe("/gc_pub", 100, &gcPubCallback);
    ROS_INFO_STREAM("Initialized pc with addr: " << &pc);
    /* Instantiate service servers */
    ros::ServiceServer moveZServer = nh.advertiseService("pipetter_move_z",
                                     &moveZCallback);
    ros::ServiceServer pickUpTipServer = nh.advertiseService("pipetter_pick_up_tip",
                                         &pickUpTipCallback);
    ros::ServiceServer ejectTipServer = nh.advertiseService("pipetter_eject_tip",
                                        &ejectTipCallback);
    ros::ServiceServer aspirateServer = nh.advertiseService("pipetter_aspirate",
                                        &aspirateCallback);
    ros::ServiceServer dispenseServer = nh.advertiseService("pipetter_dispense",
                                        &dispenseCallback);


    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        msg.vector.x = 0;
        msg.vector.y = 0;
        msg.vector.z = pc->getZPos();
        rate.sleep();
    }
    return 0;
}
