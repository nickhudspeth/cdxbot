#/usr/bin/env python

import rospy
#import zeus
from geometry_msgs.msg import Vector3Stamped

def zpos_callback(data):
    if(pipetter.has_z_drive):
        pipetter.MoveZDrive(data.vector.z, "fast")

def main():
    zpos_pub = rospy.Publisher('pipetter_zpos', Vector3Stamped, queue_size=10)
    zpos_sub = rospy.Subscriber("pipetter_cmd_zpos", Vector3Stamped, zpos_callback)
    rospy.init_node('pipetterController', anonymous=True)
    rate = rospy.Rate(100) # Publish z-position every 10ms
    

    while not rospy.is_shutdown():
        rospy.loginfo("Success!")
        exit()
        rate.sleep

if __name__ == "__main__":
    pipetter = zeus.ZeusModule(1)
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROSInterruptException!")
        pass
