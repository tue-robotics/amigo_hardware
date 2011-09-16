#!/usr/bin/env python
import roslib; roslib.load_manifest('amigo_controllers')

from amigo_controllers.srv import *
import rospy
from subprocess import call
from os import popen

def enable_controller(req):
    print "Returning [%s + %s]"%(req.controller_number, req.command)
    
    #RUN LOCALLY ON AMIGO2
    #preamble="ssh amigo@amigo2 'source /opt/ros/diamondback/setup.bash; ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros; ROS_MASTER_URI=http://amigo1:11311; "
    
    returncode = popen("ps x | grep amigo_base_controller | grep -v grep | awk '{print $1}'").readlines()
    if len(returncode) > 0:
        wheel_controllers_exist = true
    returncode = popen("ps x | grep amigo_spindle_controller | grep -v grep | awk '{print $1}'").readlines()
    if len(returncode) > 0:
        self.spindle_controllers_exist = true
        
    if ( req.command == "start" and req.command == 0 ):
        if ( wheel_controllers_exist ):
            call("rosrun ocl cdeployer-gnulinux -s `rospack find amigo_base_controller`/base_controller_start.ops & sleep 5; kill $!")
        else:
            call("roslaunch amigo_launch_files all_etherCAT_hardware.launch")


    
    
    return ControllerCommandResponse(1)

def enable_controller_server():
    rospy.init_node('amigo_controllers')
    s = rospy.Service('enable_controller', ControllerCommand, enable_controller)
    print "Ready to enable controllers :)."
    rospy.spin()

if __name__ == "__main__":
    enable_controller_server()
