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
	wheel_controllers_exist = False
	spindle_controllers_exist = False
	head_controllers_exist = False
	
	
	returncode = popen("ps x | grep amigo_base_controller | grep -v grep | awk '{print $1}'").readlines()
	if len(returncode) > 0:
		wheel_controllers_exist = True
	print wheel_controllers_exist
	
	returncode = popen("ps x | grep amigo_spindle_controller | grep -v grep | awk '{print $1}'").readlines()
	if len(returncode) > 0:
		spindle_controllers_exist = True
	print spindle_controllers_exist	
		
	returncode = popen("ps x | grep dynamixel_ethercat | grep -v grep | awk '{print $1}'").readlines()
	if len(returncode) > 0:
		head_controllers_exist = True
	print head_controllers_exist
		
	if ( req.command == "enable" and req.controller_number == 0 ):
		if ( wheel_controllers_exist ):
			print "base_controller_start.ops"
			#call("rosrun ocl cdeployer-gnulinux -s `rospack find amigo_base_controller`/base_controller_start.ops & sleep 5; kill $!")
		else:
			print "all_etherCAT_hardware.launch"
			#call("roslaunch amigo_launch_files all_etherCAT_hardware.launch")

		
	if ( req.command == "disable" and req.controller_number == 0 ):
		if ( wheel_controllers_exist ):
			print "base_controller_stop.ops"
	#		call("rosrun ocl cdeployer-gnulinux -s `rospack find amigo_base_controller`/base_controller_stop.ops & sleep 5; kill $!")



	
	
	return ControllerCommandResponse(1)

def enable_controller_server():
	rospy.init_node('amigo_controllers')
	s = rospy.Service('enable_controller', ControllerCommand, enable_controller)
	print "Ready to enable controllers :)."
	rospy.spin()

if __name__ == "__main__":
	enable_controller_server()
