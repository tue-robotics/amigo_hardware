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
	controllers_exist = [ False, False, False, False, False ]

	package_names = ("amigo_base_controller", "amigo_spindle_controller", "arm_left", "arm_right", "dynamixel_ethercat")
		
	
	for (counter, package_name) in enumerate(package_names):	
		returncode = popen("ps x | grep " + package_name + "| grep -v grep | awk '{print $1}'").readlines()
		if len(returncode) > 0:
			controllers_exist[counter] = True
	
	print controllers_exist
		
		
	if ( req.command == "enable" ):
		if ( controllers_exist[req.controller_number] ):
			print "start ", req.controller_number
			#call("rosrun ocl cdeployer-gnulinux -s `rospack find amigo_base_controller`/base_controller_start.ops & sleep 5; kill $!")
		else:
			print "kill all, load all, start ", req.controller_number
			call("/etc/ros/setup.bash; roslaunch amigo_launch_files all_etherCAT_hardware.launch")

		
	if ( req.command == "disable" ):#and controllers_exist[req.controller_number] ):
		stop_commands = ("base_controller_stop.ops", "spindle_controller_stop.ops", "arm_left_stop.ops", "arm_right_stop.ops", "dynamixel_ethercat_stop.ops")
		command = "/etc/ros/setup.bash; rosrun ocl cdeployer-gnulinux -s `rospack find " + package_names[req.controller_number] + "`/" + stop_commands[req.controller_number] + " & sleep 5; kill $!"
		print command
		call(command)

#ps x | grep amigo_base_controller | grep -v grep | awk '{print $1}' | xargs kill

	
	
	return ControllerCommandResponse(1)

def enable_controller_server():
	rospy.init_node('amigo_controllers')
	s = rospy.Service('enable_controller', ControllerCommand, enable_controller)
	print "Ready to enable controllers :)."
	rospy.spin()

if __name__ == "__main__":
	enable_controller_server()
