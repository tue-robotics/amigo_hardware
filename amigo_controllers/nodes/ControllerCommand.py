#!/usr/bin/env python
import roslib; roslib.load_manifest('amigo_controllers')

from amigo_controllers.srv import *
import rospy
from subprocess import call, Popen
#import subprocess as sp
from os import system, popen
import shlex
from time import sleep

def enable_controller(req):
	
	#RUN LOCALLY ON AMIGO2
	#preamble="ssh amigo@amigo2 'source /opt/ros/diamondback/setup.bash; ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros; ROS_MASTER_URI=http://amigo1:11311; "
	controllers_exist = [ False, False, False, False, False ]

	package_names = ("amigo_base_controller", "amigo_spindle_controller", "arm_left", "arm_right", "amigo_head_controller")
	command_prefixes = ("base_controller", "spindle_controller", "pera", "pera", "head_controller")
	allowed_commands = ("enable", "reset", "disable")
	if not req.command in allowed_commands:
		print "Command not valid!"
		return 0
		
	
	for (counter, package_name) in enumerate(package_names):	
		returncode = popen("ps x | grep " + package_name + "| grep -v grep | awk '{print $1}'").readlines()
		if len(returncode) > 0:
			controllers_exist[counter] = True
	
	print controllers_exist

		
	if ( req.command == "enable" ):
		if ( not controllers_exist[req.controller_number] ):
			#print "kill all, load all, start ", req.controller_number
			#system("ps x | grep deployer | grep -v grep | awk '{print $1}' | xargs kill")
			cmd = "ps x | grep amigo_etherCAT | grep -v grep | awk '{print $1}' | xargs kill"
			p = Popen(shlex.split(cmd))
			cmd = "roslaunch amigo_launch_files load_all_etherCAT_hardware.launch"
			p = Popen(shlex.split(cmd))
		else:
			cmd = "rosrun ocl cdeployer-gnulinux START -s `rospack find " + package_names[req.controller_number] + "`/" + command_prefixes[req.controller_number] + "_start.ops & sleep 5; kill $!"
			p = Popen(cmd, shell=True)
			sleep(5)
			p.terminate()
		
	if ( req.command == "disable" and controllers_exist[req.controller_number] ):
		cmd = "rosrun ocl cdeployer-gnulinux STOP -s `rospack find " + package_names[req.controller_number] + "`/" + command_prefixes[req.controller_number] + "_stop.ops & sleep 5; kill $!"
		p = Popen(cmd, shell=True)
		sleep(5)
		p.terminate()

		
	if ( req.command == "reset" and controllers_exist[req.controller_number] ):
			cmd = "ps x | grep amigo_etherCAT | grep -v grep | awk '{print $1}' | xargs kill"
			p = Popen(shlex.split(cmd))
			cmd = "roslaunch amigo_launch_files load_all_etherCAT_hardware.launch"
			p = Popen(shlex.split(cmd))


		#process = sp.Popen(cmd.split(" "), stdout=sp.PIPE, stderr=sp.PIPE, stdin=sp.PIPE, bufsize=1)

#ps x | grep amigo_base_controller | grep -v grep | awk '{print $1}' | xargs kill
	
	return ControllerCommandResponse(1)

def enable_controller_server():
	rospy.init_node('amigo_controllers')
	s = rospy.Service('enable_controller', ControllerCommand, enable_controller)
	print "Ready to enable controllers :)."
	rospy.spin()

if __name__ == "__main__":
	enable_controller_server()
