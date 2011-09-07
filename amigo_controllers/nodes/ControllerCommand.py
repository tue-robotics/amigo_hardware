#!/usr/bin/env python
import roslib; roslib.load_manifest('amigo_controllers')

from amigo_controllers.srv import *
import rospy

def enable_controller(req):
    print "Returning [%s + %s]"%(req.controller_number, req.command)
    return ControllerCommandResponse(1)

def enable_controller_server():
    rospy.init_node('amigo_controllers')
    s = rospy.Service('enable_controller', ControllerCommand, enable_controller)
    print "Ready to enable controllers :)."
    rospy.spin()

if __name__ == "__main__":
    enable_controller_server()
