#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "WriteArmJointsMsg.hpp"

#define PI 3.1415926535897932384626433

using namespace RTT;
using namespace PERA;

WriteArmJointsMsg::WriteArmJointsMsg(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "pos", inport );
  addPort( "joint_references", outport );
  
  // Loading properties
  addProperty ("offsets", OFFSET_VALUES);
  addProperty ("signs", SIGNAL_SIGNS);

}

WriteArmJointsMsg::~WriteArmJointsMsg(){}

bool WriteArmJointsMsg::configureHook()
{
  return true;
}

bool WriteArmJointsMsg::startHook()
{
  Logger::In in("WriteArmJointsMsg::startHook()");
  // Check validity of Ports:
  if ( !inport.connected() )
    {
      log(Error)<<"Inputport not connected!"<<endlog();
      return false;
    }
  if ( !inport.connected() ) {
      log(Warning)<<"Outputport not connected!"<<endlog();
  }

  return true;
}


void WriteArmJointsMsg::updateHook()
{
  
  // Define vector for inport reading
  doubles angles(8,0.0);
  
  // Read the inputport  
  inport.read(angles);
  
  // Define ROS message to be filled up and published
  amigo_msgs::arm_joints jointdata;
  
  // Change sign and add offset (wrt inverse kinematics)
  for ( uint i = 0; i < 7; i++ )
	{
	  jointdata.pos[i].data = angles[i]*SIGNAL_SIGNS[i]-OFFSET_VALUES[i];
	}

  // Write data to port (ie publishing on ROS topic)
  outport.write( jointdata );

  
}

ORO_CREATE_COMPONENT(PERA::WriteArmJointsMsg)
