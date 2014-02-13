#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ReadArmJointsMsg.hpp"

using namespace std;
using namespace RTT;
using namespace PERA;

ReadArmJointsMsg::ReadArmJointsMsg(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addEventPort( "joint_references", inport );
  addPort( "pos", posport );
  addPort( "vel", velport );
  addPort( "acc", accport );
  addPort( "enablePort", enablePort );
}
ReadArmJointsMsg::~ReadArmJointsMsg(){}

bool ReadArmJointsMsg::configureHook()
{
  return true;
}

bool ReadArmJointsMsg::startHook()
{
  Logger::In in("ReadArmJointsMsg::startHook()");
  // Check validity of Ports:
  if ( !inport.connected() )
    {
      log(Error)<<"Inputport not connected!"<<endlog();
      return false;
    }  
  if ( !posport.connected() ) {
      log(Warning)<<"Outputport posport not connected!"<<endlog();
  }
  if ( !velport.connected() ) {
      log(Debug)<<"Outputport velport not connected!"<<endlog();
  }
  if ( !accport.connected() ) {
      log(Debug)<<"Outputport accport not connected!"<<endlog();
  }
  if ( !enablePort.connected() ) {
      log(Warning)<<"Enableport not connected. No component outcome!"<<endlog();
  }
  
  pos.assign(8, 0.0);
  vel.assign(8, 0.0);
  acc.assign(8, 0.0);
  
  goodtogo = false;
  enable = false;
  return true;
}


void ReadArmJointsMsg::updateHook()
{ 
  
  /* Check if watchdog allows reading of inv. kin. joint coordinates.
   * Enable is false unless watchdog states otherwise.
   */
   
  enablePort.read(enable);

  /* Workaround if statement. Prevents first cycle from having any effect.
   * If not implemented the first cycle it will read all joint angles
   * should be zero (default port value) and it will go to OFFSET_VALUES
   * which is undesired.
   */
  if (goodtogo && enable)
  {
	  // Read the inputport
	  amigo_msgs::arm_joints jointdata;

	  if( inport.read(jointdata) == NewData){
		  for ( uint i = 0; i < 7; i++ ){
			  pos[i] = jointdata.pos[i].data;
			  vel[i] = jointdata.vel[i].data;
			  acc[i] = jointdata.acc[i].data;
		  }
	  }
	  
	  // Write data to port
	  posport.write( pos );
	  velport.write( vel );
	  accport.write( acc );
	  
  }

  goodtogo = true;
  
}

ORO_CREATE_COMPONENT(PERA::ReadArmJointsMsg)
