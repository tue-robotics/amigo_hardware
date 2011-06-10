#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "ReadArmJointsMsg.hpp"

using namespace RTT;
using namespace PERA;

ReadArmJointsMsg::ReadArmJointsMsg(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addProperty( "offsets", offsetValues );
  addProperty( "signs", signalSigns );
  addEventPort( "joint_coordinates", inport );
  addEventPort( "gripper_pos", ingripperport );
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
  if ( !ingripperport.connected() )
    {
      log(Error)<<"Inputport for gripper position is not connected!"<<endlog();
    }    
  if ( !posport.connected() ) {
      log(Warning)<<"Outputport posport not connected!"<<endlog();
  }
  if ( !velport.connected() ) {
      log(Warning)<<"Outputport velport not connected!"<<endlog();
  }
  if ( !accport.connected() ) {
      log(Warning)<<"Outputport accport not connected!"<<endlog();
  }
  if ( !enablePort.connected() ) {
      log(Warning)<<"homedPort not connected. No component outcome!"<<endlog();
  }
  
  goodtogo = false;
  bool enable = false;
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
   * should be zero (default port value) and it will go to offsetvalues
   * which is undesired.
   */
  if (goodtogo && enable)
  {
	  // Read the inputport
	  amigo_msgs::arm_joints jointdata;
	  double gripper_pos;
	  doubles pos(8,0.0);
	  doubles vel(8,0.0);
	  doubles acc(8,0.0);

	  if( inport.read(jointdata) == NewData){
		  for ( uint i = 0; i < 7; i++ ){
			  pos[i] = signalSigns[i]*(jointdata.pos[i].data+offsetValues[i]);
			  vel[i] = jointdata.vel[i].data;
			  acc[i] = jointdata.acc[i].data;
		  }
	  }
	  
	  if( ingripperport.read(gripper_pos) ==NewData){
		  pos[7] = signalSigns[7]*(gripper_pos+offsetValues[7]);
	  }
	  
	  // Write data to port
	  posport.write( pos );
	  velport.write( vel );
	  accport.write( acc );
	  
  }

  goodtogo = true;
  
}

ORO_CREATE_COMPONENT(PERA::ReadArmJointsMsg)
