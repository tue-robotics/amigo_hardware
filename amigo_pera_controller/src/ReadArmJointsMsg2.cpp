#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ReadArmJointsMsg2.hpp"

using namespace std;
using namespace RTT;
using namespace PERA;

ReadArmJointsMsg2::ReadArmJointsMsg2(const string& name) : TaskContext(name, PreOperational)
{
  // Creating ports:
  addPort( "joint_references", inport );
  addPort( "pos", posport );
  addPort( "vel", velport );
  addPort( "acc", accport );
  addPort( "enablePort", enablePort );
  
  // Loading properties
  addProperty( "offsets", OFFSET_VALUES );
  addProperty( "signs", SIGNAL_SIGNS );
}
ReadArmJointsMsg2::~ReadArmJointsMsg2(){}

bool ReadArmJointsMsg2::configureHook()
{
  return true;
}

bool ReadArmJointsMsg2::startHook()
{
  Logger::In in("ReadArmJointsMsg2::startHook()");
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
      log(Warning)<<"Outputport velport not connected!"<<endlog();
  }
  if ( !accport.connected() ) {
      log(Warning)<<"Outputport accport not connected!"<<endlog();
  }
  if ( !enablePort.connected() ) {
      log(Warning)<<"Enableport not connected. No component outcome!"<<endlog();
  }
  
  pos.resize(8.0,0.0);
  vel.resize(8.0,0.0);
  acc.resize(8.0,0.0);
  jointdataref1.resize(10.0,0.0); 
  jointdataref2.resize(10.0,0.0); 
  jointdataref1[0] = 0.0;
  jointdataref1[1] = 0.5; 
  jointdataref1[2] = 0.3; 
  jointdataref1[3] = 0.5;
  jointdataref1[4] = 0.0;
    
  jointdataref2[0] =  0.0;
  jointdataref2[1] = -0.5; 
  jointdataref2[2] =  0.5; 
  jointdataref2[3] = -0.5;
  jointdataref2[4] =  0.0;  
  counter = 0;
  
  goodtogo = false;
  enable = false;
  return true;
}


void ReadArmJointsMsg2::updateHook()
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

	  jointdata.pos[2].data = jointdataref2[counter];
		  
	  for ( uint i = 0; i < 7; i++ ){ 
		  pos[i] = SIGNAL_SIGNS[i]*(jointdata.pos[i].data+OFFSET_VALUES[i]);
		  vel[i] = 0.0;
		  acc[i] = 0.0;
	  }
	  
	  
	  
	  // Write data to port
	  posport.write( pos );
	  velport.write( vel );
	  accport.write( acc );
	  
  }

  goodtogo = true;
  counter++;
  
}

ORO_CREATE_COMPONENT(PERA::ReadArmJointsMsg2)
