/** BaseReset.cpp
 *
 * @class BaseReset
 *
 * \author Tim Clephas
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "BaseReset.hpp"

using namespace RTT;
using namespace AMIGO;

BaseReset::BaseReset(const string& name) :
	                    		    TaskContext(name, PreOperational)
{

  // Adding ports
  addEventPort( "safe", safeport );
  addEventPort( "ros", rosport );
  addPort( "reset", resetport );
  addEventPort( "pos", posport );
  addPort( "integratorreset", integratorresetport );
}

BaseReset::~BaseReset(){}

bool BaseReset::configureHook()
{

  return true;
}

bool BaseReset::startHook()
{


  return true;
}

void BaseReset::updateHook()
{
  Logger::In in("BaseReset::updateHook()");



  safeport.read( safe );
  if ( !safe )
  {
	  doubles pos;
  	  posport.read( pos );
  	  integratorresetport.write( pos );

  	  std_msgs::Bool rosboolmsg;
  	  if (rosport.read( rosboolmsg ) == NewData )
  	  {
  		  log(Info)<<"Received reset command from ROS"<<endlog();
  		  resetport.write( true );
  	  }
  }

  /*if (safe) //Safetychecks only usefull if safe is true
  {

    // Check validity of Ports
    if ( !refport.connected() || !errorport.connected() || !voltport.connected() )
    {
      log(Error)<<"Input port not connected!"<<endlog();
      // No connection was made, can't do my job !
      safe = false;
    }

    if ( !amplifierport.connected() )
    {
      log(Warning)<<"Output port not connected!"<<endlog();
    }

    doubles refs(3);
    refport.read( refs );
    for ( uint i = 0; i < 3; i++ )
      if ( refs[i] > max_velocities[i] )
      {
        safe = false;
        log(Error)<<"Maximum reference velocity exeeded! Disabling hardware!"<<endlog();
      }

    doubles errors(3);
    errorport.read( errors );
    for ( uint i = 0; i < 3; i++ )
      if ( errors[i] > max_errors[i] )
      {
        safe = false;
        log(Error)<<"Maximum errors exeeded! Disabling hardware!"<<endlog();
      }

    doubles voltage(4);
    voltport.read( voltage );
    for ( uint i = 0; i < 4; i++ )
      if ( voltage[i] > max_voltage )
      {
        safe = false;

        log(Error)<<"Maximum voltage exeeded! Disabling hardware!"<<endlog();
        log(Error)<<"Voltages: "<<voltage[0]<<"   "<<voltage[1]<<"   "<<voltage[2]<<"   "<<voltage[3]<<"   "<<voltage[4]<<"   "<<voltage[5]<<"   "<<voltage[6]<<"   "<<voltage[7]<<endlog();
      }
  }
  else if ( resetport.read( reset ) == NewData )
	  safe = true;
  amplifierport.write( safe );*/
}

ORO_CREATE_COMPONENT(AMIGO::BaseReset)
