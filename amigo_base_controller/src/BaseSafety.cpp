/** BaseSafety.cpp
 *
 * @class BaseSafety
 *
 * \author Tim Clephas
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include "BaseSafety.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

BaseSafety::BaseSafety(const string& name) :
	                    		    TaskContext(name, PreOperational)
{
  max_velocities.assign(3,0.0);
  max_errors.assign(4,0.0);
  max_voltage = 0;

  // Adding properties
  addProperty( "max_velocities", max_velocities );
  addProperty( "max_errors", max_errors );
  addProperty( "max_voltage", max_voltage );

  // Adding ports
  addEventPort( "error", errorport );
  addEventPort( "ref", refport );
  addEventPort( "voltage", voltport );
  addEventPort( "reset", resetport );
  addPort( "wheel_amplifiers", amplifierport );
  addPort( "errortosupervisor", errortosupervisorPort );
}

BaseSafety::~BaseSafety(){}

bool BaseSafety::configureHook()
{
  if ( max_errors.size() < 4 ) //TODO: Compare with array size
  {
    log(Error)<<"BaseSafety::Maximum errors not (well) specified!"<<endlog();
    return false;
  }
  if ( max_velocities.size() != 3 )
  {
    log(Error)<<"BaseSafety::Maximum velocities not (well) specified!"<<endlog();
    return false;
  }
  if ( max_voltage == 0 )
  {
    log(Warning)<<"BaseSafety::Maximum voltage is zero!"<<endlog();
  }
  
  return true; 
}

bool BaseSafety::startHook()
{
  // Check validity of Ports
  if ( !refport.connected() || !errorport.connected() || !voltport.connected() )
  {
    log(Error)<<"BaseSafety::Input port not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !amplifierport.connected() )
  {
    log(Warning)<<"BaseSafety::Output port not connected!"<<endlog();
  }

  safe = true;
  previousSafe = true;

  return true;
}

void BaseSafety::updateHook()
{
  //Safetychecks only usefull if safe is true
  if (safe) 
  {

    // Check validity of Ports
    if ( !refport.connected() || !errorport.connected() || !voltport.connected() )
    {
      log(Error) << "BaseSafety::Input port not connected!" << endlog();
      // No connection was made, can't do my job!
      safe = false;
    }

    if ( !amplifierport.connected() )
    {
      log(Error) << "BaseSafety::Output port not connected!" << endlog();
    }

    doubles refs(3);
    refport.read( refs );
    for ( uint i = 0; i < 3; i++ )
      if ( fabs(refs[i]) > max_velocities[i] )
      {
		errortosupervisorPort.write(true);
        safe = false;
        ROS_ERROR_STREAM( "BaseSafety::Maximum reference velocity exeeded! Axis " << i << " Value " << refs[i] << " Disabling hardware!" );
        log(Error) << "BaseSafety::Maximum reference velocity exeeded! Axis " << i << " Value " << refs[i] << " Disabling hardware!" << endlog();
      }

    doubles errors(4);
    errorport.read( errors );
    for ( uint i = 0; i < 4; i++ )
      if ( fabs(errors[i]) > max_errors[i] )
      {
		errortosupervisorPort.write(true);
        safe = false;
        ROS_ERROR_STREAM( "BaseSafety::Maximum errors exeeded! Errors: "<< errors[0] << " " << errors[1] << " " << errors[2] << " " << errors[3] << " " << "Disabling hardware!" );
        log(Error) << "BaseSafety::Maximum errors exeeded! Errors: "<< errors[0] << " " << errors[1] << " " << errors[2] << " " << errors[3] << " " << "Disabling hardware!" << endlog();
      }

    doubles voltage(4);
    voltport.read( voltage );
    for ( uint i = 0; i < 4; i++ )
      if ( fabs(voltage[i]) > max_voltage )
      {
		errortosupervisorPort.write(true);
        safe = false;
		ROS_ERROR_STREAM( "BaseSafety::Maximum voltage exeeded! Disabling hardware!" );
        log(Error) << "BaseSafety::Maximum voltage exeeded! Disabling hardware!" << endlog();
        log(Error) << "BaseSafety::Voltages: "<<voltage[0]<<"   "<<voltage[1]<<"   "<<voltage[2]<<"   "<<voltage[3]<<"   "<<voltage[4]<<"   "<<voltage[5]<<"   "<<voltage[6]<<"   "<<voltage[7]<<endlog();
      }
  }
  else if ( resetport.read( reset ) == NewData )
  {
	  ROS_INFO_STREAM( "BaseSafety::Base enabled again" );
	  safe = true;
  }
  
  // One sample delay is build in to make sure the errors are resetted before enabling the amplifiers
  amplifierport.write( safe&&previousSafe ); //TODO: I think this came irrelevant after the new supervisor...
  
  previousSafe = safe;

}


void BaseSafety::stopHook()
{
	amplifierport.write(false);
	ROS_INFO_STREAM( "BaseSafety::Stopped BaseSafety again" );
	safe = false;
}

ORO_CREATE_COMPONENT(AMIGO::BaseSafety)
