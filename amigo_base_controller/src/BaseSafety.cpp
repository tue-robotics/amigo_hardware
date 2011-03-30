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
#include <ocl/Component.hpp>

#include "BaseSafety.hpp"

using namespace RTT;
using namespace AMIGO;

BaseSafety::BaseSafety(const string& name) :
	                    		    TaskContext(name, PreOperational)
{
  max_voltage = 0;

  // Adding properties
  addProperty( "max_velocities", max_velocities );
  addProperty( "max_errors", max_errors );
  addProperty( "max_voltage", max_voltage );

  // Adding ports
  addEventPort( "error", errorport );
  addEventPort( "ref", refport );
  addEventPort( "voltage", voltport );
  addPort( "out", outport );
}

BaseSafety::~BaseSafety(){}

bool BaseSafety::configureHook()
{
  Logger::In in("BaseSafety::configureHook()");
  if ( max_errors.size() < 3 ) //TODO: Compare with array size
  {
    log(Error)<<"Maximum errors not specified!"<<endlog();
    return false;
  }
  if ( max_velocities.size() != 3 )
  {
    log(Error)<<"Maximum velocities not specified!"<<endlog();
    return false;
  }
  if ( max_voltage == 0 )
  {
    log(Warning)<<"Maximum voltage is zero!"<<endlog();
  }
  return true;
}

bool BaseSafety::startHook()
{
  Logger::In in("BaseSafety::startHook()");

  // Check validity of Ports
  if ( !refport.connected() || !errorport.connected() || !voltport.connected() )
  {
    log(Error)<<"Input port not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() )
  {
    log(Warning)<<"Output port not connected!"<<endlog();
  }

  safe = true;

  return true;
}

void BaseSafety::updateHook()
{
  Logger::In in("BaseSafety::updateHook()");

  if (safe) //Safetychecks only usefull if safe is true
  {

    // Check validity of Ports
    if ( !refport.connected() || !errorport.connected() || !voltport.connected() )
    {
      log(Error)<<"Input port not connected!"<<endlog();
      // No connection was made, can't do my job !
      safe = false;
    }

    if ( !outport.connected() )
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
  outport.write( safe );
}

ORO_CREATE_COMPONENT(AMIGO::BaseSafety)
