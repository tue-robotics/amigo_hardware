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
  addEventPort( "rosreset", rosresetport );
  addEventPort( "rosemergency", rosemergencyport );
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
  std_msgs::Bool emergency;
  rosemergencyport.read( emergency );
  safeport.read( safe );
  if ( !safe )
  {
	  doubles pos;
  	  posport.read( pos );
  	  integratorresetport.write( pos );

  	  std_msgs::Bool rosboolmsg;
  	  if (rosresetport.read( rosboolmsg ) == NewData )
  	  {
  		  log(Info)<<"BaseReset::Received reset command from ROS"<<endlog();
  		  resetport.write( true );
  	  }
  }
  else if ( emergency.data )
  {
	  log(Info)<<"BaseReset::Button pressed"<<endlog();
	  doubles pos;
  	  posport.read( pos );
  	  integratorresetport.write( pos );
  }


}

ORO_CREATE_COMPONENT(AMIGO::BaseReset)
