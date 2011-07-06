#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <amigo_ref_interpolator/interpolator.h>

#include "PERA_FF_Refgen.hpp"

using namespace RTT;
using namespace PERA;

PERA_Refgen::PERA_Refgen(const string& name) : TaskContext(name, PreOperational)
{
  
  addProperty( "ActuatedJoint", actJnt );
  addProperty( "InterpolatorDt", InterpolDt );
  addProperty( "InterpolatorEps", InterpolEps );
  addProperty( "NrInterpolators", NrInterpolators );
  
}

PERA_Refgen::~PERA_Refgen(){}

bool PERA_Refgen::configureHook()
{
    
  Logger::In in("PERA_Refgen::configureHook()");

  addPort( "posout", posoutport );
 
  mRefGenerators.resize(NrInterpolators);
  desiredPos.resize(NrInterpolators);
  mRefPoints.resize(NrInterpolators);
  
  for ( uint i = 0; i < NrInterpolators; i++ ){
    string name = "interpolator"+to_string(i+1);
    addProperty( name, interpolators[i]);
  }
  
  cnt=0;

  return true;

}

bool PERA_Refgen::startHook()
{
  Logger::In in("PERA_Refgen::startHook()");

  if ( !posoutport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
    // No connection was made, can't do my job !
  }
  
  
  for ( uint i = 0; i < NrInterpolators; i++ ){
	  // Set the initial desired position to the one stored in the property
	  desiredPos[i] = interpolators[i][0];
  }
  
  // Compute corresponding time unit T and maximum acceleration of the actuated joint
  T=1.5*interpolators[(int)actJnt-1][0]/interpolators[(int)actJnt-1][1];
  interpolators[(int)actJnt-1][2] = 3*interpolators[(int)actJnt-1][1]/T;
  log(Debug)<<"T = "<<T<<"and Amax = "<<interpolators[(int)actJnt-1][2]<<endlog();
  
  initTime = os::TimeService::Instance()->getNSecs();
  
  return true;
}


void PERA_Refgen::updateHook()
{
  
  timeNow = (os::TimeService::Instance()->getNSecs())-initTime;
  //log(Debug)<<"timeNow outside if = "<<(double)(timeNow/1E9)<<endlog();
  
  if ((double)(timeNow/1E9) >= T){
      log(Debug)<<"timeNow inside if = "<<(double)(timeNow/1E9)<<endlog();
      initTime = os::TimeService::Instance()->getNSecs();
      
      if ( cnt % 2 == 0 ){
            desiredPos[actJnt-1]=0.0;
            log(Debug)<<"desiredPos = 0.0"<<endlog();
      }
      else {
         desiredPos[actJnt-1]=interpolators[(int)actJnt-1][0];
         log(Debug)<<"desiredPos = "<<desiredPos[actJnt-1]<<endlog();
      }
      
      cnt++;
  }
  
  // Read the inputports
  doubles outpos(NrInterpolators,0.0);
  
  // Compute the next reference points
  for ( uint i = 0; i < NrInterpolators; i++ ){
      mRefPoints[i] = mRefGenerators[i].generateReference(desiredPos[i], interpolators[i][1], interpolators[i][2], InterpolDt, false, InterpolEps);
      outpos[i]=mRefPoints[i].pos;
  }

  // Write new positions
  posoutport.write( outpos );

}

ORO_CREATE_COMPONENT(PERA::PERA_Refgen)
