/** BaseWheelReset.cpp
 *
 * @class BaseWheelReset
 *
 * \author Janno Lunenburg
 * \date April, 2012
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "BaseWheelReset.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;

BaseWheelReset::BaseWheelReset(const string& name) :
                                                    TaskContext(name, PreOperational)
{

    /// Adding ports
    addPort( "safe", safePort );
    addPort( "emergency", rosEmergencyPort );
    addPort( "inputRef", inputRefPort) ;
    addEventPort( "measuredPos", measuredPosPort );

    addPort( "outputPos", outputPosPort );
    addPort( "outputOdom", outputOdomPort );

}

BaseWheelReset::~BaseWheelReset(){}

bool BaseWheelReset::configureHook()
{

    return true;
}

bool BaseWheelReset::startHook()
{

    currentStatus = NO_INFO;
    previousStatus = NO_INFO;
    correctionPos.assign(4,0.0);
    correctionOdom.assign(4,0.0);
    measInPos.assign(4,0.0);
    refInPos.assign(4,0.0);
    OutPos.assign(4,0.0);
    OutOdom.assign(4,0.0);

    return true;
}

void BaseWheelReset::updateHook()
{
    std_msgs::Bool button;
    rosEmergencyPort.read( button );
    safePort.read( safe );
    if ( !safe || button.data )
    {
        currentStatus = UNSAFE;
    }
    else
    {
		currentStatus = OK;
	}
	
	measuredPosPort.read( measInPos );
	inputRefPort.read( refInPos );
		
    /// Continuously update position output port
	if ( !safe || button.data )
    {
		for ( uint i = 0; i < 4; i++ )
        {	
            correctionPos[i] = refInPos[i] - measInPos[i];   
        }
    }
    
    /// Only update odom output when released from an unsafe situation
	if (currentStatus == OK && previousStatus != OK)
    {	
		log(Warning) << "Resetting correctionOdom" << endlog();
        for ( uint i = 0; i < 4; i++ )
        {
            correctionOdom[i] = refInPos[i] - measInPos[i];
        }
    }
	
	/// Update the output positions
    for ( uint i = 0; i < 4; i++)
    {
        OutPos[i] = measInPos[i] + correctionPos[i];
        OutOdom[i] = measInPos[i] + correctionOdom[i];
    }
	
	//TODO: Remove testloop
	if (currentStatus == OK && previousStatus != OK)
	{
		log(Warning) << "refInPos\t" << refInPos[0] << "\t" << refInPos[1] << "\t" << refInPos[2] << "\t" << refInPos[3] << endlog();
        log(Warning) << "OutPos\t" << OutPos[0] << "\t" << OutPos[1] << "\t" << OutPos[2] << "\t" << OutPos[3] << endlog();
        log(Warning) << "OutOdom\t" << OutOdom[0] << "\t" << OutOdom[1] << "\t" << OutOdom[2] << "\t" << OutOdom[3] << endlog();
	}
	
    outputPosPort.write( OutPos );
    outputOdomPort.write( OutOdom );
    
    previousStatus = currentStatus;

}

ORO_CREATE_COMPONENT(AMIGO::BaseWheelReset)
