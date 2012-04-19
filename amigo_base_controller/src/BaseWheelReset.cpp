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

    // Adding ports
    addPort( "safe", safePort );
    addPort( "emergency", rosEmergencyPort );
    addPort( "inputRef", inputRefPort) ;
    addEventPort( "measuredPos", measuredPosPort );

    addPort( "outputRef", outputRefPort );

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
    correctionPos.assign(3,0.0);

    return true;
}

void BaseWheelReset::updateHook()
{
    std_msgs::Bool button;
    rosEmergencyPort.read( button );
    safePort.read( safe );
    if ( !safe || button.data )
        currentStatus = UNSAFE;
    else
        currentStatus = OK;

    measuredPosPort.read( measInPos );

    // Only update correction pos when released from an unsafe situation
    if (currentStatus == OK && previousStatus != OK)
    {
        inputRefPort.read( refInPos );
        for ( uint i = 0; i < 3; i++ ){
            correctionPos[i] = refInPos[i] - measInPos[i];
        }
    }

    for ( uint i = 0; i < 3; i++){
        OutPos[i] = measInPos[i] + correctionPos[i];
    }
    outputRefPort.write( OutPos );

}

ORO_CREATE_COMPONENT(AMIGO::BaseWheelReset)
