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
    addEventPort( "safe", safePort );
    addEventPort( "emergency", rosEmergencyPort );
    addEventPort( "inputRef", inputRefPort) ;
    addPort( "measuredPos", measuredPosPort );

    addPort( "outputRef", outputRefPort );

}

BaseWheelReset::~BaseWheelReset(){}

bool BaseWheelReset::configureHook()
{

    return true;
}

bool BaseWheelReset::startHook()
{


    return true;
}

void BaseWheelReset::updateHook()
{
    std_msgs::Bool button;
    rosEmergencyPort.read( button );
    safePort.read( safe );

    doubles refPos;

    if ( !safe || button.data )
    {

        // In case of emergency button or unsafe situation:
        // Use the measured position as input position for the feedback loop to ensure zero tracking error
        ///log(Warning)<<"BaseWheelReset: Resetting wheel position"<<endlog();
        measuredPosPort.read( refPos );

    }

    else
    {

        // During normal operation:
        // Use the input reference position for the feedback loop
        inputRefPort.read( refPos );
        ///log(Warning)<<"BaseWheelReset: Normal operation"<<endlog();

    }
    outputRefPort.write( refPos );
}

ORO_CREATE_COMPONENT(AMIGO::BaseWheelReset)
