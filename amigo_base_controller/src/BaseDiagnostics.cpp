/** BaseDiagnostics.cpp
 *
 * @class BaseDiagnostics
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "BaseDiagnostics.hpp"

using namespace RTT;
using namespace AMIGO;

BaseDiagnostics::BaseDiagnostics(const string& name) :
	                    		    		TaskContext(name, PreOperational)
{

	// Adding ports
	addEventPort( "safe", safeport );
	//addEventPort( "rosdiagnostics", rosdiagnosticsport );
	//addEventPort( "rosemergency", rosemergencyport );
	addPort( "diagnostics", diagnosticsport );
	addEventPort( "pos", posport );
	addPort( "integratordiagnostics", integratordiagnosticsport );
}

BaseDiagnostics::~BaseDiagnostics(){}

bool BaseDiagnostics::configureHook()
{

	return true;
}

bool BaseDiagnostics::startHook()
{


	return true;
}

void BaseDiagnostics::updateHook()
{
	vector<diagnostic_msgs::DiagnosticStatus> statuses;

	diagnostic_updater::DiagnosticStatusWrapper status;


	static bool first = true;
	if (first)
	{
		first = false;
		status.add("Robot Description", "AMIGO");
	}
	status.addf("Max EtherCAT roundtrip (us)", "%.2f", 5);
	status.addf("Avg EtherCAT roundtrip (us)", "%.2f", 7);

	status.name = "Realtime Control Loop";
	if (false) // Warning
	{
		if (true)
			status.level = 1;
		else
			status.level = 0;
		status.message = "Realtime loop used too much time in the last 30 seconds.";
	}
	else
	{
		status.level = 0;
		status.message = "OK";
	}

	if (false)
	{
		status.mergeSummaryf(status.ERROR, "Halting, realtime loop only ran at %.4f Hz", 77);
	}
	;




	statuses.push_back(status);


	diagnostic_msgs::DiagnosticArray msg;

	msg.status=statuses;

	diagnosticsport.write( msg );
	log(Info)<<"publish"<<endlog();

}

ORO_CREATE_COMPONENT(AMIGO::BaseDiagnostics)
