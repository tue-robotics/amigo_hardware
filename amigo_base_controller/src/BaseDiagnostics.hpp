/** BaseDiagnostics.hpp
 *
 * @class BaseDiagnostics
 *
 * \author Tim Clephas
 * \date May, 2011
 * \version 1.0
 *
 */

#ifndef BASEDIAGNOSTICS_HPP
#define BASEDIAGNOSTICS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>

using namespace RTT;

namespace AMIGO
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class BaseDiagnostics
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;

    /* Declaring and output port*/
    InputPort<bool> safeport;
    //InputPort<std_msgs::Bool> rosdiagnosticsport;
    //InputPort<std_msgs::Bool> rosemergencyport;
    OutputPort<diagnostic_msgs::DiagnosticArray> diagnosticsport;

    InputPort<doubles> posport;
    OutputPort<doubles> integratordiagnosticsport;

    /* Declaring property variables */
    doubles max_velocities;
    doubles max_errors;
    double max_voltage;

    /* Declaring global variables */
    bool safe;

    public:

    BaseDiagnostics(const string& name);
    ~BaseDiagnostics();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
