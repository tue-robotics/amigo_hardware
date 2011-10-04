#ifndef WriteArmDataMsg_HPP
#define WriteArmDataMsg_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float32MultiArray.h>


using namespace RTT;

namespace PERA
{

  typedef vector<double> doubles;
  
  /*! \class PlotData
   *  \brief Defines Orocos component for runtime plotting PERA data.
   * 
   * This component gathers information from specified orocos component
   * ports specified within a deployerfile. However it is specifically
   * for usage with the PERA deployerfile for quickly assessing data
   * channels.
   */

  class PlotData
  : public RTT::TaskContext
    {
    private:

	//! Inputport 1
    InputPort<doubles> port1;
    //! Inputport 2
    InputPort<doubles> port2;
    //! Inputport 3
    InputPort<doubles> port3;
    //! Inputport 4
    InputPort<doubles> port4;
    //! Inputport 5
    InputPort<doubles> port5;
    //! Inputport 6
    InputPort<doubles> port6;
    //! Inputport 7
    InputPort<doubles> port7;
    //! Inputport 8
    InputPort<doubles> port8;
    //! Inputport 9
    InputPort<doubles> port9;
    //! Inputport 10
    InputPort<doubles> port10;
    //! Outputport for outputting the gathered data
    OutputPort<std_msgs::Float32MultiArray> dataPort;
    
    //! Selects the joint to be monitored
    int selectJoint;
    //! Selects the first motor to be monitored
    int selectMotor1; 
    //! Selects the second motor to be monitored
    int selectMotor2;
    //! Vectors for storing received data
    doubles port1Data, port2Data, port3Data, port4Data, port5Data, port6Data, port7Data, port8Data, port9Data, port10Data;
    //! MultiArray ROS msg for outputting the gathered data
    std_msgs::Float32MultiArray dataMsg;
	
	public:

    PlotData(const string& name);
    ~PlotData();

    //! Configuration sequence, executed before startHook()
	bool configureHook();
	//! Starting sequence, executed once upon startup of the component
	bool startHook();
	//! Update sequence, performed at specified rate
	void updateHook();
    
    };
}
#endif
