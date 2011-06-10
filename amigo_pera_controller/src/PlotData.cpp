#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "PlotData.hpp"

using namespace RTT;
using namespace AMIGO;

PlotData::PlotData(const string& name) : TaskContext(name, PreOperational)
{
	addPort("port1", port1);
	addPort("port2", port2);
	addPort("port3", port3);
	addPort("port4", port4);
	addPort("port5", port5);
	addPort("port6", port6);
	addPort("port7", port7);
	addPort("port8", port8);
	addPort("dataPort", dataPort);
	
	addProperty("selectJoint", selectJoint);
	addProperty("selectMotor1", selectMotor1);
	addProperty("selectMotor2", selectMotor2);
}
PlotData::~PlotData(){}

bool PlotData::configureHook()
{
  return true;
}

bool PlotData::startHook()
{
	port1Data.resize(8);
	port2Data.resize(8);
	port3Data.resize(8);
	port4Data.resize(8);
	port5Data.resize(8);
	port6Data.resize(8);
	port7Data.resize(8);
	port8Data.resize(8);
	
	dataMsg.data.resize(10);
		
	return true;
}


void PlotData::updateHook()
{
	port1.read(port1Data);
	port2.read(port2Data);
	port3.read(port3Data);
	port4.read(port4Data);
	port5.read(port5Data);
	port6.read(port6Data);
	port7.read(port7Data);
	port8.read(port8Data);
	
	dataMsg.data[0]=port1Data[selectJoint];
	dataMsg.data[1]=port2Data[selectJoint];
	dataMsg.data[2]=port3Data[selectJoint];
	dataMsg.data[3]=port4Data[selectJoint];
	dataMsg.data[4]=port5Data[selectJoint];
	dataMsg.data[5]=port6Data[selectJoint];
	dataMsg.data[6]=port6Data[selectMotor1];
	dataMsg.data[7]=port6Data[selectMotor2];
	dataMsg.data[8]=port8Data[selectMotor1];
	dataMsg.data[9]=port8Data[selectMotor2];
	
	dataPort.write(dataMsg);
}

ORO_CREATE_COMPONENT(AMIGO::PlotData)
