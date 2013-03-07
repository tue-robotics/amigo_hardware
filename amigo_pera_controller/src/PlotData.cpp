#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PlotData.hpp"

using namespace std;
using namespace RTT;
using namespace PERA;

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
	addPort("port9", port9);
	addPort("port10", port10);
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
	port9Data.resize(8);
	port10Data.resize(8);
	
	dataMsg.data.resize(11);
		
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
	port9.read(port9Data);
	port10.read(port10Data);
	
	//dataMsg.data[0]=port1Data[selectJoint];
	//dataMsg.data[1]=port2Data[selectJoint];
	//dataMsg.data[2]=port3Data[selectJoint];
	//dataMsg.data[3]=port4Data[selectJoint];
	//dataMsg.data[4]=port5Data[selectJoint];
	//dataMsg.data[5]=port7Data[selectJoint];
	//dataMsg.data[6]=port6Data[selectMotor1];
	//dataMsg.data[7]=port6Data[selectMotor2];
	//dataMsg.data[8]=port8Data[selectMotor1];
	//dataMsg.data[9]=port8Data[selectMotor2];
	
	
	dataMsg.data[0]=port1Data[selectJoint];
	dataMsg.data[1]=port2Data[selectJoint];
	dataMsg.data[2]=port3Data[selectJoint];
	dataMsg.data[3]=port3Data[selectJoint+1];
	dataMsg.data[4]=port4Data[selectJoint];
	dataMsg.data[5]=port5Data[selectJoint];
	dataMsg.data[6]=port6Data[selectMotor1];
	dataMsg.data[7]=port6Data[selectMotor2];
	dataMsg.data[8]=port8Data[selectJoint];
	dataMsg.data[9]=port9Data[selectJoint];
	dataMsg.data[10]=port10Data[selectJoint];
	
	dataPort.write(dataMsg);
}

ORO_CREATE_COMPONENT(PERA::PlotData)
