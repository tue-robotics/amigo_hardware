/***************************************************************************
 tag: Janno Lunenburg  Fri August 13 16:00:00 CET 2013  PanTiltControllerJointState.hpp

 PanTiltControllerJointState.hpp -  description
 -------------------
 begin                : Sat February 19 2011
 copyright            : (C) 2011 Sava Marinkov
 email                : s.marinkov@student.tue.nl

 ***************************************************************************/

#ifndef PANTILTCONTROLLERJOINTSTATE_HPP
#define PANTILTCONTROLLERJOINTSTATE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <soem_beckhoff_drivers/CommMsgBig.h>
#include <sensor_msgs/JointState.h>
#include "AX12_control_table.h"
#include <queue>

#define RAD_TO_STEP             195.37860814
#define TRIAL_MAX 				20

using namespace std;
using namespace RTT;
using namespace soem_beckhoff_drivers;

namespace AMIGO {

typedef std::vector<double> doubles;

class PanTiltControllerJointState : public RTT::TaskContext {
	private:

		OutputPort<CommMsgBig> instructionPort;
		InputPort<CommMsgBig> statusPort;
		InputPort<sensor_msgs::JointState> goalPosPort;
		OutputPort<sensor_msgs::JointState> currentPosPort;
		
		InputPort<bool> serialRunningPort;
		InputPort<bool> serialReadyRxPort;
		
		CommMsgBig instruction;
		CommMsgBig status;
		sensor_msgs::JointState goalPos;
		int trial, newPosition;
		sensor_msgs::JointState currentPos;
		
		int state, pstate;
		int commStatus;
		unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10];
		unsigned char gbStatusPacket[MAXNUM_TXPARAM+10];
		int gbStatusSize;
		int pan_id, tilt_id, pan_goal, tilt_goal, pan_max, pan_min, tilt_max, tilt_min, pan_speed, tilt_speed, pan_offset, tilt_offset;
	public:
		PanTiltControllerJointState(const std::string& name);
		~PanTiltControllerJointState(){};

		bool configureHook();

		bool startHook();
	  
		void updateHook();

		void stopHook(){};
	
	private:
		bool readReference();
	
		void dxl_set_txpacket_instruction(int instruction);
		void dxl_tx_rx_packet(void);
		void dxl_set_txpacket_parameter(int index, int value);
		void dxl_set_txpacket_length(int length);
		
		void dxl_rx_packet(void);
		int dxl_get_rxpacket_error(int errbit);
		int dxl_rxpacket_isError(void);
		int dxl_get_rxpacket_length(void);
		int dxl_get_rxpacket_id(void);
		int dxl_get_rxpacket_parameter(int index);
		bool dxl_check_rxpacket_checksum(void);
		
		int dxl_makeword(int lowbyte, int highbyte);
		int dxl_get_lowbyte(int word);
		int dxl_get_highbyte(int word);

		void dxl_ping(int id);
		void dxl_read_byte(int id, int address);
		void dxl_write_byte(int id, int address, int value);
		void dxl_read_word(int id, int address);
		void dxl_write_word(int id, int address, int value);
		
		void dxl_tx_packet(void);
		void dxl_set_txpacket_id( int id );
		void printErrorCode(void);

};
}
#endif
