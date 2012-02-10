/***************************************************************************
 tag: Sava Marinkov  Thu May 12 12:50:00 CET 2011  PanTiltController.cpp

 PanTiltController.cpp -  description
 -------------------
 begin                : Sat February 19 2011
 copyright            : (C) 2011 Sava Marinkov
 email                : s.marinkov@student.tue.nl

 ***************************************************************************/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>
#include "PanTiltController.hpp"

using namespace RTT;
using namespace soem_beckhoff_drivers;
using namespace AMIGO;

PanTiltController::PanTiltController(const std::string& name) :
	TaskContext(name, PreOperational) {
	addPort("instruction", instructionPort).doc("Dynamixel instruction packet port");
	addPort("status", statusPort).doc("Dynamixel status packet port");
	addPort("serialRunning", serialRunningPort).doc("Serial device running port");
	addPort("serialReadyRx", serialReadyRxPort).doc("Serial device ready receive port");	
	addPort("goalPos", goalPosPort).doc("Goal head position");
	addPort("currentPosPan", currentPosPanPort).doc("Current head pan position");
	addPort("currentPosTilt", currentPosTiltPort).doc("Current head tilt position");
	addProperty( "pan_id", pan_id).doc("Pan dynamixel id");
	addProperty( "tilt_id", tilt_id).doc("Tilt dynamixel id");
	addProperty( "pan_max", pan_max).doc("Pan max angle, 0 to 1023");
	addProperty( "pan_min", pan_min).doc("Pan min angle, 0 to 1023");
	addProperty( "tilt_max", tilt_max).doc("Tilt max angle, 0 to 1023");
	addProperty( "tilt_min", tilt_min).doc("Tilt min angle, 0 to 1023");
	addProperty( "pan_speed", pan_speed).doc("Pan speed, 0 to 1023");
	addProperty( "tilt_speed", tilt_speed).doc("Tilt speed, 0 to 1023");
	addProperty( "pan_offset", pan_offset).doc("Pan offset angle, 0 to 1023");
	addProperty( "tilt_offset", tilt_offset).doc("Tilt offset angle, 0 to 1023");
	// ToDO: Add status port, check http://www.google.com/url?q=http://www.electronickits.com/robot/BioloidAX-12(english).pdf&sa=U&ei=xYA1T5r6IYLBhAfHlZnqAQ&ved=0CAQQFjAA&client=internal-uds-cse&usg=AFQjCNFq1o8ghhtWNHEBkxUA5kbnz17hpw
	log(Debug) << "PanTiltController constructor done." << endlog();
}

bool PanTiltController::configureHook() {
	state = 1;
	for (int i = 0; i < MAXNUM_TXPARAM+10; i++) {
		gbInstructionPacket[i] = 0;
		gbStatusPacket[i] = 0;
	}
	currentPan.data = 0.0;
	currentTilt.data = 0.0;
	newPosition = 0;
	commStatus = COMM_RXSUCCESS;
	
	log(Debug) << "PanTiltController configuration done." << endlog();
	return true;
}

bool PanTiltController::startHook() {
	log(Debug) << "pan_id " << pan_id << endlog();
	log(Debug) << "tilt_id " << tilt_id << endlog();
	log(Debug) << "pan_max " << pan_max << endlog();
	log(Debug) << "pan_min " << pan_min << endlog();
	log(Debug) << "tilt_max " << tilt_max << endlog();
	log(Debug) << "tilt_min " << tilt_min << endlog();
	log(Debug) << "pan_speed " << pan_speed << endlog();
	log(Debug) << "tilt_speed " << tilt_speed << endlog();
	log(Debug) << "pan_offset " << pan_offset << endlog();
	log(Debug) << "tilt_offset " << tilt_offset << endlog();	
	log(Debug) << "PanTiltController start done." << endlog();
	return true;
}

bool PanTiltController::readReference() {
	if (goalPosPort.read(goalPos) == NewData) {
		log(Debug) << "PanTiltController: new pan/tilt reference obtained."<< endlog();
		pan_goal = (int) ((goalPos.head_pan)*RAD_TO_STEP+pan_offset);
		if (pan_goal> pan_max)
			pan_goal = pan_max;
		if (pan_goal < pan_min)
			pan_goal = pan_min;

		tilt_goal = (int) ((goalPos.head_tilt)*RAD_TO_STEP+tilt_offset);
		if (tilt_goal > tilt_max)
			tilt_goal = tilt_max;
		if (tilt_goal < tilt_min)
			tilt_goal = tilt_min;
			
		return true;
	}
	return false;
}

void PanTiltController::updateHook() {
	bool serialRunning = false;
	if(!(serialRunningPort.read(serialRunning) == NewData)) {
		return;
	}
	if (commStatus == COMM_RXSUCCESS){
		trial = 0;
		pstate = state;
		switch (state) {
			case 1:
				log(Debug) << "PanTiltController: setting pan speed"<< endlog();
				dxl_write_word(pan_id, AX_MOVING_SPEED_L, pan_speed);
				//dxl_write_byte(pan_id, AX_STATUS_RETURN, 1);
				state++;
				break;
			case 2:
				log(Debug) << "PanTiltController: setting tilt speed"<< endlog();
				dxl_write_word(tilt_id, AX_MOVING_SPEED_L, tilt_speed);
				//dxl_write_byte(tilt_id, AX_STATUS_RETURN, 1);
				state++;
				break;
			case 3:
				log(Debug) << "PanTiltController: homing pan"<< endlog();
				dxl_write_word(pan_id, AX_GOAL_POSITION_L, pan_offset);
				state++;
				break;
			case 4:
				log(Debug) << "PanTiltController: homing tilt"<< endlog();
				dxl_write_word(tilt_id, AX_GOAL_POSITION_L, tilt_offset);
				state++;
				break;
			case 5:
				if (readReference()) {
					log(Debug) << "PanTiltController: syncwrite pan/tilt goal position"<< endlog();
					dxl_set_txpacket_id(BROADCAST_ID);
					dxl_set_txpacket_instruction(INST_SYNC_WRITE);
					dxl_set_txpacket_parameter(0, AX_GOAL_POSITION_L);
					dxl_set_txpacket_parameter(1, 2);
					dxl_set_txpacket_parameter(2, pan_id);
					dxl_set_txpacket_parameter(3, dxl_get_lowbyte(pan_goal));
					dxl_set_txpacket_parameter(4, dxl_get_highbyte(pan_goal));
					dxl_set_txpacket_parameter(5, tilt_id);
					dxl_set_txpacket_parameter(6, dxl_get_lowbyte(tilt_goal));
					dxl_set_txpacket_parameter(7, dxl_get_highbyte(tilt_goal));
					dxl_set_txpacket_length(10);
					dxl_tx_rx_packet();
				}
				//state++;
				state = 8;
				break;
			/*case 6:
				log(Debug) << "PanTiltController: checking whether pan is moving"<< endlog();
				dxl_read_byte(pan_id, AX_MOVING);
				state++;
				break;
			case 7:
				if (dxl_get_rxpacket_parameter(0) == 1) {
					log(Debug) << "PanTiltController: Pan is moving."<< endlog();
					state++;
				} else  {
					log(Debug) << "PanTiltController: Pan is not moving."<< endlog();
					state = 10;
				}
				break;*/
			case 8:
				log(Debug) << "PanTiltController: Requesting current pan position"<< endlog();				
				dxl_read_word(pan_id, AX_PRESENT_POSITION_L);
				state++;
				break;
			case 9:
				newPosition = dxl_makeword(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));
				log(Debug) << "PanTiltController: received new pan position, " << newPosition << endlog();
				currentPan.data = (newPosition-pan_offset)/RAD_TO_STEP;
				currentPosPanPort.write(currentPan);
				//state++;
				state = 12;
				break;		
			/*case 10:
				log(Debug) << "PanTiltController: checking whether tilt is moving"<< endlog();
				dxl_read_byte(tilt_id, AX_MOVING);
				state++;
				break;
			case 11:
				if (dxl_get_rxpacket_parameter(0) == 1) {
					log(Debug) << "PanTiltController: Tilt is moving."<< endlog();
					state++;
				} else  {
					log(Debug) << "PanTiltController: Tilt is not moving."<< endlog();
					state = 5;
				}
				break;*/
			case 12:
				log(Debug) << "PanTiltController: Requesting current tilt position"<< endlog();				
				dxl_read_word(tilt_id, AX_PRESENT_POSITION_L);
				state++;
				break;
			case 13:
				newPosition = dxl_makeword(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));
				log(Debug) << "PanTiltController: received new tilt position, " << newPosition << endlog();
				currentTilt.data = (newPosition-tilt_offset)/RAD_TO_STEP;
				currentPosTiltPort.write(currentTilt);
				state = 5;
				break;
			default:
				state = 1;
		}
	} else if (commStatus == COMM_RXCORRUPT) {
		log(Debug) << "PanTiltController: status corrupt. Resending the instruction."<< endlog();
		state = pstate;
		commStatus = COMM_RXSUCCESS;
	} else if (commStatus == COMM_RXWAITING) {
		if (trial>TRIAL_MAX) {
			log(Debug) << "PanTiltController: waited too long for status. Resending the instruction."<< endlog();
			state = pstate;
			commStatus = COMM_RXSUCCESS;
		} else {
			dxl_rx_packet();
			trial++;
		}
	}
}

void PanTiltController::dxl_tx_rx_packet(void) {
	dxl_tx_packet();
	if (commStatus == COMM_TXSUCCESS) {
		dxl_rx_packet();
	}
}

void PanTiltController::dxl_tx_packet(void)
{
	commStatus = COMM_TXSUCCESS;

	unsigned char TxNumByte;
	unsigned char checksum = 0;

	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	for(int i=0; i<(gbInstructionPacket[LENGTH]+1); i++) checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	
	log(Debug) << "Sent instruction: ";
	for(int i=0; i<TxNumByte; i++) {
		log(Debug) << (unsigned int) gbInstructionPacket[i] << " ";
	}
	log(Debug) << endlog();
	
	instruction.channels.resize(2);
	instruction.channels[0].datapacket.clear();
	instruction.channels[0].datapacket.resize(TxNumByte);
	instruction.channels[0].datasize = TxNumByte;
	
	for (int i=0; i<TxNumByte; i++) {
		instruction.channels[0].datapacket[i]=gbInstructionPacket[i];
	}
	instructionPort.write(instruction);
}

void PanTiltController::dxl_rx_packet(void) {
	if (gbInstructionPacket[ID] == BROADCAST_ID) {
		log(Debug) << "PanTiltController: broadcast id, no status packet"<< endlog();
		commStatus = COMM_RXSUCCESS;
		return;
	}

	if (!(gbInstructionPacket[INSTRUCTION] == INST_READ)) {
		log(Debug) << "PanTiltController: not read instruction, no status packet"<< endlog();
		commStatus = COMM_RXSUCCESS;
		return;
	}
	
	if (!(statusPort.read(status) == NewData)) {		
		log(Debug) << "PanTiltController: waiting for data"<< endlog();
		commStatus = COMM_RXWAITING;
		return;
	} else {
		log(Debug) << "Received status: ";
		gbStatusSize = status.channels[0].datasize;
		for (int i=0; i<gbStatusSize; i++) {		
			log(Debug) << (unsigned int) status.channels[0].datapacket[i] << " ";
			gbStatusPacket[i] = status.channels[0].datapacket[i];
			
		}
		log(Debug) << endlog();
		
		unsigned char tmpLength = 6;
		if( gbInstructionPacket[INSTRUCTION] == INST_READ )
			tmpLength += gbInstructionPacket[PARAMETER+1];
		
		if ((gbStatusSize >= 6) && (gbStatusPacket[0] == 0xff) && (gbStatusPacket[1] == 0xff) && dxl_check_rxpacket_checksum()) {
			if ((gbStatusPacket[ID] != gbInstructionPacket[ID]) || (gbStatusSize != tmpLength)){
				log(Debug) << "PanTiltController: not my status packet or length not valid. waiting another"<< endlog();
				commStatus = COMM_RXWAITING;
			} else {
				log(Debug) << "PanTiltController: status packet ok"<< endlog();
				printErrorCode();
				commStatus = COMM_RXSUCCESS;
			}
		} else {
			log(Warning) << "Invalid status packet."<< endlog();
			commStatus = COMM_RXCORRUPT;
		}
	}
}

void PanTiltController::dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

void PanTiltController::dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

void PanTiltController::dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

void PanTiltController::dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

int PanTiltController::dxl_get_rxpacket_id(void) {
	return (int)gbStatusPacket[ID];
}

int PanTiltController::dxl_rxpacket_isError(void)
{
	if(gbStatusPacket[ERRBIT])
		return 1;

	return 0;
}

int PanTiltController::dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}

int PanTiltController::dxl_get_rxpacket_length(void)
{
	return (int)gbStatusPacket[LENGTH];
}

bool PanTiltController::dxl_check_rxpacket_checksum(void)
{
	unsigned char checksum = 0;
	
	for(int i=0; i<(gbStatusPacket[LENGTH]+1); i++) 
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;
	return (gbStatusPacket[gbStatusPacket[LENGTH]+3] == checksum);
}

int PanTiltController::dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

int PanTiltController::dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int PanTiltController::dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

int PanTiltController::dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

void PanTiltController::dxl_ping( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_tx_rx_packet();
}

void PanTiltController::dxl_read_byte( int id, int address )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_tx_rx_packet();
}

void PanTiltController::dxl_write_byte( int id, int address, int value )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_tx_rx_packet();
}

void PanTiltController::dxl_read_word( int id, int address )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_tx_rx_packet();
}

void PanTiltController::dxl_write_word( int id, int address, int value )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_tx_rx_packet();
}

void PanTiltController::printErrorCode(void)
{
	int id = dxl_get_rxpacket_id();
	if (dxl_rxpacket_isError()) {
		if (id == pan_id) {
			log(Warning) << "PAN Dynamixel: ";
		} else if (id == tilt_id) {
			log(Warning) << "TILT Dynamixel: ";
		} else {
			log(Warning) << "UNKNOWN Dynamixel id: ";
		}
	}
	if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		log(Warning) << "Input voltage error!" << endlog();

	if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		log(Warning) << "Angle limit error!" << endlog();

	if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		log(Warning) << "Overheat error!" << endlog();	

	if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		log(Warning) << "Out of range error!" << endlog();	

	if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		log(Warning) << "Checksum error!" << endlog();		

	if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		log(Warning) << "Overload error!" << endlog();			

	if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		log(Warning) << "Instruction code error!" << endlog();				
}

ORO_CREATE_COMPONENT(PanTiltController)
