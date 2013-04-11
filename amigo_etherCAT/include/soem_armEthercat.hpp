/***************************************************************************
 tag: Sava Marinkov, Ruud van den Bogaert,  Fri Mar 23 12:44:00 CET 2011  soem_armEthercat.h

 soem_armEthercat.h -  dedicated ethercat module TU/e
 -------------------
 begin                : Fri November 23 2012
 copyright            : (C) 2012 Sava Marinkov & Ruud van den Bogaert
 email                : s.marinkov@student.tue.nl , r.v.d.bogaert@tue.nl

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef SOEM_ARMETHERCAT_H
#define SOEM_ARMETHERCAT_H

#include <soem_master/soem_driver_factory.h>
#include <soem_master/soem_driver.h>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <vector>
#include <math.h>
#include <iostream>
#include "COE_config.h"
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <std_msgs/Bool.h>

using namespace std;
typedef vector<double> doubles;

typedef struct PACKED {
		uint16 status_register; 		// General system status register
		uint16  encoder_angle_1;  		// Actual position of motor encoder 1
		uint16  encoder_angle_2;  		// Actual position of motor encoder 2
		uint16  encoder_angle_3;  		// Actual position of motor encoder 3
		uint8  spare_digital_in; 		// Spare I/O connector digital inputs
		float force_1; 					// Analog ADC value of force sensor input 1
		float position_1; 				// Analog ADC value of position sensor 1
		float force_2; 					// Analog ADC value of force sensor input 2
		float position_2; 				// Analog ADC value of position sensor 2
		float force_3; 					// Analog ADC value of force sensor input 3
		float position_3; 				// Analog ADC value of position sensor 3
		float spare_analog_in_1; 		// Analog ADC Value for AD spare input 1
		float spare_analog_in_2; 		// Analog ADC Value for AD spare input 2
		float supply_int_5V; 			// Analog ADC value of 5V supply voltage for ADC
		float supply_int_12V; 			// Analog ADC value of 12V supply voltage for H bridges
		float supply_ext_24V; 			// Analog ADC value of 24V main supply voltage
		float supply_int_1V2; 			// Analog ADC value of 1.2V supply voltage for FPGA
		float supply_int_1V65;	 		// Analog ADC value of 1.65 supply ADC vref voltage
		uint32 message_index; 			// Message index counter
	} in_armEthercatMemoryt;


	typedef struct PACKED {
			uint16 pwm_duty_motor_1; 	// PWM duty cycle for motor 1
			uint16 pwm_duty_motor_2; 	// PWM duty cycle for motor 2
			uint16 pwm_duty_motor_3; 	// PWM duty cycle for motor 3
			uint8  spare_digital_out; 	// Spare I/O connector digital inputs
			uint8  heart_beat;			// Heart beat 
	} out_armEthercatMemoryt;

	using namespace RTT;

	namespace soem_beckhoff_drivers {

	class SoemARMETHERCAT: public soem_master::SoemDriver {
	public:
		SoemARMETHERCAT(ec_slavet* mem_loc);
		~SoemARMETHERCAT() {};

		void update();
		bool configure();
        void write_pwm(float val1,float val2,float val3);
        void read_encoders();
        void read_supply();
        void read_forces();
        void read_positions();
        void read_spareanalog();
        void stop();

	private:
        int printEnabled;
        int printDisabled;
        uint16 cntr;
        uint16 cntr2;
        uint16 cntr3;
        bool enable;
        bool enablestatus;
        bool setOutputToZero;
        uint8 heart_beat_source;
        uint16 initRelEnc1;
        uint16 initRelEnc2;
        uint16 initRelEnc3;
        uint16 encoderAngle0;
		uint16 encoderAngle1;
		uint16 encoderAngle2;
	  	std::vector<float> forceSensors;
	  	std::vector<float> positionSensors;
	  	std::vector<float> spareAnalogIns;     					// ToDo: Why call these spare???
	  	std::vector<float> motorCurrents;
	    std::vector<float> supplyVoltages;
		std::vector<float> pwmDutyMotors;		
		
		EncoderMsg encoderAngle0_msg;
		EncoderMsg encoderAngle1_msg;
		EncoderMsg encoderAngle2_msg;
	  	AnalogMsg forceSensors_msg;
	  	AnalogMsg positionSensors_msg;
	  	AnalogMsg spareAnalogIns_msg;
	  	AnalogMsg motorCurrents_msg;
	 	AnalogMsg supplyVoltages_msg;
		AnalogMsg pwmDutyMotors_msg;
        std_msgs::Bool enable_msg;
		
        in_armEthercatMemoryt* m_in_armEthercat;
        out_armEthercatMemoryt* m_out_armEthercat;

		OutputPort<EncoderMsg> port_out_encoderAngle0;
		OutputPort<EncoderMsg> port_out_encoderAngle1;		
		OutputPort<EncoderMsg> port_out_encoderAngle2;
	 	OutputPort<AnalogMsg> port_out_forceSensors;
	 	OutputPort<AnalogMsg> port_out_positionSensors;
	 	OutputPort<AnalogMsg> port_out_spareAnalogIns;
	 	OutputPort<AnalogMsg> port_out_motorCurrents;
	 	OutputPort<AnalogMsg> port_out_supplyVoltages;
        // OutputPort<std::vector<Bool> >  port_out_spareDigitalIns;

		// ToDo: Use messages similar to soem_beckhoff_drivers? 
		// This basically holds for every port		
		OutputPort	< std::vector<float> > 		port_out_pwmDutyMotors;   
        InputPort	< AnalogMsg>  				port_in_pwmDutyMotors;
        InputPort   < bool>                     port_in_enable;
        InputPort   < bool>                     port_in_reNullPort;
	 	//InputPort	< std::vector<bool> >  		port_in_spareDigitalOuts;  
        //OutputPort	< std::vector<bool> >  	port_out_spareDigitalOuts;

    };
	}
#endif
