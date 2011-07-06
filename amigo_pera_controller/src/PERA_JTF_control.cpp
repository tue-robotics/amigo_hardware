/*! 
 * \author Sava Marinkov
 * \date May, 2011
 * \version 1.0 
 */
#include "PERA_JTF_control.hpp"

using namespace RTT;
using namespace PERA;

PERA_JTF_control::PERA_JTF_control(const std::string& name): TaskContext(name, PreOperational) {
	/// Inports
	addPort("joint_position_measured", 		jointPosMeasuredInPort);
	addPort("joint_velocity_measured", 		jointVelMeasuredInPort);
	addPort("joint_acceleration_measured", 	jointAccMeasuredInPort);
	addPort("joint_position_reference", 	jointPosReferenceInPort);
	addPort("joint_velocity_reference", 	jointVelReferenceInPort);
	addPort("joint_acceleration_reference", jointAccReferenceInPort);
	addPort("joint_torque_measured", 		jointTorqueMeasuredInPort);
			
	/// Outports
	addPort("joint_torque_control_output", 	jointTorqueControlOutPort);
	addPort("parameter_vector", 			parameterVectorOutPort);

	/// Properties
	addProperty( "motor_constants", mc);
	addProperty( "joint_inertia", J);
	addProperty( "coulomb_friction", Fc);			
	addProperty( "viscous_friction", Fv);		
	addProperty( "torque_gain", k);
	addProperty( "torque_offset", g);
	addProperty( "Lambda_gain", xLambda);
	addProperty( "L_gain", xL);
	addProperty( "Gamma_gain", xGamma);
		
	mc.resize(NUM_OF_JOINTS,0.0);
	J.resize(NUM_OF_JOINTS,0.0);
	Fc.resize(NUM_OF_JOINTS,0.0);
	Fv.resize(NUM_OF_JOINTS,0.0);
	k.resize(NUM_OF_JOINTS,0.0);
	g.resize(NUM_OF_JOINTS,0.0);

	m_pos.resize(NUM_OF_JOINTS,0.0);
	m_vel.resize(NUM_OF_JOINTS,0.0);
	m_acc.resize(NUM_OF_JOINTS,0.0);
	r_pos.resize(NUM_OF_JOINTS,0.0);
	r_vel.resize(NUM_OF_JOINTS,0.0);
	r_acc.resize(NUM_OF_JOINTS,0.0);
	m_tor.resize(NUM_OF_JOINTS,0.0);		
	jointControlTorque.resize(NUM_OF_JOINTS,0.0);
	parameterVector.resize(NUM_OF_JOINTS*PARAMS_PER_JOINT,0.0);
		
	q.setZero(NUM_OF_JOINTS,1);
	qd.setZero(NUM_OF_JOINTS,1);
	qdd.setZero(NUM_OF_JOINTS,1);

	r.setZero(NUM_OF_JOINTS,1);
	rd.setZero(NUM_OF_JOINTS,1);
	rdd.setZero(NUM_OF_JOINTS,1);

	theta.setZero(NUM_OF_JOINTS*PARAMS_PER_JOINT,1);
	Y.setZero(NUM_OF_JOINTS,NUM_OF_JOINTS*PARAMS_PER_JOINT);
	
	v.setZero(NUM_OF_JOINTS,1);
	vd.setZero(NUM_OF_JOINTS,1);
	s.setZero(NUM_OF_JOINTS,1);
	
	Lambda.setZero(NUM_OF_JOINTS,NUM_OF_JOINTS);
	L.setZero(NUM_OF_JOINTS,NUM_OF_JOINTS);
	Gamma.setZero(NUM_OF_JOINTS*PARAMS_PER_JOINT,NUM_OF_JOINTS*PARAMS_PER_JOINT);
	
	tm.setZero(NUM_OF_JOINTS,1);
}

PERA_JTF_control::~PERA_JTF_control(){}

bool PERA_JTF_control::configureHook(){
	return true;
}

bool PERA_JTF_control::startHook(){
	for (int i=0; i<NUM_OF_JOINTS*PARAMS_PER_JOINT; i++) {
		Gamma(i,i) = xGamma[i];
	}

	for (int i=0; i<NUM_OF_JOINTS; i++) {
		Lambda(i,i) = xLambda[i];
		L(i,i) = xL[i];
		theta(PARAMS_PER_JOINT*i) = J[i];
		theta(PARAMS_PER_JOINT*i+1) = Fc[i];
		theta(PARAMS_PER_JOINT*i+2) = Fv[i];
		theta(PARAMS_PER_JOINT*i+3) = k[i];
		theta(PARAMS_PER_JOINT*i+4) = g[i];
	}
	//log(Info)<<"Gamma "<< Gamma << endlog();
	//log(Info)<<"Lambda "<< Lambda << endlog();
	//log(Info)<<"L "<< L << endlog();
	//log(Info)<<"theta "<< theta << endlog();
	old_time = os::TimeService::Instance()->getNSecs()*1e-9;
	return true;
}

void PERA_JTF_control::updateHook(){
	/// Read measured joint position, velocity and acceleration
	jointPosMeasuredInPort.read(m_pos);
	jointVelMeasuredInPort.read(m_vel);
	jointAccMeasuredInPort.read(m_acc);
	
	/// Read reference joint position, velocity and acceleration
	jointPosReferenceInPort.read(r_pos);
	jointVelReferenceInPort.read(r_vel);
	jointAccReferenceInPort.read(r_acc);
	
	/// Read measured joint torques
	jointTorqueMeasuredInPort.read(m_tor);
	
	/// Fill in the vectors with the current data
	for (int i=0; i<NUM_OF_JOINTS; i++) {
		q(i) = m_pos[i];
		qd(i) = m_vel[i];
		qdd(i) = m_acc[i];
		r(i) = r_pos[i];
		rd(i) = r_vel[i];
		rdd(i) = r_acc[i];
	}
	
	/// Calculate v, vd and s
	v = rd - Lambda*(q-r);
	vd = rdd - Lambda*(qd-rd);
	s = qd - v;
	
	/// Fill in the regressor matrix
	for (int i=0; i<NUM_OF_JOINTS; i++) {
		y.setZero(1,PARAMS_PER_JOINT);
		y << vd(i), tanh(v(i)), v(i), m_tor[i], 1.0;
		Y.block(i,PARAMS_PER_JOINT*i,1,PARAMS_PER_JOINT) = y;
	}

	/// Update the parameter vector
	double dt = determineDt();
	theta = theta - Gamma*((Y.transpose())*s)*dt;
	//log(Info) << "theta = " << (theta.segment(pos,n)).transpose() <<endlog();
	
	/// Calculate the control effort (motor torques)
	tm = Y*theta - L*s;	
	
	//log(Info) << "ff = " << (Y*theta).transpose() <<endlog();
	//log(Info) << "fb = " << (-L*s).transpose() <<endlog();

	for (int i=0; i<NUM_OF_JOINTS; i++) {
		jointControlTorque[i] = tm(i)*mc[i];
	}

	for (int i=0; i<NUM_OF_JOINTS*PARAMS_PER_JOINT; i++) {
		parameterVector[i] = theta(i);
	}
		
	/// Write the calculated motor torques (actually motor currents) and the current parameter vector to the outports
	parameterVectorOutPort.write(parameterVector);
	jointTorqueControlOutPort.write(jointControlTorque);
}

void PERA_JTF_control::stopHook(){
}

double PERA_JTF_control::determineDt(){
	long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
	double dt = (double)(new_time - old_time);
	old_time = new_time;
	return dt;
}

ORO_CREATE_COMPONENT(PERA::PERA_JTF_control)

