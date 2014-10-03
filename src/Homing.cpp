#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "Homing.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace AMIGOGENERIC;


Homing::Homing(const string& name) : TaskContext(name, PreOperational)
{  
    // Ports
	addPort( "position",pos_inport );
    addPort( "ref_out", ref_outport );
    addPort( "homing_finished", homingfinished_outport );

	// Properties
    addProperty( "vector_size",     N               ).doc("Number of joints");
    addProperty( "bodypart",        bodypart        ).doc("Name of the bodypart, (fill in BODYPARTNAME)");
    addProperty( "prefix",          prefix          ).doc("Prefix of components (for example: SPINDLE or RPERA)");

    addProperty( "homing_type",     homing_type     ).doc("Type of homing choose from: ['endswitch','servoerror', 'absolutesensor', 'forcesensor']");    
    addProperty( "require_homing",  require_homing  ).doc("Vector of boolean values to specify which joints should be homed");
    addProperty( "homing_order",    homing_order    ).doc("The order in which the joints are homed, for example: array [2 3 1]");
    addProperty( "homing_direction",homing_direction).doc("Homing direction");
    addProperty( "homing_velocity", homing_velocity ).doc("Homing velocities");
    addProperty( "homing_stroke",   homing_stroke   ).doc("Stroke from homing point to zero positions (encoders are resetted using this value)");
    addProperty( "homing_midpos",   homing_midpos   ).doc("position that the joint should have during homing. To avoid collisions with other bodies/ itself");
    addProperty( "homing_endpos",   homing_endpos   ).doc("position that the body should go to after homing is finished.");

    addProperty( "homing_forces",   homing_forces    ).doc("Force threshold for force sensor homing");
    addProperty( "homing_errors",   homing_errors   ).doc("Error threshold for endstop homing");
    addProperty( "homing_absPos",   homing_absPos   ).doc("Value of the absolute sensor at qi=0 for absolute sensor homing");
}

Homing::~Homing()
{
}

bool Homing::configureHook()
{
    // Input checks generic
    if (homing_type.size() != N || require_homing.size() != N || homing_order.size() != N  ) {
        log(Error) << prefix <<"_Homing: size of homing_type, require_homing or homing_order does not match vector_size"<<endlog();
        return false;
    }
    if (homing_direction.size() != N || homing_velocity.size() != N || homing_stroke.size() != N || homing_midpos.size() != N || homing_endpos.size() != N  ) {
        log(Error) << prefix <<"_Homing: size of homing_direction, homing_velocity, homing_stroke, homing_midpos or homing_endpos does not match vector_size"<<endlog();
        return false;
    }

    // Check which types of homing are required
    endswitchhoming = false;
    errorhoming     = false;
    absolutehoming  = false;
    forcehoming     = false;
    for (uint j = 0; j<N; j++) {
        if (homing_type[j] == "endswitch") {
            endswitchhoming = true;
        } else if (homing_type[j] == "servoerror") {
            errorhoming = true;
        } else if (homing_type[j] == "absolutesensor") {
            absolutehoming = true;
        } else if (homing_type[j] == "forcesensor") {
            forcehoming = true;
        } else {
            log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose from :['endswitch','servoerror', 'absolutesensor', 'forcesensor']"<<endlog();
            return false;
        }
    }

    // Input checks specific for homing types
    if ( (forcehoming && homing_forces.size() != N) || (errorhoming && homing_errors.size() != N) || (absolutehoming && homing_absPos.size() != N) ) {
        log(Error) << prefix <<"_Homing: size of homing_force, homing_error or homing_absPos does not match vector_size"<<endlog();
        return false;
    }

    // Define homing type specific ports
    if (endswitchhoming) {
        addPort( "endswitch", endswitch_inport );
        // to do make this port inside a vector of ports such that more joints than one can use this type of homing
    }
    if (errorhoming) {
        addPort( "servo_error_in", jointerrors_inport );
    }
    if (absolutehoming) {
        addPort( "abs_pos_in", absPos_inport );
    }
    if (forcehoming) {
        addPort( "force_in", forces_inport );
    }

	return true;
}

bool Homing::startHook()
{
    // Connect to Components
    TaskContext* Supervisor = this->getPeer("Supervisor");
    TaskContext* ReadEncoders = this->getPeer( prefix + "_ReadEncoders");
    TaskContext* ReferenceGenerator = this->getPeer( prefix + "_ReferenceGenerator");

    // Check Connections
    if ( !Supervisor ) {
        log(Error) << "Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }
    if ( !ReadEncoders ) {
        log(Error) << prefix <<"_Homing: Could not find :" << prefix << "_ReadEncoders component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }

    // Fetch Operations
    StartBodyPart = Supervisor->getOperation("StartBodyPart");
    StopBodyPart = Supervisor->getOperation("StopBodyPart");
    ResetEncoder = ReadEncoders->getOperation("reset");
    SetMaxRefGenVel = ReferenceGenerator->getOperation("SetMaxRefGenVel");

    // Check Operations
    if ( !StartBodyPart.ready() ) {
        log(Error) << prefix <<"_Homing: Could not find Supervisor.StartBodyPart Operation!"<<endlog();
        return false;
    }
    if ( !StopBodyPart.ready() ) {
        log(Error) << prefix <<"_Homing: Could not find Supervisor.StopBodyPart Operation!"<<endlog();
        return false;
    }
    if ( !ResetEncoder.ready() ) {
        log(Error) << prefix <<"_Homing: Could not find :" << prefix << "_ReadEncoder.reset Operation!"<<endlog();
        return false;
    }

    // Check homing type specific port connections
    if (endswitchhoming) {
        if (!endswitch_inport.connected()) {
            log(Error) << prefix <<"_Homing: endswitch_inport not connected!"<<endlog();
            return false;
        }
    }
    if (errorhoming) {
        if (!jointerrors_inport.connected()) {
            log(Error) << prefix <<"_Homing: jointerrors_inport not connected!"<<endlog();
            return false;
        }
    }
    if (absolutehoming) {
        if (!absPos_inport.connected()) {
            log(Error) << prefix <<"_Homing: absPos_inport not connected!"<<endlog();
            return false;
        }
    }
    if (forcehoming) {
        if (!forces_inport.connected()) {
            log(Error) << prefix <<"_Homing: forces_inport not connected!"<<endlog();
            return false;
        }
    }

    // Set variables
    jointNr = 0; // note that 0 corresponds with joint q1
    joint_finished = false;
    state = 0;
    firsttime = true;

    position.assign(N,0.0);
    ref_out.resize(3);
    ref_out_prev.resize(3);
    for (uint k = 0; k<N; k++) {
        ref_out[k].assign(N,0.0);
        ref_out_prev[k].assign(N,0.0);
    }
		
	return true;
}

void Homing::updateHook()
{
    if (require_homing[jointNr] == 0.0) {
        jointNr++;
    }

    // Read positions
    pos_inport.read(position);

    if (state == 0) {
        // Update reference
        updateHomingRef(jointNr);

        // Check homing criterion
        joint_finished = evaluateHomingCriterion(jointNr);
        if (joint_finished) {
            // Reset encoders and send joint to midpos
            StopBodyPart(bodypart);
            ResetEncoder(jointNr,homing_stroke[jointNr]);
            StartBodyPart(bodypart);
            updateMidposRef(jointNr);
            state++;
        }
    }
    if (state == 1) {
        if ( (position[jointNr] > (homing_midpos[jointNr]*0.999) ) && (position[jointNr] < (homing_midpos[jointNr]*1.001) ) ) {
            jointNr++;
            firsttime = true;
            state = 0;
            // Check if last joint is finished
            if(jointNr==N) {
                log(Warning) << "Homing: Finished " << bodypart << "homing."<<endlog();
                homingfinished_outport.write(true);
            }
        }
    }
}

void Homing::updateHomingRef( uint jointNr)
{
    if (homing_type[jointNr] != "absolutesensor" ) {
        if (firsttime) {
            // generate ref_out
            for (uint j = 0; j<N; j++) {
                ref_out[j][0] = position[j];
                ref_out[j][1] = homing_velocity[j];
                ref_out[j][2] = 0.0; // Use default
            }
            ref_out[jointNr][0] = homing_direction[jointNr]*25.0;

            // write ref
            ref_outport.write(ref_out);
            firsttime = false;
        }
    } else { // absolute sensor homing

        // read absolute sensor output
        ints absolutesensoroutput;
        absPos_inport.read(absolutesensoroutput);

        // determine direction
        double direction = 1.0; // positive unless goal - measured < 0
        if ( (homing_absPos[jointNr]-absolutesensoroutput[jointNr]) < 0 ) {
            direction = -1.0;
        }

        // determine velocityfactor
        double velocityfactor = 1.0;
        if ( fabs(homing_absPos[jointNr]-absolutesensoroutput[jointNr]) < 20 ) {
            velocityfactor = 0.2;
        }

        // generate ref_out
        for (uint j = 0; j<N; j++) {
            ref_out[j][0] = position[j];
            ref_out[j][1] = homing_velocity[j]*velocityfactor;
            ref_out[j][2] = 0.0; // Use default
        }
        ref_out[jointNr][0] = direction*25.0;

        // Only write ref if a new ref has been generated
        if (ref_out_prev != ref_out) {
            ref_out_prev = ref_out;
            ref_outport.write(ref_out);
            log(Warning) << "Homing: sent new ref for analog homing joint " << jointNr << "."<<endlog();
        }
    }

    return;
}

void Homing::updateMidposRef( uint jointNr)
{
    // generate ref_out
    for (uint j = 0; j<N; j++) {
        ref_out[j][0] = position[j];
        ref_out[j][1] = 0.0; // Use default
        ref_out[j][2] = 0.0; // Use default
    }
    ref_out[jointNr][0] = homing_midpos[jointNr];

    // write ref_out
    ref_outport.write(ref_out);

    log(Warning) << "Homing: sent joint " << jointNr << " to midpos."<<endlog();

    return;
}

bool Homing::evaluateHomingCriterion( uint jointNr)
{
    bool result = false;

    if (homing_type[jointNr] == "endswitch" ) {
        std_msgs::Bool endswitch_msg;
        endswitch_inport.read(endswitch_msg);
        result = endswitch_msg.data;
    } else if (homing_type[jointNr] == "servoerror" ) {
        doubles jointerrors;
        jointerrors_inport.read(jointerrors);
        if (jointerrors[jointNr] > homing_errors[jointNr]) {
            result = true;
        }
    } else if (homing_type[jointNr] == "absolutesensor" ) {
        ints absolutesensoroutput;
        absPos_inport.read(absolutesensoroutput);
        if (absolutesensoroutput[jointNr] == homing_absPos[jointNr]) {
            result = true;
        }
    } else if (homing_type[jointNr] == "forcesensor" ) {
        doubles forces;
        forces_inport.read(forces);
        if (forces[jointNr] > homing_forces[jointNr]) {
            result = true;
        }
    } else {
        log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose from :['endswitch','servoerror', 'absolutesensor', 'forcesensor']"<<endlog();
        return false;
    }

    if (result == true ) {
        log(Warning) << "Homing: Homing Position reached for joint " << jointNr << "."<<endlog();
    }

    return result;
}

ORO_CREATE_COMPONENT(AMIGOGENERIC::Homing)
