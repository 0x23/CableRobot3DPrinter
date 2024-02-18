/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

//*** INCLUDE *************************************************************************************

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Config.h"
#include "libs/StreamOutput.h"
#include "libs/StepperMotor.h"

#include <math.h> 
#include "Vector3.h"
#include "Gcode.h"
#include "Robot.h"
#include "Stepper.h"
#include "Touchprobe.h"
#include "BaseSolution.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "nuts_bolts.h"
#include "Conveyor.h"
#include "SerialMessage.h"
#include "StreamOutputPool.h"

//*** DEFINE **************************************************************************************

#define touchprobe_enable_checksum           CHECKSUM("touchprobe_enable")
#define touchprobe_log_enable_checksum       CHECKSUM("touchprobe_log_enable")
#define touchprobe_logfile_name_checksum     CHECKSUM("touchprobe_logfile_name")
#define touchprobe_log_rotate_mcode_checksum CHECKSUM("touchprobe_log_rotate_mcode")
#define touchprobe_pin_checksum              CHECKSUM("touchprobe_pin")
#define touchprobe_debounce_count_checksum   CHECKSUM("touchprobe_debounce_count")

//*** CLASS ***************************************************************************************

Touchprobe::Touchprobe() {
}

void Touchprobe::on_module_loaded() {
    // if the module is disabled -> do nothing
    this->enabled = THEKERNEL->config->value( touchprobe_enable_checksum )->by_default(false)->as_bool();
    if( !(this->enabled) ){
        // as this module is not needed free up the resource
        delete this;
        return;
    }
    this->ProbeSpeed = 100;
    // load settings
    Touchprobe::on_config_reload(this);
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_IDLE);
}

void Touchprobe::on_config_reload(void* argument){
    pin.from_string(  THEKERNEL->config->value(touchprobe_pin_checksum)->by_default("nc" )->as_string())->as_input();
    debounce_count =  THEKERNEL->config->value(touchprobe_debounce_count_checksum)->by_default(100)->as_number();

    steppers[0] = THEKERNEL->robot->actuators[0];
    steppers[1] = THEKERNEL->robot->actuators[1];
    steppers[2] = THEKERNEL->robot->actuators[2];

    this->should_log = enabled = THEKERNEL->config->value( touchprobe_log_enable_checksum )->by_default(false)->as_bool();
    if( should_log){
        filename = THEKERNEL->config->value(touchprobe_logfile_name_checksum)->by_default("/sd/probe_log.csv")->as_string();
        FlushLogfileMCode = THEKERNEL->config->value(touchprobe_log_rotate_mcode_checksum)->by_default(0)->as_int();
        logfile = NULL;
    }
}

/**
 * performs probing. After probing 'returnProbe' should be called to move to initial position. Alternatively axis position can be reset at current position (similar to homing).
 * @param StepsMoved                steps moved from startpoint to probe hit (or target position when probe hit did not occure)
 * @param AbsoluteMotorPositions    contains the absoluthe axis positions after probing
 * @param RelativeTargetPosition    target cartesian position relative to start position
 * @param ProbeSpeed                Probing speed in mm/s, since probing does'nt perform acceleration/decelaration the speed should be sufficiently low
 */
bool Touchprobe::doProbing(int StepsMoved[3], float AbsoluteMotorPositions[3], float RelativeTargetPosition[3], float ProbeSpeed, bool ReturnToStart) {
    // Enable the motors and set initial speed
    THEKERNEL->stepper->turn_enable_pins_on();
   
   float LastActuatorPos[3];
    for(int j=0; j<3; j++)
        StepsMoved[j] = 0;
    
    // get current position
    float InitialPosition[3];
    float InitialActuatorPosition[3];
    THEKERNEL->robot->get_axis_position(InitialPosition);
    THEKERNEL->robot->arm_solution->cartesian_to_actuator(InitialPosition, InitialActuatorPosition);
	for(int j=0; j<3; j++)
        LastActuatorPos[j] = InitialActuatorPosition[j];
          
    // prepare motion
    float LineSegmentLength = 1.0f; // in mm
    Vector3 RelativeTargetPos(RelativeTargetPosition[0], RelativeTargetPosition[1], RelativeTargetPosition[2]);
    Vector3 LineSegmentDelta = RelativeTargetPos.normalized()*LineSegmentLength;
    int Iterations = int(RelativeTargetPos.mag()/LineSegmentLength);
	if(Iterations == 0) Iterations = 1;
	
	float SpeedMultiplier = 1.0f;
	int ProbeHitCount = 0;
		
    // move probe, use many small line segments
    for(int i=1; i<=Iterations; i++) {
        Vector3 NextPosition;
        
        // compute next target position in cartesian coordinates
        for(int j=X_AXIS; j<=Z_AXIS; j++)
            NextPosition[j] = (i<Iterations) ? (InitialPosition[j] + float(i)*LineSegmentDelta[j]) : (InitialPosition[j]+RelativeTargetPosition[j]);
        
        // compute actuator position for next step
        float ActuatorPos[3];
        THEKERNEL->robot->arm_solution->cartesian_to_actuator(NextPosition.elem, ActuatorPos);
        
        // compute velocities
        Vector3 Velocities(ActuatorPos[0]-LastActuatorPos[0], ActuatorPos[1]-LastActuatorPos[1], ActuatorPos[2]-LastActuatorPos[2]); 
		Velocities = Velocities.normalized()*ProbeSpeed;

        // move steppers
		for(int j=X_AXIS; j<=Z_AXIS; j++) {
            float StepsPerMM = steppers[j]->get_steps_per_mm();
            int Steps = int((ActuatorPos[j]-InitialActuatorPosition[j])*StepsPerMM) - StepsMoved[j];
            steppers[j]->move(Steps<0, abs(Steps), abs(int(Velocities[j]*StepsPerMM*SpeedMultiplier)));
        }
		
		// call the idle method and hope it doesn't take that long
        // maybe this can be removed for the sake of more accurate results
        //THEKERNEL->call_event(ON_IDLE);
            
        // wait for probe hit or motor stop and increase probe hit counter if necessary
        unsigned int DeltaSteps[3];
        ProbeHitCount += (waitForProbe(DeltaSteps) == true) ? 1 : 0;
	
        // update step counter
        for(int j=X_AXIS; j<=Z_AXIS; j++) {
            StepsMoved[j] += Velocities[j] > 0 ? DeltaSteps[j] : -DeltaSteps[j];
            LastActuatorPos[j] = ActuatorPos[j];
        }
   
        // stop if probe was triggered
        if(ProbeHitCount == 1)
            break;
    }
    
    // compute absolute motor positions
    for(int j=X_AXIS; j<=Z_AXIS; j++)
        AbsoluteMotorPositions[j] = InitialActuatorPosition[j] + StepsMoved[j]/steppers[j]->get_steps_per_mm();
    
    // return to start position
    if(ReturnToStart)
        returnProbe(StepsMoved, ProbeSpeed);
    
    return ProbeHitCount>0;
}


/**
 * moves the probe to its initial position
 * @param StepsMoved      steps moved, relative to initial position
 * @param MotorFeedrate   motor feedrate in mm/s
 */
bool Touchprobe::returnProbe(int StepsMoved[3], float MotorFeedrate) {	
    // move motors
    for(int i=X_AXIS; i<=Z_AXIS; i++) {
        steppers[i]->move(StepsMoved[i]>0, abs(StepsMoved[i]), MotorFeedrate*steppers[i]->get_steps_per_mm());
    }
	   
    // wait until stepper reached position
    while(steppers[X_AXIS]->is_moving() || steppers[Y_AXIS]->is_moving() || steppers[Z_AXIS]->is_moving())
        THEKERNEL->call_event(ON_IDLE);
	
    return true;
}

/**
 * wait for probe hit and report moved steps.
 * @param StepsMoved    number of steps moved during this method
 * @return true when probe was triggered, false when motors stopped before probe hit
 */
bool Touchprobe::waitForProbe(unsigned int StepsMoved[3]) {
    bool ProbeTriggered = false;
    unsigned int debounce = 0;
    
    // wait until all steppers stopped
    while(steppers[X_AXIS]->is_moving() || steppers[Y_AXIS]->is_moving() || steppers[Z_AXIS]->is_moving()) {
        // if the touchprobe is active...
        if( pin.get() ) {
            //...increase debounce counter...
            if( debounce < debounce_count) {
                // ...but only if the counter hasn't reached the max. value
                debounce++;
            } else {
                ProbeTriggered = true;
                break;
            }
        } else {
            // The probe was not hit yet, reset debounce counter
            debounce = 0;
        }	
    }
    
    // read out steps
    for(int i=X_AXIS; i<=Z_AXIS; i++)
        StepsMoved[i] = steppers[i]->get_stepped();

	// stop the steppers
    for(int i=X_AXIS; i<=Z_AXIS; i++) {
		if(this->steppers[i]->is_moving())
			steppers[i]->move(false, 0, 0);
    }
				
    return ProbeTriggered;
}

/**
 * moves the endeffector to the given position by issueing gcodes
 * it is not recommended to call this method when the probe did not return to its startposition
 * This happens when doProbing was called with the 'ReturnToStart' parameter set to false false and
 * 'returnProbe' was not called.
 * @param P         position, NAN entries are ignored and the respective axis is not moved
 * @param Feedrate  Feddrate of move in mm/s
 * @param Relative  when true the given coordinates are interpreted as relative movement
 */
void Touchprobe::moveProbe(float x, float y, float z, float Feedrate, bool Relative) {
    char buf[32];
    char cmd[64];

    strcpy(cmd, Relative ? "G91 G0 " : "G0");

    if(!isnan(x)) {
        int n = snprintf(buf, sizeof(buf), " X%1.3f", x);
        strncat(cmd, buf, n);
    }
    if(!isnan(y)) {
        int n = snprintf(buf, sizeof(buf), " Y%1.3f", y);
        strncat(cmd, buf, n);
    }
    if(!isnan(z)) {
        int n = snprintf(buf, sizeof(buf), " Z%1.3f", z);
        strncat(cmd, buf, n);
    }

    // use specified feedrate (mm/sec)
    int n = snprintf(buf, sizeof(buf), " F%1.1f", Feedrate*60);
    strncat(cmd, buf, n);
    
    // disable relative mode
    if(Relative)
        strcat(cmd, " G90");
		
    // send as a command line as may have multiple G codes in it
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    THEKERNEL->conveyor->wait_for_empty_queue();
}

/**
 * handleincoming g-codes
 */
void Touchprobe::on_gcode_received(void* argument) {
    Gcode* gcode = static_cast<Gcode*>(argument);
    Robot* robot = THEKERNEL->robot;

    if( gcode->has_g) {
        if( gcode->g == 31 ) {
            float RelativePosition[3];
            float InitialPosition[3];
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            // fetch relative position from G-code
            robot->get_axis_position(InitialPosition);
            for(char c = 'X'; c <= 'Z'; c++)
                RelativePosition[c-'X'] = gcode->has_letter(c) ? robot->to_millimeters(gcode->get_value(c)) - ( robot->absolute_mode ? InitialPosition[c-'X'] : 0 ) : 0;
            
            // fetch probing speed from G-Code
            if(gcode->has_letter('F'))
                ProbeSpeed = robot->to_millimeters( gcode->get_value('F') ) / robot->get_seconds_per_minute();
                
            // do probing
            int StepsMoved[3];
            float AbsoluteMotorPositions[3];
            bool ProbeTriggered = doProbing(StepsMoved, AbsoluteMotorPositions, RelativePosition, ProbeSpeed, false);

            //compute axis position at current motor position using forward kinematic
            float WorldPos[3];
            robot->arm_solution->actuator_to_cartesian(AbsoluteMotorPositions, WorldPos);
            for(int i = X_AXIS; i<=Z_AXIS; i++)
                robot->reset_axis_position(WorldPos[i], i);
            
            // output result
            if(ProbeTriggered) {
                gcode->stream->printf("Probehit: %1.3f %1.3f %1.3f\n", robot->from_millimeters(WorldPos[0]), robot->from_millimeters(WorldPos[1]), robot->from_millimeters(WorldPos[2]));
            }
            
            // log position
            if( this->should_log && ProbeTriggered ){
                fprintf(logfile, "%1.3f %1.3f %1.3f\n", robot->from_millimeters(WorldPos[0]), robot->from_millimeters(WorldPos[1]), robot->from_millimeters(WorldPos[2]) );
                flush_log();
            }
        }
    } else if(gcode->has_m) {
		if( gcode->m == 119 ) {
			THEKERNEL->streams->printf("Probe switch state: %i\n", (int)pin.get());
		}
		
        // log rotation
        // for now this only writes a separator
        // TODO do a actual log rotation
        if( FlushLogfileMCode != 0 && should_log && gcode->m == FlushLogfileMCode){
            string name;
            fputs("--\n",logfile);
            flush_log();
        }
    }
}

void Touchprobe::flush_log(){
    //FIXME *sigh* fflush doesn't work as expected, see: http://mbed.org/forum/mbed/topic/3234/ or http://mbed.org/search/?type=&q=fflush
    //fflush(logfile);
 //   fclose(logfile);
    //can't reopen the file here -> crash
    logfile = NULL;
}
// Workaround for the close<->reopen crash, which itself is a workaround for wrong (or unimplemented) fflush behaviour
void Touchprobe::on_idle(void* argument){
    if( logfile == NULL) {
        // NOTE: File creation is buggy, a file may appear but writing to it will fail
 //       logfile = fopen( filename.c_str(), "a");
    }
}

