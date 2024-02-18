/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef EXTURDER_MODULE_H
#define EXTRUDER_MODULE_H

#include "Tool.h"
#include "Pin.h"
#include <vector>

class StepperMotor;
class Block;
class Gcode;

//--- PositionMotorController -----------------------------------------------------------------------------------------

// TODO: if this seems to be useful for other purposes, put it into seperat file or into the stepper motor files

/**
 * allows to control a stepper motor by setting its target position
 * the class automatically handles motor start and stops as well as direction changes 
 * it uses a closed loop PD-controller to control the motor acceleration
 */
 class PositionMotorController {
    public:
        PositionMotorController();
                
        // initialize the controller
        void init(StepperMotor* motor);
        // sets the closed loop controller gains
        void set_gains(float Kp, float Kd);
        // gets the closed loop controller gains
        void get_gains(float& Kp, float& Kd);
        // set the target position of the motor
        void set_target_position(float target_position);
        // sets the maximum velocity
        void set_max_velocity(float max_velocity);
        // sets the maximum accelertion
        void set_max_acceleration(float max_acceleration);
        // get current velocity in steps/sec
        float get_target_position();
        // returns the current motor position in steps relative to the last call of reset_position()
        int get_position();
        // returns the current motor velocity in steps/s
        float get_velocity();
        // resets the position
        void reset_position(int value = 0);
        // returns the stepper motor
        StepperMotor* get_motor();
        // updates the motor velocity, call this periodically 
        void update(float dt);
        
    private:
        void set_velocity(float velocity);
        
        int position;               // current position in steps when the last motor->move() command was issued - use get_position() for real position
        float velocity;				// current motor velocity in steps/s
        float target_position;      // in steps
        float last_error;			// in steps
        StepperMotor* motor;        // the stepper motor under control
        
        float min_velocity;         // minimum velocity of the motor in steps/s
        float max_velocity;         // maximum velocity of the motor in steps/s
        float max_acceleration;		// max acceleration in steps/s²
        float Kp;					// proportional gain
        float Kd;					// differential gain
};

//---------------------------------------------------------------------------------------------------------------------

class ExtruderAdvanceCompensation {
    public:
        ExtruderAdvanceCompensation();
        
        void init(StepperMotor* motor, float max_speed_steps_per_s, float accelleration_ticks_dt);
        void on_gcode_received(Gcode* gcode, uint16_t identifier);
        void on_config_reload(uint16_t config_identifier, float acceleration, float max_speed, float steps_per_millimeter);
        void on_acceleration_tick(float current_position, float travel_distance, float travel_ratio, float steps_per_millimeter, Block* current_block);
        PositionMotorController* get_motor_controller();
        
        
    private:
		float accelleration_ticks_dt;
		float extruder_advance_factor;                              // extruder advance factor aka K
		float max_advance_distance;									// maximum advance distance in mm
        float retract_feedrate_threshold;                            // advance will be deactivated for extruder motions above this threshold (mm/s)
		PositionMotorController motor_controller;					// allows to control the motor by its target position
                                                                    // with a filament force sensor we could also build a closed loop extrusion controller with this
                                                                    // it would be more accurate than our current approximation models of the extruder 
};

//---------------------------------------------------------------------------------------------------------------------

// also known as flux compensator
class ExtruderSlipCompensation {
	public:
		ExtruderSlipCompensation();
		
		void on_config_reload(uint16_t config_identifier);
		float compute_slip_compensation(float extrusion_distance, float extrusion_velocity);
    
	private:
        bool compute_slip_compenstaion_parameter(std::vector<float>& multiplier_table, float parameter[3]);
		
        float slip_compenstaion_parameter[3];                       // parameter for slip(velocity) function 
};

//---------------------------------------------------------------------------------------------------------------------

// NOTE Tool is also a module, no need for multiple inheritance here
class Extruder : public Tool {
    public:
        Extruder(uint16_t config_identifier, bool single= false);
        virtual ~Extruder();

        void     on_module_loaded();
        void     on_config_reload(void* argument);
        void     on_gcode_received(void*);
        void     on_gcode_execute(void* argument);
        void     on_block_begin(void* argument);
        void     on_block_end(void* argument);
        void     on_play(void* argument);
        void     on_pause(void* argument);
        void     on_halt(void* argument);
        void     on_speed_change(void* argument);
        void     acceleration_tick(void);
        uint32_t stepper_motor_finished_move(uint32_t dummy);
        Block*   append_empty_block();

    private:
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        uint32_t rate_increase() const;

        StepperMotor*  stepper_motor;
        Pin            step_pin;                     // Step pin for the stepper driver
        Pin            dir_pin;                      // Dir pin for the stepper driver
        Pin            en_pin;

        float          target_position;              // End point ( in mm ) for the current move/gcode
        float          current_position;             // current extruder position ( in mm ), updated at the end of each block 
                                                     // when filament slip compensation is enabled this value is not comparable to target_position
        float          unstepped_distance;           // overflow buffer for requested moves that are less than 1 step
        Block*         current_block;                // Current block we are stepping, same as Stepper's one

        // kept together so they can be passed as public data
        struct {
            float steps_per_millimeter;         // Steps to travel one millimeter
            float filament_diameter;            // filament diameter
            float extruder_multiplier;          // flow rate 1.0 == 100%
            float acceleration;                 // extruder accleration SOLO setting
            float retract_length;               // firmware retract length
        };

        float saved_current_position;
        float volumetric_multiplier;
        float feed_rate;               // mm/sec

        float travel_ratio;
        float travel_distance;

        // for firmware retract
        float retract_feedrate;
        float retract_recover_feedrate;
        float retract_recover_length;
        float retract_zlift_length;
        float retract_zlift_feedrate;

        struct {
            char mode:3;        // extruder motion mode,  OFF, SOLO, or FOLLOW
            bool absolute_mode:1; // absolute/relative coordinate mode switch
            bool saved_absolute_mode:1;
            bool paused:1;
            bool single_config:1;
            bool retracted:1;
            bool cancel_zlift_restore:1; // hack to stop a G11 zlift restore from overring an absolute Z setting
        };

        //--- extruder compensation -----------------------------------------------------------------------------------
		
        ExtruderAdvanceCompensation* extruder_advance;
		ExtruderSlipCompensation* extruder_slip_compensation;
};

//*** FUNCTION ********************************************************************************************************

// computes the parameter of a best fit quadratic curve through the data points. The curve is defined by y = ax²+bx+c
bool find_quadratic_factors(float* data_x, float* data_y, unsigned int data_point_count, float &a, float &b, float &c);

#endif
