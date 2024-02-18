/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TOUCHPROBE_H_
#define TOUCHPROBE_H_

//*** INCLUDE ************************************************************************************* 

#include "libs/Module.h"
#include "Pin.h"
#include <stdio.h>

//*** CLASS ***************************************************************************************

class StepperMotor;
class Vector3;

class Touchprobe: public Module {
    public:
        Touchprobe();
        
        void on_module_loaded() override;
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument) override;
        void on_idle(void* argument) override; 
        
        bool doProbing(int StepsMoved[3], float AbsoluteMotorPositions[3], float RelativeTargetPosition[3], float ProbeSpeed, bool ReturnToStart = true);
        bool returnProbe(int StepsMoved[3], float MotorFeedrate);
        void moveProbe(float x, float y, float z, float Feedrate, bool Relative);
        
    private:
        bool waitForProbe(unsigned int StepsMoved[3]);
        bool waitForMotorStop();
        void flush_log();

        FILE*          logfile;
        std::string    filename;
        StepperMotor*  steppers[3];
        Pin            pin;
        unsigned int   debounce_count;

        float          ProbeSpeed;
        unsigned int   FlushLogfileMCode;
        bool           enabled;
        bool           should_log;
};

#endif
