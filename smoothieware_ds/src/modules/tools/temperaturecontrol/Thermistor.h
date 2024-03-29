/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "TempSensor.h"
#include "RingBuffer.h"
#include "Pin.h"

#include <tuple>

#define QUEUE_LEN 32


class Thermistor : public TempSensor
{
    public:
        Thermistor();
        ~Thermistor();

        // TempSensor interface.
        void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
        float get_temperature();
        bool set_optional(const sensor_options_t& options);
        bool get_optional(sensor_options_t& options);
        void get_raw();
        static std::tuple<float,float,float> calculate_steinhart_hart_coefficients(float t1, float r1, float t2, float r2, float t3, float r3);

    private:
        int new_thermistor_reading();
        float adc_value_to_temperature(int adc_value);
        void calc_jk();

        // Thermistor computation settings using beta, not used if using SHHhttp://panucattdevices.freshdesk.com/helpdesk/attachments/1015374088
        float r0;
        float t0;

        // on board resistor settings
        int r1;
        int r2;

        union {
            // this saves memory as we only use either beta or SHH
            struct{
                float beta;
                float j;
                float k;
            };
            struct{
                float c1;
                float c2;
                float c3;
            };
        };

        Pin  thermistor_pin;

        RingBuffer<uint16_t,QUEUE_LEN> queue;  // Queue of readings

        struct {
            bool bad_config:1;
            bool use_steinhart_hart:1;
        };
};

#endif
