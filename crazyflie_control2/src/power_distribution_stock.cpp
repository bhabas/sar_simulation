/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"
#include "controller_gtc.h"

void powerDistribution(control_t *control,const uint32_t tick)
{
    float f_thrust_g = control->thrust;
    float f_roll_g = (float)(control->roll)*1e-3f;
    float f_pitch_g = (float)(control->pitch)*1e-3f;
    float f_yaw_g = (float)(control->yaw)*1e-3f;

    M1_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g - f_pitch_g + f_yaw_g)); // Add respective thrust components and limit to (0 <= PWM <= 60,000)
    M2_pwm = limitPWM(thrust2PWM(f_thrust_g + f_roll_g + f_pitch_g - f_yaw_g));
    M3_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g + f_pitch_g + f_yaw_g));
    M4_pwm = limitPWM(thrust2PWM(f_thrust_g - f_roll_g - f_pitch_g - f_yaw_g));

    if(motorstop_flag){ // Cutoff all motor values
        f_thrust_g = 0.0f;
        M1_pwm = 0;
        M2_pwm = 0;
        M3_pwm = 0;
        M4_pwm = 0;
    }

    // printf("M1: %u\t M2: %u\t M3: %u\t M4: %u\n",M1_pwm,M2_pwm,M3_pwm,M4_pwm);
}