/**
 * @file profile.h
 * @brief Profile Generation for Position, Velocity and Torque
 *      Implements position profile based on Linear Function with
 *      Parabolic Blends, velocity profile and torque profiles are
 *      based on linear functions.
 * @author Synapticon GmbH <support@synapticon.com>
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POSITION_PROFILE_H
#define __POSITION_PROFILE_H

#ifdef __cplusplus
 extern "C" {
#endif
 
#include <stdio.h>
#include <stdint.h>
#include "math.h"

typedef struct _position_profile_t 
{
  /*User Inputs*/

  float Acc;                // max acceleration
  float Vm;                 // max velocity
  float qi;                 // initial position
  float qf;                 // final position

  /*Profile time*/

  float T;                  // total no. of Samples
  float s_time;             // sampling time

  /*internal velocity variables*/
  float gama;               // smoothness 0-Trapezoidal, 1-maxumum smoothness
  int32_t sgn;              // sign of travel distance
  float Yf;                 // abs(travel distance)
  float Ya;                 // distance of acceleration
  float Ys;                 // half Yf
  float Vw;                 // profile max velocity

  float To;                 // time for accelerate from 0 to Vw >> To = Vw/Acc
  float Ta;                 // total time for acceleration >> Ta = To*(1+gama)
  float tao;                // time for jerk (acceleration from 0 to Acc)
  float Tm;                 // half of Ta
  float Tk;                 // cruise time
  float Ts;                 // half of total time
  float Tt;                 // total profile time

  float q;                  // position profile output
  float v;                  // velocity profile output
  float a;                  // acceleration profile output

  int32_t steps;            // number of steps required to execute this profile

} position_profile_t;


/*Profile Position Mode*/

/**
 * @brief Initialize Position Profile
 *
 * @param motion_profile
 * @param target_position
 * @param actual_position
 * @param velocity for the position profile
 * @param acceleration for the position profile
 * @param deceleration for the position profile
 *
 * @return no. of steps for position profile : range [1 - steps]
 */
int32_t position_profile_init(position_profile_t *position_profile, float target_position, float actual_position, float velocity, float acceleration);

/**
 * @brief Generate Position Profile
 *
 * @param motion_profile
 * @param step current step of the profile
 *
 * @return corresponding target position at the step input
 */
float position_profile_caculate(position_profile_t *position_profile, int32_t step);


#ifdef __cplusplus
}
#endif
#endif


