/**
 * @file profile_position.c
 * @brief Profile Generation for Position
 * Implements position profile based on Linear Function with
 * Parabolic Blends.
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include "MC_PositionProfile.h"

float position_profile_q(position_profile_t pp, float t);
float position_profile_v(position_profile_t pp, float t);
float position_profile_a(position_profile_t pp, float t);

int32_t position_profile_init(position_profile_t *position_profile, float target_position, float actual_position, float velocity, float acceleration)
{
  float Throw, Yaux;

  position_profile->qf = (float) target_position;
  position_profile->qi = (float) actual_position;
  position_profile->Vm = (float) velocity;
  position_profile->Acc = (float) acceleration;
  position_profile->gama = 0.5f;


  Throw = position_profile->qf - position_profile->qi;

  /* Step 1 */
  position_profile->sgn = 1;
  if (Throw < 0)
  {
    position_profile->sgn = -1;
  }
  
  /* Step 2 */
  position_profile->Yf = fabsf(Throw);
  
  /* Step 3 */
  position_profile->Ys = position_profile->Yf / 2.0f;

  /* Step 4 */
  Yaux = 0.5f * (1.0f + position_profile->gama) * position_profile->Vm * position_profile->Vm / position_profile->Acc;

  /* Step 5 & 6 */
  if (position_profile->Ys <= Yaux)  /* Short throw motion */
  {
    position_profile->Ya = position_profile->Ys;
    position_profile->Vw = sqrtf(2.0f * position_profile->Ys*position_profile->Acc / (1.0f + position_profile->gama));
  }
  else                              /* Long throw motion */
  {
    position_profile->Ya = Yaux;
    position_profile->Vw = position_profile->Vm;
  }

  /* Step 7 */
  position_profile->To = position_profile->Vw/position_profile->Acc;

  /* Step 8 */
  position_profile->Ta = position_profile->To*(1.0f+position_profile->gama);

  /* Step 9 */
  position_profile->tao = position_profile->gama*position_profile->To;

  /* Step 10 */
  position_profile->Tm = position_profile->Ta/2.0f;

  /* Step 11 */
  position_profile->Tk = 2.0f*(position_profile->Ys - position_profile->Ya)/position_profile->Vm;

  /* Step 12 */
  position_profile->Ts = position_profile->Ta + position_profile->Tk/2.0f;

  /* Step 13 */
  position_profile->Tt = 2.0f * position_profile->Ts;


  position_profile->T = position_profile->Tt / 0.005f;  // 5 ms

  position_profile->s_time = 0.005f;                     // 5 ms

  position_profile->steps = (int32_t) round(position_profile->T);

  return position_profile->steps;
}

float position_profile_caculate(position_profile_t *position_profile, int32_t step)
{
  float t = position_profile->s_time * step;

  // If the actual and the target positions are already equal,
  // just return one of them as a target position
  if (position_profile->qi == position_profile->qf) 
  {
    position_profile->q = position_profile->qf;
    position_profile->v = 0;
    position_profile->a = 0;
    
    return position_profile->qf;
  }

  position_profile->q = position_profile->qi + (float)position_profile->sgn * position_profile_q(*position_profile, t);
  
  position_profile->v = (float)position_profile->sgn * position_profile_v(*position_profile, t);
  
  position_profile->a = (float)position_profile->sgn * position_profile_a(*position_profile, t);

  return position_profile->q;
}

float position_profile_q(position_profile_t pp, float t)
{
  float q = 0.0f;

  if (t <= 0.0f)
  {
    q = 0.0f;
  }
  else if (t <= pp.tao) 
  {
    q = pp.Acc * t * t * t / pp.tao / 6.0f;
  } 
  else if ( (t <= pp.To) && (pp.tao < t ) ) 
  {
    float faux = t - pp.tao * 0.5f;
    q = pp.Acc*faux*faux * 0.5f + pp.Acc*pp.tao*pp.tao/24.0f;
  } 
  else if ( (t <= pp.Ts) && (pp.To < t ) ) 
  {
    q = pp.Ys + pp.Vw*(t - pp.Ts) + position_profile_q(pp, pp.Ta - t);
  }
  else if ( pp.Ts < t  ) 
  {
    q = pp.Yf - position_profile_q(pp, pp.Tt - t);
  }  

  return q;
}

float position_profile_v(position_profile_t pp, float t)
{
  float v = 0.0f;

  if (t <= 0.0f)
  {
    v = 0.0f;
  }
  else if (t <= pp.tao) 
  {
    v = pp.Acc * t * t / pp.tao / 2.0f;
  } 
  else if ( (t <= pp.To) && (pp.tao < t ) ) 
  {
    v = pp.Acc*(2.0f * t - pp.tao)/2.0f;
  } 
  else if ( (t <= pp.Ts) && (pp.To < t ) ) 
  {
    v = pp.Vw - position_profile_v(pp, pp.Ta - t);
  }
  else if ( pp.Ts < t  ) 
  {
    v = position_profile_v(pp, pp.Tt - t);
  }  

  return v;
}

float position_profile_a(position_profile_t pp, float t)
{
  float a = 0.0f;

  if (t <= 0.0f)
  {
    a = 0.0f;
  }
  else if (t <= pp.tao) 
  {
    a = pp.Acc * t / pp.tao;
  } 
  else if ( (t <= pp.To) && (pp.tao < t ) ) 
  {
    a = pp.Acc;
  } 
  else if ( (t <= pp.Ts) && (pp.To < t ) ) 
  {
    a = position_profile_a(pp, pp.Ta - t);
  }
  else if ( pp.Ts < t  ) 
  {
    a = -position_profile_a(pp, pp.Tt - t);
  }  

  return a;
}

