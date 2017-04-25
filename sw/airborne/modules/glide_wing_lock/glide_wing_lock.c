/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/glide_wing_lock/glide_wing_lock.c"
 * @author Kevin van Hecke
 * Locks the wing of an ornicopter so it can glide.
 */

#include "modules/glide_wing_lock/glide_wing_lock.h"

#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

#include "mcu_periph/adc.h"
#include "subsystems/commands.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"

struct adc_buf adcbuf;

int lock_wings;
int last_commanded_thrust = 0;
//wing down ~1568-1650
//wing up ~2930-50
//wing going down > 0 .. 1500
//wing going up 1500 .. 2999
#ifndef WING_POS_FULL_UP
#define WING_POS_FULL_UP 3000 // mod 3000
#endif
#ifndef WING_POS_FULL_DOWN
#define WING_POS_FULL_DOWN 1500
#endif
#ifndef WING_RANGE_THRESH
#define WING_RANGE_THRESH 100
#endif

#ifndef WING_POS_LOCK_MIN_THRESH
#define WING_POS_LOCK_MIN_THRESH 800
#endif
#ifndef WING_POS_LOCK_MAX_THRESH
#define WING_POS_LOCK_MAX_THRESH 900
#endif
#ifndef WING_PRELOCK_THRUST
#define WING_PRELOCK_THRUST 3000
#endif
#ifndef WING_PRELOCK_MIN_THRUST
#define WING_PRELOCK_MIN_THRUST 2300
#endif
#ifndef WING_POS_LOCK_SWITCH
#define WING_POS_LOCK_SWITCH RADIO_AUX2
#endif

int wing_lock_pos_max_thresh = WING_POS_LOCK_MAX_THRESH;
int wing_lock_pos_min_thresh = WING_POS_LOCK_MIN_THRESH;
int wing_prelock_thrust = WING_PRELOCK_THRUST;
int wing_prelock_min_thrust = WING_PRELOCK_MIN_THRUST;

void glide_wing_lock_init(void)
{
  adc_buf_channel(ADC_CHANNEL_MOTORSENSOR, &adcbuf, 1);
}
float wpos = 0;
void glide_wing_lock_event()
{
  static int lockstate = 2;
  if (radio_control.values[WING_POS_LOCK_SWITCH] > (MIN_PPRZ / 2) && last_commanded_thrust > 0) {

    if (last_commanded_thrust < wing_prelock_min_thrust)
      last_commanded_thrust = wing_prelock_min_thrust;

    wpos = adcbuf.sum / adcbuf.av_nb_sample;
    switch (lockstate) {
      case 0:
        lock_wings = 1; // immidiately put speed to fixed speed, so that rc throttle is not influencing anything anymore
        if (wpos > WING_POS_FULL_UP - WING_RANGE_THRESH || wpos < WING_RANGE_THRESH) { //wait until wing position is full up
          lock_wings = 2;
          lockstate++;
        }
        break;
      case 1:
        if (wpos > wing_lock_pos_min_thresh && wpos < wing_lock_pos_max_thresh) { // wait for exact wing position (just under lock position) and direction (going down)
          //esc brakes when throttle = 0, which should lock the wing in this position;
          lock_wings = 3;
          lockstate++;
        }
        break;
      default:
        break;
    }
  } else {
    lock_wings = 0;
    lockstate = 0;
  }
}

uint16_t tmppos;
void glide_wing_lock_periodic()
{
  uint16_t wpostmp = adcbuf.sum / adcbuf.av_nb_sample;
  DOWNLINK_SEND_ADC_GENERIC(DefaultChannel, DefaultDevice, &wpostmp, &tmppos);
}



void set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight, bool motors_on)
{
  if (!(in_flight)) { cmd_in[COMMAND_YAW] = 0; }
  if (!(motors_on)) { cmd_in[COMMAND_THRUST] = 0; }
  cmd_out[COMMAND_ROLL] = cmd_in[COMMAND_ROLL];
  cmd_out[COMMAND_PITCH] = cmd_in[COMMAND_PITCH];
  cmd_out[COMMAND_YAW] = cmd_in[COMMAND_YAW];
  cmd_out[COMMAND_THRUST] = cmd_in[COMMAND_THRUST];

  if (lock_wings == 0)
    last_commanded_thrust = cmd_in[COMMAND_THRUST];

  if (lock_wings == 1)
    cmd_out[COMMAND_THRUST] = last_commanded_thrust;

  if (lock_wings == 2) {
    //apply linear brake
    float tmp = WING_POS_LOCK_MIN_THRESH - wpos;
    float f = (WING_PRELOCK_THRUST/WING_POS_LOCK_MIN_THRESH) * tmp;

    if (f > wing_prelock_min_thrust && f < WING_PRELOCK_THRUST)
      cmd_out[COMMAND_THRUST] = f;
    else if (f<wing_prelock_min_thrust)
      cmd_out[COMMAND_THRUST] = wing_prelock_min_thrust; // minimal thrust to keep wing moving
    else
      cmd_out[COMMAND_THRUST] = WING_PRELOCK_THRUST;

  } else if (lock_wings == 3) {
    cmd_out[COMMAND_THRUST] = 0;
  }
   tmppos =  cmd_out[COMMAND_THRUST]; //tmp pprz msg variable
}



