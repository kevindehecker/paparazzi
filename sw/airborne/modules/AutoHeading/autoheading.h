/*
 * Copyright (C) 2014 Kevin van Hecke
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/autoheading.h
 *  @brief interface to usb stereocam 
 */

#ifndef AUTOHEADING_H_
#define AUTOHEADING_H_

#include <unistd.h>             /*  for ssize_t data type  */
#include "std.h"
 
extern uint8_t vision_objectthreshold;
extern uint8_t vision_colorthreshold;
extern float vision_turnspeed;
extern bool vision_turnbutton;
extern float vision_pitchangle;
extern float vision_turnStepSize;
extern uint32_t vision_hysteresesDelayFactor;
extern uint32_t vision_filterWidth;


extern uint8_t maxU;
extern uint8_t minU;
extern uint8_t maxV;
extern uint8_t minV;



extern void autoheading_start(void);
extern void autoheading_stop(void);
extern void autoheading_periodic(void);

extern void autoheading_turnButton(uint8_t whatever);
extern void autoheading_setMaxU(uint8_t value);


#endif /* AUTOHEADING_H_ */
 