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

/** @file modules/autoheading.c
 *  @brief interface to usb stereocam 
 */

 #include "autoheading.h"
 #include "generated/modules.h"

#include <stdbool.h>

#include "firmwares/rotorcraft/guidance/guidance_h.h" // to set heading

// Serial Port
#include "mcu_periph/uart.h"

#define STEREO_PORT    UART1


#include "led.h"




#define __StereoLink(dev, _x) dev##_x
#define _StereoLink(dev, _x)  __StereoLink(dev, _x)
#define StereoLink(_x) _StereoLink(STEREO_PORT, _x)
#define StereoBuffer() StereoLink(ChAvailable())

uint8_t vision_threshold;
bool vision_turnbutton;
float vision_turnspeed;
float vision_pitchangle;

float vision_turnStepSize;
uint32_t hysteresesDelay;


extern void autoheading_turnButton(float whatever) {
    whatever = whatever;
    vision_turnbutton = true;
}

extern void autoheading_start(void){
    vision_turnspeed = 1;
    vision_threshold = 7;
    vision_turnbutton = false;
    vision_pitchangle=-4;
    hysteresesDelay=0;
    vision_turnStepSize=120;
    UART1Init();
}


extern void autoheading_stop(void) {	

}


static void stereo_parse(uint8_t c);
static void stereo_parse(uint8_t c) {
    if (hysteresesDelay==0) {
        if (c < (vision_threshold + '0' )) {  
            hysteresesDelay = (vision_turnStepSize / vision_turnspeed ) / (float)AUTOHEADING_PERIODIC_FREQ;
            incrementHeading(vision_turnStepSize);
           // LED_ON(3); //RADIO_CONTROL_LED
           // LED_ON(2); 
            LED_ON(1); 
        } else {
           // LED_TOGGLE(3);
           // LED_TOGGLE(2);
            LED_TOGGLE(1);

        } 
    }

}

extern void autoheading_periodic(void) {
    if (hysteresesDelay>0) {
        hysteresesDelay--;        
    }
    

    stereo_parse(StereoLink(Getch()));

        
        
  if (UART1ChAvailable())
  {

    


    while (StereoLink(ChAvailable()))
      stereo_parse(StereoLink(Getch()));
  }

    

    setHeading_P(vision_turnspeed);
    setAutoHeadingPitchAngle(vision_pitchangle);

    if (vision_turnbutton) {
        vision_turnbutton = false; // make it a one shot turn
        incrementHeading(vision_turnStepSize);
    }
}


