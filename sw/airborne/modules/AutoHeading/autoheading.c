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
#include "led.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

#define STEREO_PORT    UART1

#define __StereoLink(dev, _x) dev##_x
#define _StereoLink(dev, _x)  __StereoLink(dev, _x)
#define StereoLink(_x) _StereoLink(STEREO_PORT, _x)
#define StereoBuffer() StereoLink(ChAvailable())

uint8_t vision_objectthreshold;
uint8_t vision_colorthreshold;
bool vision_turnbutton;
float vision_turnspeed;
float vision_pitchangle;

float vision_turnStepSize;
uint32_t hysteresesDelay;
uint32_t vision_hysteresesDelayFactor;
uint32_t vision_filterWidth;

uint32_t noroofcnt;

uint8_t maxU;
uint8_t minU;
uint8_t maxV;
uint8_t minV;
uint8_t str[2] = {};

int debugcnt;

extern void autoheading_turnButton(float whatever) {
    whatever = whatever;
    vision_turnbutton = true;
}

extern void autoheading_setMaxU(uint8_t value) {
    // maxU = value;
    
    // unsigned char tmp[3];
    // tmp[0] = 'U';
    // tmp[1] = value;
    // tmp[2] = '\0';

    // StereoLink(Transmit(tmp));
}

extern void autoheading_start(void){
    vision_turnspeed = 1;
    vision_colorthreshold = 5;
    vision_objectthreshold = 5;
    vision_turnbutton = false;
    vision_pitchangle=-2.5;
    hysteresesDelay=0;
    vision_turnStepSize=95;
    vision_filterWidth = 5;
    vision_hysteresesDelayFactor = 2;
    noroofcnt = 0;

    maxU = 66;
    minU = 66;
    maxV = 66;
    minV = 66;


    UART1Init();
}


extern void autoheading_stop(void) {	

}

static void stereo_parse(uint8_t c);
static void stereo_parse(uint8_t c) {
    
    debugcnt =(debugcnt+1) % 10 ;
    if (debugcnt == 0) {
        /* debug message */
        if (c > 127) {
            str[1] = c-128; // roof/color
        }
        else {
            str[0] = c; // objects
        }
        DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 2, str);
    }
    if (hysteresesDelay==0) {
        
        if (c > 127) { // roof detector byte
            if (c-128 < vision_colorthreshold) {  
                noroofcnt++;
                if (noroofcnt >= vision_filterWidth) {
                    hysteresesDelay = vision_hysteresesDelayFactor * ((vision_turnStepSize / vision_turnspeed ) / (float)AUTOHEADING_PERIODIC_FREQ);
                    incrementHeading(vision_turnStepSize);           
                    LED_ON(1);   
                }                             
            } else {
                //reset movg avg
                noroofcnt = 0;


            }
        } else { // stereo detector byte
            if (c >= vision_objectthreshold) {  
                //hysteresesDelay = (vision_turnStepSize / vision_turnspeed ) / (float)AUTOHEADING_PERIODIC_FREQ;
               // incrementHeading(vision_turnStepSize);                    
                //LED_ON(1); 
            } 
        }        
    } else {
        LED_OFF(1); //off if nothing is happening
    }
    
}

extern void autoheading_periodic(void) {
    
    stereo_parse(StereoLink(Getch()));               
    if (UART1ChAvailable())
    {
      while (StereoLink(ChAvailable()))
        stereo_parse(StereoLink(Getch()));
    }
   
    if (hysteresesDelay>0) {
        hysteresesDelay--;        
    }
    setHeading_P(vision_turnspeed);
    setAutoHeadingPitchAngle(vision_pitchangle);

    if (vision_turnbutton) {
        vision_turnbutton = false; // make it a one shot turn
        incrementHeading(vision_turnStepSize);
    }
}


