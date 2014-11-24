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
float vision_rollangle;

bool_t go_to_blue;
float vision_turnStepSize;
volatile uint32_t hysteresesDelay;
uint32_t vision_hysteresesDelayFactor;
uint32_t vision_filterWidth;

uint32_t noroofcnt;
volatile uint32_t objectcnt;
uint8_t lastReceivedStereoCnt;
uint8_t lastReceivedColorCnt_L;
uint8_t lastReceivedColorCnt_R;




extern void autoheading_turnButton(uint8_t whatever) {
    whatever = whatever; //kill warning
    vision_turnbutton = true; // give the turn command to periodic()
}

extern void autoheading_setMaxU(uint8_t value) {
     value = value;
    
    // unsigned char tmp[3];
    // tmp[0] = 'U';
    // tmp[1] = value;
    // tmp[2] = '\0';

    // StereoLink(Transmit(tmp));
}

extern void autoheading_start(void){
    vision_turnspeed = 0.23;
    vision_colorthreshold = 1;
    vision_objectthreshold = 255;
    vision_turnbutton = false;
    vision_pitchangle=-5.2;
    
    vision_turnStepSize=90;
    vision_filterWidth = 1;
    vision_hysteresesDelayFactor = 3;
    vision_rollangle = 10.0;
    go_to_blue = FALSE;
    

    hysteresesDelay=0;
    noroofcnt = 0;
    objectcnt = 0;

    UART1Init();
}


extern void autoheading_stop(void) {	

}

static bool handleColorPackage(void);
static bool handleColorPackage(void) {
    //check the full header of this package:    
    uint8_t c = StereoLink(Getch());
    if (c != 'o') {return false;}
    c = StereoLink(Getch());
    if (c != 'l') {return false;}

    //ok, certain of color package. Retrieve the package:

    uint8_t data[8];
    data[3] = StereoLink(Getch());
    data[4] = StereoLink(Getch());
    data[5] = StereoLink(Getch());
    data[6] = StereoLink(Getch());
    data[7] = StereoLink(Getch());

    //some more error checking:
    if (data[5] != 255) {return false;}
    if (data[3] != data[6]) {return false;}
    if (data[4] != data[7]) {return false;}

    /********all ok, handle the data********/

    uint8_t cnt_R = data[3];
    uint8_t cnt_L = data[4];


    lastReceivedColorCnt_L = cnt_L;
    lastReceivedColorCnt_R = cnt_R;
 

    if ((hysteresesDelay==0) && (go_to_blue == TRUE)) { // wait until previous turn was completed
        if (cnt_L < vision_colorthreshold || cnt_R < vision_colorthreshold) { //if not enough color was detected either in the left or right half of the image
            //if (cnt_L < cnt_R)  { //turn to the half in which most color was measured
            incrementHeading(vision_turnStepSize);
            // } else {
            //     incrementHeading(vision_turnStepSize); 
            // }
            hysteresesDelay = (float)vision_hysteresesDelayFactor * (((float)vision_turnStepSize / (float)vision_turnspeed ) / (float)AUTOHEADING_PERIODIC_FREQ);                  
        } else {
            //LED_OFF(1); //off if nothing is happening
            noroofcnt = 0; // currently unused            
        }
    }
   

    return true;
}


static bool handleStereoPackage(void);
static bool handleStereoPackage(void) {
    //check the full header of this package:    
    uint8_t c = StereoLink(Getch());
    if (c != 'i') {return false;}
    c = StereoLink(Getch());
    if (c != 's') {return false;}

    //ok, certain of color package. Retrieve the package:

    uint8_t data[8];
    data[3] = StereoLink(Getch());
    data[4] = StereoLink(Getch());
    data[5] = StereoLink(Getch());
    
    //some more error checking:
    if (data[4] != 255) {return false;}
    if (data[3] != data[5]) {return false;}
    
    /********all ok, handle the data********/

    c =  data[3];
    lastReceivedStereoCnt = c; // save it globally, for the telemetry message
   
    if ((hysteresesDelay==0) && (go_to_blue == TRUE)) { // wait until previous turn was completed
            if (c > vision_objectthreshold) {  //if an object was detected
                objectcnt++;
                if (objectcnt > vision_filterWidth) { // if an object was detected long enough
                    objectcnt=0;                    
                    hysteresesDelay = (float)vision_hysteresesDelayFactor * (((float)vision_turnStepSize / (float)vision_turnspeed ) / (float)AUTOHEADING_PERIODIC_FREQ); 
                    incrementHeading(vision_turnStepSize);
                } 
            } else {
                objectcnt=0;
                //LED_OFF(1); //off if nothing is happening        
            }            
    }
    return true;    
}


uint8_t fuckingsuperbitrfreducer;


extern void autoheading_periodic(void) {
        
    // if (UART1ChAvailable()) 
    // {         
        bool res = false;
        while (!res && UART1ChAvailable()) {
            uint8_t c = StereoLink(Getch());
            
            if (c == 'c')  {
                res = handleColorPackage();
            } else if  (c == 'd') {
                res = handleStereoPackage();
            } else {
                //weird data; kill it with fire     
                while (StereoLink(ChAvailable()))
                    StereoLink(Getch());  
            }
        }   
    // }



fuckingsuperbitrfreducer = (fuckingsuperbitrfreducer +1) % 5;
    
    if (fuckingsuperbitrfreducer == 0) {
        //telemytrize that shit
        DOWNLINK_SEND_STEREO(DefaultChannel, DefaultDevice, &vision_colorthreshold,&vision_objectthreshold, &lastReceivedColorCnt_L,&lastReceivedColorCnt_R, &lastReceivedStereoCnt, &hysteresesDelay);
    }




    if (hysteresesDelay>0) { //keep track whether the drone is turning
        hysteresesDelay--;        
        setAutoHeadingPitchAngle(0); // if the drone is turning, pitch backward to slow down
        
        setAutoHeadingRollAngle(vision_rollangle);
  

        LED_TOGGLE(1);
    } else {

        setAutoHeadingPitchAngle(vision_pitchangle); // if not turning, try to keep constant forward speed
        setAutoHeadingRollAngle(0.0);
        LED_OFF(1);
    }

    setHeading_P(vision_turnspeed); // set turn speed, should be moved out of periodic loop...

   if (vision_turnbutton) {
        vision_turnbutton = false; // make it a one shot turn
        incrementHeading(vision_turnStepSize);
   } 
}


