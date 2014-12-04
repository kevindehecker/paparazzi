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

/** @file modules/IC_slave.h
 *  @brief tcp interface to IC
 */

 #include "IC_slave.h"

#include <stdbool.h>
#include <stdio.h>

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */
#include <errno.h>
#include <string.h> 		/* memset */

#include <stdlib.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h> 

#include "firmwares/rotorcraft/guidance/guidance_h.h" // to set heading
#include "generated/modules.h" // to get periodic frequency


 #include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
 #include "navigation.h"

 #include "subsystems/datalink/downlink.h"


 /*  private function declarations  */
int initSocket(void) ;
bool Read_socket(char * c, size_t maxlen) ;
int closeSocket(void);

/*  Global variables  */
int       list_s;                /*  listening socket          */
struct    sockaddr_in servaddr;  /*  socket address structure  */
struct 	ICDataPackage tcp_data;

float alpha;

int32_t IC_threshold;
int32_t IC_threshold_std;
bool IC_turnbutton;
float IC_turnspeed;
float IC_pitchangle;
float IC_rollangle;
float IC_turnStepSize;

uint32_t hysteresesDelay;
uint32_t IC_hysteresesDelayFactor;

bool obstacle_detected;



int closeSocket(void) {
	return close(list_s);
}

int initSocket() {

    /*  Create the listening socket  */
    if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
	   fprintf(stderr, "TCP server: Error creating listening socket.\n");
	   return 1;
    }
    // printf("list_s: %d\n",list_s );

    /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

	char ipa[15];
	sprintf(ipa, "127.0.0.1");

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(PORT);
	if(inet_pton(AF_INET, ipa, &servaddr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return 1;
    } 

    if( !connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
       printf("\n Error connect failed: %s:%d \n",ipa,PORT);
       return 1;
    } 
     printf("Connected to IC program: %s:%d \n",ipa,PORT);

	return 0;
}

bool Read_socket(char * c, size_t maxlen) {
    int n = 0;    
    while (n<maxlen) {    
        usleep(1000); //TODO: improve this
        int tmpn = read(list_s, c+n, maxlen-n);        
        if (tmpn>0) {
        	n+=tmpn;      	
        } else {return true;} // return problem == true
        
    }
    return false;
}


extern void IC_slave_TurnButton(bool value) {
    IC_turnbutton = value;
}

extern void IC_start(void){
		
	obstacle_detected = false;
    IC_turnspeed = 0.1;
    IC_threshold = 90;
    IC_threshold_std = 70;
    IC_turnbutton = false;
    IC_pitchangle=-2;
  
    IC_turnStepSize=90;
    IC_hysteresesDelayFactor = 3;
    IC_rollangle = 10.0;    
    
    hysteresesDelay=0;
    IC_turnbutton=true;
    
    initSocket();	
	printf("IC module started\n");


}


extern void IC_stop(void) {	
//	closeSocket();
	printf("IC module stopped\n");	
}

extern void IC_periodic(void) {
	//read the data from the video tcp socket
	
	char * c = (char *) &tcp_data; 
	if (Read_socket(c,sizeof(tcp_data))) {return;};
	printf("IC gt: %d, std: %d, nn: %d, thresh: %d\n",tcp_data.avgdisp_gt,tcp_data.avgdisp_gt_stdev,tcp_data.avgdisp_nn, IC_threshold);	

    if (tcp_data.avgdisp_gt_stdev < IC_threshold_std) {
        if (tcp_data.avgdisp_gt > IC_threshold) {
            obstacle_detected = true;
        } else {
            obstacle_detected = false;
        }
    }
    else { // if variance is too high, probably only far away objects....
        obstacle_detected = false;

    }


    DOWNLINK_SEND_STEREO(DefaultChannel, DefaultDevice, &(tcp_data.avgdisp_gt),&(tcp_data.avgdisp_gt_stdev),&(tcp_data.avgdisp_nn), &IC_threshold,&IC_threshold_std, &alpha);


   //  if (hysteresesDelay==0)  { // wait until previous turn was completed
   //      if (tcp_data.avgdisp_gt > IC_threshold) { //if object detected
            
   //          incrementHeading(IC_turnStepSize);
            
   //          hysteresesDelay = (float)IC_hysteresesDelayFactor * (((float)IC_turnStepSize / (float)IC_turnspeed ) / (float)IC_PERIODIC_FREQ);                  
   //      } 
   //  }


   //  if (hysteresesDelay>0) { //keep track whether the drone is turning
   //      hysteresesDelay--;        
   //      setAutoHeadingPitchAngle(0); // if the drone is turning, pitch backward to slow down        
   //      setAutoHeadingRollAngle(IC_rollangle);
   //  } else {
   //      setAutoHeadingPitchAngle(IC_pitchangle); // if not turning, try to keep constant forward speed
   //      setAutoHeadingRollAngle(0.0);        
   //  }

   //  setHeading_P(IC_turnspeed); // set turn speed, should be moved out of periodic loop...

   // if (IC_turnbutton) {
   //      IC_turnbutton = false; // make it a one shot turn
   //      incrementHeading(IC_turnStepSize);
   // }

   //obstacle_detected = IC_turnbutton;  // test switch in  IC settings tab
}


//AP_MODE_ATTITUDE_Z_HOLD (A_ZH) , heading aanpassen


bool increase_nav_heading(int32_t *heading, int32_t increment) {
  *heading = *heading + increment;
  return false;
}


/*
* Moves a waypoint forward with *distance* in direction of *heading*
*
*/
 bool increase_nav_waypoint(int wp_id_current,int wp_id_goal, int32_t distance, int32_t heading) {

    alpha = -ANGLE_FLOAT_OF_BFP(heading) +1.5708;
    // DOWNLINK_SEND_STEREO(DefaultChannel, DefaultDevice, &alpha);

    struct EnuCoor_i *wpc = &waypoints[wp_id_current];
    struct EnuCoor_i *wpg = &waypoints[wp_id_goal];

    //float alpha = (float)heading * 0.0175;

    float x= cosf(alpha) * (float)distance;
    float y= sinf(alpha) * (float)distance;
    (*wpg).x = (*wpc).x+ x;
    (*wpg).y = (*wpc).y+ y;

    return false;
  
}

bool goBackaBit(int wp_id_current,int wp_id_prevgoal) {

    struct EnuCoor_i wpc = waypoints[wp_id_current];
    //struct EnuCoor_i *wpg = &waypoints[wp_id_goal];
    struct EnuCoor_i wpp = waypoints[wp_id_prevgoal];

    wpc.x = wpp.x+ (wpc.x - wpp.x)/2;
    wpc.y = wpp.y+ (wpc.y - wpp.y)/2;
    return false;
}

