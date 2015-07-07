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


#include "autopilot.h"
#include "guidance/guidance_h_ref.h"

 #include "subsystems/datalink/downlink.h"


 /*  private function declarations  */
bool initSocket(void) ;
bool Read_socket(char * c, size_t maxlen) ;
int closeSocket(void);
bool Write_socket(char * c, size_t n);

/*  Global variables  */
int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
struct    sockaddr_in servaddr;  /*  socket address structure  */
struct 	ICDataPackage tcp_data;



float alpha;

int32_t IC_threshold_nn;
int32_t IC_threshold_gt;
int32_t IC_threshold_gtstd;
bool IC_turnbutton;

int8_t IC_flymode;
int8_t IC_learnmode;
int8_t IC_actionDummy;

bool obstacle_detected; // signal to flightplan if obstacle is detected
int noDataCounter;      //signal to flightplan if IC is running OK

int closeSocket(void) {
	return close(list_s);
}

bool initSocket() {

    /*  Create the listening socket  */
    if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
	   fprintf(stderr, "TCP server: Error creating listening socket.\n");
	   return true;
    }
     printf("list_s: %d\n",list_s );

    /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

	char ipa[15];
	sprintf(ipa, "127.0.0.1");

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(PORT);
	if(inet_pton(AF_INET, ipa, &servaddr.sin_addr)<=0)
    {
        printf("inet_pton error occured\n");
        return true;
    } 



    connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) ;
    //if(!connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) )
    //{
    //   printf("Error connect failed: %s:%d, %d \n",ipa,PORT, test);
      // return true;
    //} 
     printf("Started IC program tcp listener: %s:%d \n",ipa,PORT);

	return false;
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
bool Write_socket(char * c, size_t n) {

    while ( n > 0 ) {
        int nwritten = 0;
        if ( (nwritten = write(list_s, c, n)) <= 0 ) {
            if ( errno == EINTR )
                nwritten = 0;
            else
                return true;
        }
        n -= nwritten;
        c += nwritten;
    }
    return false;
}

extern void IC_slave_FlyModeButton(int8_t value) {
    IC_flymode = value;
}
extern void IC_slave_LearnModeButton(int8_t value) {
    IC_learnmode = value;    
    char str[2];
    str[0]=value+48;
    str[1]=0;
    Write_socket(str,2);
    printf("Send to IC %s\n", str);
}

extern void IC_slave_ActionButton(int8_t value) {
    char str[2];
    IC_actionDummy = value;
    if (value==0) { 
        str[0]='c';
    } else if(value==1) { 
        str[0]='i';
    } else if(value==2) { 
        str[0]='s';
    } else if(value==3) { 
        str[0]='l';
    } 
    str[1]=0;
    Write_socket(str,2);
    printf("Send to IC %s\n", str);
}



extern void IC_start(void){		
	obstacle_detected = false;
    
    IC_threshold_gt = 100;
    IC_threshold_nn = 30;
    IC_threshold_gtstd = 55;
            
    IC_turnbutton=true;
    noDataCounter=0;

    IC_learnmode = stereo_textons; // current default in IC
    
    if (initSocket()) {
        printf("Could not connect to IC\n"); // hmm, this does not work
        exit(1);
    }	
	printf("IC module started\n");
}


extern void IC_stop(void) {	
	closeSocket();
	printf("IC module stopped\n");	
}

extern void IC_periodic(void) {
	//read the data from the video tcp socket
	
	char * c = (char *) &tcp_data; 
	if (Read_socket(c,sizeof(tcp_data))) {
        noDataCounter++;
        if (noDataCounter>100) {
            printf("No IC data received for too long.") ;
            closeSocket();
            initSocket();
            noDataCounter=0;           
        }
        return;
    } else {
        noDataCounter=0;
    }
	if (IC_flymode==stereo) {
        printf("IC gt: %d, std: %d, thresh_gt: %d, fps: %f\n",tcp_data.avgdisp_gt,tcp_data.avgdisp_gt_stdev,IC_threshold_gt, tcp_data.fps);
    } else {
        printf("IC nn: %d, thresh_nn: %d, fps: %f\n",tcp_data.avgdisp_nn,IC_threshold_nn, tcp_data.fps);
    }
    

if (IC_flymode==stereo){
    //if (tcp_data.avgdisp_gt_stdev < IC_threshold_gtstd) {
    if (tcp_data.avgdisp_gt > IC_threshold_gt) {
            obstacle_detected = true;
        // } else {
        //     obstacle_detected = false;
        // }
    }
    else { // if variance is too high, probably only far away objects....
        obstacle_detected = false;
    }

} else {
    obstacle_detected = (tcp_data.avgdisp_nn > IC_threshold_nn); 
}

    DOWNLINK_SEND_STEREO(DefaultChannel, DefaultDevice, &(tcp_data.avgdisp_gt),&(tcp_data.avgdisp_gt_stdev),&(tcp_data.avgdisp_nn), &IC_threshold_gt,&IC_threshold_gtstd,&IC_threshold_nn, &alpha,&(tcp_data.fps));


   //obstacle_detected = IC_turnbutton;  // test switch in  IC settings tab
}

/************************** FLIGHT PLAN FUNCTIONS *********************************/
/*
* Rotates (yaw) the heading of drone with increment
*
*/
bool increase_nav_heading(int32_t *heading, int32_t increment) {
     *heading = *heading + increment;
  return false;
}


/*
* Moves goal waypoint *wp_id_goal* forward with *distance* in direction of *heading* from *wp_id_current*
*
*/
 bool increase_nav_waypoint(int wp_id_current,int wp_id_goal, int32_t distance, int32_t heading) {

    alpha = -ANGLE_FLOAT_OF_BFP(heading) +1.5708;

    struct EnuCoor_i *wpc = &waypoints[wp_id_current].enu_i;
    struct EnuCoor_i *wpg = &waypoints[wp_id_goal].enu_i;

    float x= cosf(alpha) * (float)distance;
    float y= sinf(alpha) * (float)distance;
    (*wpg).x = (*wpc).x+ x;
    (*wpg).y = (*wpc).y+ y;

    return false;
  
}

bool goBackaBit(int wp_id_current,int wp_id_prevgoal) {

    struct EnuCoor_i wpc = waypoints[wp_id_current].enu_i;
    //struct EnuCoor_i *wpg = &waypoints[wp_id_goal];
    struct EnuCoor_i wpp = waypoints[wp_id_prevgoal].enu_i;

    wpc.x = wpp.x+ (wpc.x - wpp.x)/2;
    wpc.y = wpp.y+ (wpc.y - wpp.y)/2;
    return false;
}
