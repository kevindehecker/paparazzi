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

 #include <unistd.h>
#include <fcntl.h>

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
int Read_socket(void);
int closeSocket(void);
bool Write_socket(char * c, size_t n);

/*  Global variables  */
int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
struct    sockaddr_in servaddr;  /*  socket address structure  */
struct 	ICDataPackage tcp_data;

struct  ICDataPackage tcp_data_tmpbuf;
int nBytesInBuf;

float navHeading;

float alpha;

int32_t IC_threshold_est; // unused!
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
     nBytesInBuf=0;

     fcntl(list_s, F_SETFL, O_NONBLOCK);

	return false;
}

/*
* Reads the in buffer until all data has been processed. If more than one new tcp_data struct is received, they are overwritten with the last fully received struct
*/
int Read_socket() {
    char *c = (char *) &tcp_data_tmpbuf;
    int tmpn = 1;
    int totaln =0;

    while (true) { // run until all data has been received, or an error occured
        tmpn =read(list_s, c+nBytesInBuf, sizeof(tcp_data)-nBytesInBuf);
        if (tmpn>0) {
            totaln+=tmpn;
            nBytesInBuf+=tmpn;
            //printf("Received %d bytes\n", nBytesInBuf );
            if (nBytesInBuf==sizeof(tcp_data)) {
                nBytesInBuf=0;
                memcpy(&tcp_data,&tcp_data_tmpbuf,sizeof(tcp_data));
            }
        } else if (tmpn<0)  {
            //printf("Err %d, %d, %d\n", tmpn,nBytesInBuf,totaln );
            return totaln;
        } else { // I don't think this output is possible with non-blocking read?
            //printf("No data available, %d from %d bytes\n", nBytesInBuf,sizeof(tcp_data));
            return totaln;
        }
    }
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
   // IC_slave_setThreshold(IC_threshold_gt); //tmp solution...
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

extern void IC_slave_setThreshold(int8_t value) {
    char str[2];
    str[0]=value;
    str[1]=0;
    Write_socket(str,2);
    printf("Send to IC %s\n", str);
}

extern void IC_start(void){

    IC_threshold_gt = 12;
    // IC_threshold_est = 1;
    // IC_threshold_gtstd = 55;

    IC_turnbutton=true;
    noDataCounter=0;
    nav_heading=0;

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
    static int8_t tmp = 0;
    tmp++;
    if (tmp == 10) {
        tmp=0;
        DOWNLINK_SEND_STEREO(DefaultChannel, DefaultDevice, &(tcp_data.avgdisp_gt),&(tcp_data.avgdisp_gt_stdev),&(tcp_data.avgdisp_est), &IC_threshold_gt,&IC_threshold_gtstd,&(tcp_data.avgdisp_est_thresh), &navHeading,&(tcp_data.fps));
    }



	if (!Read_socket()) {
        noDataCounter++;
        if (noDataCounter>100) {
            tcp_data.avgdisp_gt=16;
            printf("No IC data received for too long.") ;
            closeSocket();
            initSocket();
            noDataCounter=0;
        }
        return; // no new data, so exit the function
    }
    noDataCounter=0; // reset time out counter
    IC_threshold_est= tcp_data.avgdisp_est_thresh;

	if (IC_flymode==stereo) {
        printf("IC gt: %d, std: %d, thresh_gt: %d, est: %d, thresh_est: %d fps: %f\n",tcp_data.avgdisp_gt,tcp_data.avgdisp_gt_stdev,IC_threshold_gt,tcp_data.avgdisp_est,tcp_data.avgdisp_est_thresh, tcp_data.fps);
    } else {
        printf("IC est: %d, thresh_est: %d, fps: %f\n",tcp_data.avgdisp_est,tcp_data.avgdisp_est_thresh, tcp_data.fps);
    }

    if (IC_flymode==stereo) {
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
        obstacle_detected = (tcp_data.avgdisp_est > tcp_data.avgdisp_est_thresh);
    }




       //obstacle_detected = IC_turnbutton;  // test switch in  IC settings tab
}


bool init_nav_heading() {
    navHeading =stateGetNedToBodyEulers_f()->psi;
    return false;
}


/************************** FLIGHT PLAN FUNCTIONS *********************************/
/*
* Rotates (yaw) the heading of drone with increment
*
*/
bool increase_nav_heading( float increment) {
     navHeading = navHeading + increment;
     if (navHeading > 6.27) {
        navHeading=0.0;
     }
     NavHeading(navHeading);
  return false;
}
float rh; //random heading
bool rh_reached;
bool set_rand_heading() {
    rh=(float)rand()/(float)(RAND_MAX);
    rh*=2;
    rh_reached=false;
    return false;
}
bool increase_nav_heading_till_r(float increment) {
    rh-=increment;
    if (rh>0) {
        navHeading = navHeading + increment;
        if (navHeading > 6.27) {
            navHeading=0.0;
        }
        NavHeading(navHeading);
    } else {
        rh_reached=true;
    }
  return false;
}
// bool set_rand_heading() {
//     //float r1 = (float)rand()/(float)(RAND_MAX);
//     //if (r1 > 0.8) {

// float r2 = (float)rand()/(float)(RAND_MAX);
// r2*=2;
// printf("Rand: %f\n",r2);
//         navHeading = navHeading + r2;

//      if (navHeading > 6.27) {
//         navHeading-=6.27;
//      }

//         NavHeading(navHeading);
//     //}
//     return false;
// }



/*
* Moves goal waypoint *wp_id_goal* forward with *distance* in direction of *heading* from *wp_id_current*
*
*/
 bool increase_nav_waypoint(int wp_id_current,int wp_id_goal, float distance) {

distance = distance/2; //tmp test

    alpha = -navHeading+1.57;


    struct EnuCoor_f *wpc = &waypoints[wp_id_current].enu_f;
    struct EnuCoor_f *wpg = &waypoints[wp_id_goal].enu_f;

    float x= cos(alpha) * distance;
    float y= sin(alpha) * distance;
    (*wpg).x = (*wpc).x+ x;
    (*wpg).y = (*wpc).y+ y;

    struct EnuCoor_i *wpg_i = &waypoints[wp_id_goal].enu_i;
    wpg_i->x = POS_BFP_OF_REAL(wpg->x);
    wpg_i->y = POS_BFP_OF_REAL(wpg->y);
    waypoint_globalize(wp_id_goal);


    DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel, DefaultDevice, (uint8_t *)&wp_id_goal, &(wpg_i->x), &(wpg_i->y), &(wpg_i->z));


    return false;

}

// bool goBackaBit(int wp_id_current,int wp_id_prevgoal) {

//     struct EnuCoor_i wpc = waypoints[wp_id_current].enu_i;
//     //struct EnuCoor_i *wpg = &waypoints[wp_id_goal];
//     struct EnuCoor_i wpp = waypoints[wp_id_prevgoal].enu_i;

//     wpc.x = wpp.x+ (wpc.x - wpp.x)/2;
//     wpc.y = wpp.y+ (wpc.y - wpp.y)/2;
//     return false;
// }


//VOLGENS EWOUD:
//                    x
//                     |       *
//                     |      *
//                     |alpha*
//                     |    s
//                     |   *
//                     |  *
//                     | *
// -------------------------------------------y

//PRAKTIJK:
//                    y
//                     |       *
//                     |      *
//                     |     *
//                     o    s
//                     |   *
//                     |  *
//                     | *  -alpha+pi/2
// --------------------------a----------------x
//
//  cos(alpha) = a*s -> a = cos(alpha)/s
//
//  sin(alpha) = o*s -> o = sin(alpha)/s

