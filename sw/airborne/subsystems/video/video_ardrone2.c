/*
 * Copyright (C) 2012-2013 Kevin van Hecke
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/video/video_ardrone2.c
 * Video implementation for ardrone2.
 *
 * Use the tcp output of a custom GStreamer framework plugin to receive 
 * telemetry based on video
 */

#include "video_ardrone2.h"
#include <stdio.h>
#include "video_message_structs.h"

#include "subsystems/electrical.h" // for testing only, to set vsupply

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

char** str_split(char* a_str, const char *  a_delim, unsigned int * amount);

struct VideoARDrone video_impl;
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

/*  Global constants  */

#define MAX_LINE           (1000)

/*  Global variables  */
int       list_s;                /*  listening socket          */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */


int closeSocket(void) {
	return close(list_s);
}

int initSocket() {

    /*  Create the listening socket  */
    if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
	fprintf(stderr, "tcp server: Error creating listening socket.\n");
	return -1;
    }


    /*  Set all bytes in socket address structure to
        zero, and fill in the relevant data members   */

	char ipa[10];
	sprintf(ipa, "127.0.0.1");

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(PORT);
	if(inet_pton(AF_INET, ipa, &servaddr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return 1;
    } 



    if( connect(list_s, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return 1;
    } 



	return 1;
}

/*  Read a line from a socket  */

int Read_msg_socket(char * data, unsigned int size) {

	int n;
	n = read(list_s, data, size);
    return n;

}


/*  Write a line to a socket  */

ssize_t Write_msg_socket(char * data, unsigned int size) {
    size_t      nleft;
    ssize_t     nwritten;
	nleft  = size;
	nwritten =0;
	
    while ( nleft > 0 ) {
	if ( (nwritten = write(list_s, data, nleft)) <= 0 ) {
	    if ( errno == EINTR )
		nwritten = 0;
	    else
		return -1;
	}
	nleft  -= nwritten;
	data += nwritten;
    }

    return nwritten;
	
}

void video_init(void) {

	//init and start the GST framework
	//for now this is being done by the makefile.omap from ppz center upload button
	//the following code does not work properly:
	//	int status = system("/data/video/kevin/initvideoall.sh");
	//as it waits until script is done (which never happens)
	//-> no init is needed, framework is started automatically

	//init the socket
	initSocket();
}


void video_receive(void) {
  

	//read the data from the video tcp socket
	
	if (Read_msg_socket((char *) &gst2ppz,sizeof(gst2ppz))>=0) {
		printf("Received data. Maxy: %d, counter: %d\n",gst2ppz.maxY,gst2ppz.counter);
		video_impl.maxY = gst2ppz.maxY;
		video_impl.max_idx = gst2ppz.max_idx;
		video_impl.max_idy = gst2ppz.max_idy;
		video_impl.counter = gst2ppz.counter;
	}

//testing

	electrical.vsupply = video_impl.max_idx; // for testing!!!
	electrical.current = video_impl.max_idy; // for testing!!!
	ppz2gst.heading = gst2ppz.counter;		// testing!
	Write_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));

}


