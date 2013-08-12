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


struct VideoARDrone video_impl;

/*  Global constants  */

#define MAX_LINE           (1000)

/*  Global variables  */
int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */


int closeSocket(void) {
	return close(conn_s);
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

ssize_t Readline_socket(void *vptr, size_t maxlen) {

	int n;

	while ( (n = read(list_s, buffer, sizeof(buffer)-1)) > 0)
    {
		buffer[n] = 0;
		if(fputs(buffer, stdout) == EOF)
		{
			printf("\n Error : Fputs error\n");
		}
	}

	buffer[n+1] = 0;
	printf ("Result: %s",buffer);

	return 1; // TODO, change
}





void video_init(void) {

	//init and start the GST framework
	//	int status = system("/data/video/kevin/initvideoall.sh");

	//init the socket
	initSocket();
}


void video_receive(void) {
  

	char * tmp;

	//read the data from the video tcp socket
	video_impl.maxY = 666;	
	//printf("test out!");
	Readline_socket(tmp, 100);


}


