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
 * @file subsystems/video/video_ardrone2.h
 * Video implementation for ardrone2.
 *
 * Use the tcp output of a custom GStreamer framework plugin to receive 
 * telemetry based on video
 */

#ifndef VIDEO_ARDRONE2_H
#define VIDEO_ARDRONE2_H

#include "std.h"
#include "subsystems/video.h"


struct VideoARDrone {
  uint32_t maxY;             // test output variable
};
extern struct VideoARDrone video_impl;



#include <unistd.h>             /*  for ssize_t data type  */

#define LISTENQ        (1024)   /*  Backlog for listen()   */
#define PORT	       (2002)  


/*  private function declarations  */
	int initSocket(void) ;
	ssize_t Readline_socket(void *vptr, size_t maxlen);
	ssize_t Writeline_socket(char * text, size_t n);
	int closeSocket(void);


#endif /* VIDEO_ARDRONE2_H */
