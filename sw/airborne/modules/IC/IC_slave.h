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

#ifndef IC_SLAVE_H_
#define IC_SLAVE_H_

#include <unistd.h>             /*  for ssize_t data type  */
#include "std.h"

 
struct ICDataPackage {
  int avgdisp_gt;
  int avgdisp_gt_stdev;
  int avgdisp_nn;
  char endl;             // endl fix :)
};
extern struct ICDataPackage video_impl;

enum flymode{ stereo, textons};

enum learnmode{none,stereo_only,textons_only,stereo_textons};

extern int32_t IC_threshold_gt;
extern int32_t IC_threshold_gtstd;
extern int32_t IC_threshold_nn;
extern bool IC_turnbutton;
extern int8_t IC_flymode;
extern int8_t IC_learnmode;


extern bool obstacle_detected;

extern bool increase_nav_heading(int32_t *heading,int32_t increment);
extern bool increase_nav_waypoint(int wp_id_current,int wp_id_goal, int32_t distance, int32_t heading);
extern bool goBackaBit(int wp_id_current,int wp_id_prevgoal);

extern int noDataCounter;

#define PORT	6969

extern void IC_start(void);
extern void IC_stop(void);
extern void IC_periodic(void);

extern void IC_slave_FlyModeButton(int8_t value);
extern void IC_slave_LearnModeButton(int8_t value);

#endif /* IC_SLAVE_H_ */
 