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
    int frameID;
    int avgdisp_est;
    int avgdisp_est_thresh;
    int ROCchoice; // 0 for stereo, 1 for estimator. ROC choice
    float fps;
    char endl;             // endl fix, makes it worker nicer in terminal for debugging :)
};
extern struct ICDataPackage video_impl;

/***********Enums****************/
enum learnModus_t {learn_none, learn_stereo_only, learn_textons_only, learn_stereo_textons, learn_stereo_textons_active};
enum exploreModus_t {explore_on_stereo, explore_on_mono, explore_on_ROC};

extern int32_t IC_threshold_gt;
extern int32_t IC_threshold_gt_msg;
extern int32_t IC_threshold_gtstd;
extern int32_t IC_threshold_est;
extern bool IC_turnbutton;
extern int8_t IC_flymode;
extern int8_t IC_learnmode;
extern int8_t IC_actionDummy;

extern float navHeading;


extern bool obstacle_detected;
extern bool rh_reached;

extern bool init_nav_heading(void);
extern bool increase_nav_heading( float increment);
extern bool increase_nav_waypoint(int wp_id_current,int wp_id_goal, float distance);
//extern bool goBackaBit(int wp_id_current,int wp_id_prevgoal);

extern bool set_rand_heading(void);
extern bool increase_nav_heading_till_r(float increment);

extern int noDataCounter;

#define PORT	6969

extern void IC_start(void);
extern void IC_stop(void);
extern void IC_periodic(void);

extern void IC_slave_FlyModeButton(int8_t value);
extern void IC_slave_LearnModeButton(int8_t value);
extern void IC_slave_ActionButton(int8_t value);

#endif /* IC_SLAVE_H_ */

