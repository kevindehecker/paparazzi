/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/ramlogger/ramlogger.c"
 * @author Kevin van Hecke
 * Logs data directly into RAM mem. Sends it to GCS through a message.
 */

#include "modules/ram_logger/ram_logger.h"
#define MAXBUFFERSIZE 16383
#include "std.h"
#include "subsystems/datalink/telemetry.h"
#include "led.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/uart_arch.h"
#include "datalink/pprz_dl.h"


int ram_logger_enable_logging;
int ram_logger_download_log;
int ram_logger_stop_telemetry;

unsigned char data1[MAXBUFFERSIZE];
int count =0;
int sentCounter = -1;
int interval = 0;
//bool started;
//bool circular_buffer_mode = false; // TODO: make this a setting;
//bool sentnow = false;

#define COM_PORT   (&DOWNLINK_DEVICE.device)

void ram_logger_init(void){
    for (int i = 0; i< MAXBUFFERSIZE; i++)
        data1[0] = i;
  //  started = false;
}

void ram_logger_start(void) {
//    count = 0;
//    sentCounter = 0;
//    started = true;

}
void ram_logger_stop(void) {
//    started = false;
//    sentCounter = 0;

}
void ram_logger_periodic(void){
//    static int divider = 0;
//    if (divider++ % 1000 == 0) {
//        sentnow = true;
//    }
}

void ram_logger_event(void) {
//    if (sentnow) {
//        sentnow = false;


//    }
}

void ram_logger_download_handle(int enable) {
    if (enable) {
        for (int i = 0; i < MAXBUFFERSIZE; i++) {
            COM_PORT->put_byte(COM_PORT->periph, 0, data1[i]);
        }
    }
}
extern void ram_logger_logging_handle(int enable) {
    for (int i = 0; i < MAXBUFFERSIZE; i++) {
        data1[i] = i;
    }
}
extern void ram_logger_telemetry_handle(int enable){
    LED_TOGGLE(SYS_TIME_LED);
    if (enable) {
        disable_datalink = true;
        telemetry_mode_Main = 255 ; //disable all downlink telemetry
    } else {
        disable_datalink = false;
        telemetry_mode_Main = 0 ; //disable all downlink telemetry
    }
}
