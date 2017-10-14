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
#define MAXBUFFERSIZE 65536
#define ESPBUFFERSIZE 2048
#include "std.h"
#include "subsystems/datalink/telemetry.h"
#include "led_hw.h"

int ram_logger_enable_logging;
int ram_logger_download_log;
int ram_logger_stop_telemetry;

int disable_telemetry_delay = -1; // to re-enable normal telemetry
int log_packge_delay = 0; // log_packge_delay in between log packages
int send_id = 0; // send data over datalink counter pointer
unsigned char data1[MAXBUFFERSIZE];

#define COM_PORT   (&DOWNLINK_DEVICE.device)

void ram_logger_init(void){    
    data1[0] = 'l';
    data1[1] = 'o';
    data1[2] = 'g';
    data1[3] = '_';
    data1[4] = 's';
    data1[5] = 't';

    //init with some test data:
    for (int i = 0; i< MAXBUFFERSIZE; i++) {
        data1[i] = 67;
        if (i % ESPBUFFERSIZE == 0) {
            data1[i] = '1' +  i / ESPBUFFERSIZE;
            if (i>0)
                data1[i-1] = '\n';
        }
    }
    data1[MAXBUFFERSIZE-1] = '\n';

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
    if (disable_telemetry_delay > -1)
        disable_telemetry_delay++;
    if (disable_telemetry_delay > 50) {
        disable_datalink = false;
        telemetry_mode_Main = 0;
        disable_telemetry_delay = -1;
    }

    if (log_packge_delay>0)
        log_packge_delay --;
}



void ram_logger_event(void) {
    if (log_packge_delay==0) {
        if (disable_datalink && send_id < MAXBUFFERSIZE) {
            if (send_id % ESPBUFFERSIZE == 0) {
                COM_PORT->put_byte(COM_PORT->periph, 0, 'l');
                COM_PORT->put_byte(COM_PORT->periph, 0, 'o');
                COM_PORT->put_byte(COM_PORT->periph, 0, 'g');
                COM_PORT->put_byte(COM_PORT->periph, 0, '_');
                COM_PORT->put_byte(COM_PORT->periph, 0, 's');
                COM_PORT->put_byte(COM_PORT->periph, 0, 't');
                //data1[cnt] = '1' +  cnt / ESPBUFFERSIZE;
            }
            COM_PORT->put_byte(COM_PORT->periph, 0, data1[send_id]);
            send_id++;
        }
        if (send_id >= MAXBUFFERSIZE && disable_telemetry_delay < 0) {
            disable_telemetry_delay = 0;
        }
        if (send_id % ESPBUFFERSIZE == 0)
            log_packge_delay = 200;
    }

}

void ram_logger_download_handle(int enable) {
    LED_OFF(1);
    if (enable) {
        send_id = 0;
        disable_datalink = true;
        disable_telemetry_delay = -1;
        telemetry_mode_Main = 255;
    }
}
extern void ram_logger_logging_handle(int enable) {
  //  for (int i = 0; i < MAXBUFFERSIZE; i++) {
  //      data1[i] = i;
  //  }
}
extern void ram_logger_telemetry_handle(int enable){
   // LED_TOGGLE(SYS_TIME_LED);
    if (enable) {
        //disable_datalink = true;
        //telemetry_mode_Main = 255 ; //disable all downlink telemetry
    } else {
        //disable_datalink = false;
        //telemetry_mode_Main = 0 ; //disable all downlink telemetry
    }
}
