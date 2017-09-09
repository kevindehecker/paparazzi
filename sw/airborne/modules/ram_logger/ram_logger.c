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
#define MAXBUFFERSIZE 32767
#include "std.h"
#include "subsystems/datalink/telemetry.h"

static unsigned char data1[MAXBUFFERSIZE];
int count =0;
int sentCounter = -1;
int interval = 0;
bool started;
bool circular_buffer_mode = false; // TODO: make this a setting;

void ram_logger_init(void){
    for (int i = 0; i< MAXBUFFERSIZE; i++)
        data1[0] = 0;
    started = false;
}


void ram_logger_start(void) {
    count = 0;
    sentCounter = 0;
    started = true;
}
void ram_logger_stop(void) {
    started = false;
    sentCounter = 0;

}
void ram_logger_periodic(void){
    if (started) {
        data1[count] = count; // tmp test
        count +=1;
        if (count > MAXBUFFERSIZE) {
            if (circular_buffer_mode)
                count = 0;
            else
                started = false;
        }
    }
    interval++;
}


void ram_logger_event(void) {

    if (sentCounter >= 0 && interval > 200) {
        uint16_t l = 127;
        interval = 0;
        unsigned char * ptr = data1;
        ptr += sentCounter;
        *ptr = sentCounter;
        DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, l, ptr);
        sentCounter +=l;
        if (sentCounter  >= count-l)
            sentCounter =-1; // stop sending when all data was sent
    }
}
