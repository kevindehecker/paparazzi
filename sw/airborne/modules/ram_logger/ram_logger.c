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
 * Logs data directly into RAM mem and sends it over the datalink in a special package.
 */

#include "modules/ram_logger/ram_logger.h"
#include "std.h"
#include "subsystems/datalink/telemetry.h"
#include "led_hw.h"
#include "subsystems/abi.h"

int ram_logger_enable_logging;
int ram_logger_download_log;

int disable_telemetry_delay = -1; // to re-enable normal telemetry
int log_packge_delay = 0; // log_packge_delay in between log packages
int send_id = 0; // send data over datalink counter pointer

#define CCMRAM __attribute__((section(".ram4")))

#define MAXBUFFERSIZE 4096
unsigned char data1[MAXBUFFERSIZE];
CCMRAM unsigned char data2[MAXBUFFERSIZE];
#define TOTALBUFFERSIZE 8192 // must be 2 * MAXBUFFERSIZE
#define ESPBUFFERSIZE 2048
struct RAM_log_data {
    int32_t accx;
    int32_t accy;
    int32_t accz;
    int32_t gyrop;
    int32_t gyroq;
    int32_t gyror;
} __attribute__((__packed__));

static abi_event gyro_ev;
static abi_event accel_ev;

#define COM_PORT (&DOWNLINK_DEVICE.device)
int entry_id1 = 0;
int entry_id2 = 0;


static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t  __attribute__((unused)) stamp, struct Int32Rates *gyro)
{

    struct RAM_log_data * tmp;
    if ((entry_id1 +1 )* sizeof(struct RAM_log_data) < MAXBUFFERSIZE) {
        tmp = (struct RAM_log_data * ) data1;
        tmp[entry_id1].gyrop = gyro->p;
        tmp[entry_id1].gyroq = gyro->q;
        tmp[entry_id1].gyror = gyro->r;
    } else {
        tmp = (struct RAM_log_data * ) data2;
        tmp[entry_id2].gyrop = gyro->p;
        tmp[entry_id2].gyroq = gyro->q;
        tmp[entry_id2].gyror = gyro->r;
    }
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
    struct RAM_log_data * tmp;
    if ((entry_id1 +1 ) * sizeof(struct RAM_log_data) < MAXBUFFERSIZE) {
        tmp = (struct RAM_log_data * ) data1;
        tmp[entry_id1].accx = accel->x;
        tmp[entry_id1].accy = accel->y;
        tmp[entry_id1].accz = accel->z;
    } else {
        tmp = (struct RAM_log_data * ) data2;
        tmp[entry_id2].accx = accel->x;
        tmp[entry_id2].accy = accel->y;
        tmp[entry_id2].accz = accel->z;
    }

    if ((entry_id1 +1 ) * sizeof(struct RAM_log_data) < MAXBUFFERSIZE)
        entry_id1++;
    else if ((entry_id2 +2 ) * sizeof(struct RAM_log_data) < MAXBUFFERSIZE) //stop after +2 because in the next pass that will be +1 (which just fits)
        entry_id2++;
    else
        LED_TOGGLE(1);
}

void ram_logger_init(void){    
    AbiBindMsgIMU_GYRO_INT32(ABI_BROADCAST, &gyro_ev, gyro_cb);
    AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &accel_ev, accel_cb);

    //init with some test data:
    for (int i = 0; i< MAXBUFFERSIZE; i++) {
        data1[i] = 66;
        data2[i] = 67;
        if (i % ESPBUFFERSIZE == 0) {
            data1[i] = '0' +  i / ESPBUFFERSIZE;
            data2[i] = '0' +  i / ESPBUFFERSIZE;
            if (i>0) {
                data1[i-1] = '\n';
                data2[i-1] = '\n';
            }
        }

    }
    data1[MAXBUFFERSIZE-1] = '\n';
    data2[MAXBUFFERSIZE-1] = '\n';
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
        if (disable_datalink && send_id < TOTALBUFFERSIZE) {
            if (send_id % ESPBUFFERSIZE == 0) {
                COM_PORT->put_byte(COM_PORT->periph, 0, 'l');
                COM_PORT->put_byte(COM_PORT->periph, 0, 'o');
                COM_PORT->put_byte(COM_PORT->periph, 0, 'g');
                COM_PORT->put_byte(COM_PORT->periph, 0, '_');
                COM_PORT->put_byte(COM_PORT->periph, 0, 's');
                COM_PORT->put_byte(COM_PORT->periph, 0, 't');
            }
            if (send_id < MAXBUFFERSIZE)
                COM_PORT->put_byte(COM_PORT->periph, 0, data1[send_id]);
            else
                COM_PORT->put_byte(COM_PORT->periph, 0, data2[send_id-MAXBUFFERSIZE]);
            send_id++;
        }
        if (send_id >= TOTALBUFFERSIZE && disable_telemetry_delay < 0) {
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
extern void ram_logger_logging_handle(__attribute__((unused)) int enable) {
    LED_OFF(1);
    entry_id1 = 0;
    entry_id2 = 0;
//    float tmp[10];
//    tmp[0] = entry_id1;
//    tmp[1] = entry_id1;
//    tmp[2] = sizeof(struct RAM_log_data);
//    DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 3,tmp);
}

