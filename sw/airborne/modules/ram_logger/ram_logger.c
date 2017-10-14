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
#include "mcu_periph/uart.h"
#include "mcu_periph/uart_arch.h"
#include "datalink/pprz_dl.h"

int ram_logger_enable_logging;
int ram_logger_download_log;
int ram_logger_stop_telemetry;

unsigned char data1[MAXBUFFERSIZE];

#define COM_PORT   (&DOWNLINK_DEVICE.device)

void ram_logger_init(void){    
    data1[0] = 'l';
    data1[1] = 'o';
    data1[2] = 'g';
    data1[3] = '_';
    data1[4] = 's';
    data1[5] = 't';

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
int downcount = -1;
int wait = false;
void ram_logger_periodic(void){
//    static int divider = 0;
//    if (divider++ % 1000 == 0) {
//        sentnow = true;
//    }
    if (downcount > -1)
        downcount++;
    if (downcount > 50) {
//        USART_CR1((uint32_t)p->reg_addr) |= USART_CR1_TXEIE; // Enable TX interrupt
        disable_datalink = false;
        telemetry_mode_Main = 0;
        downcount = -1;
//        LED_ON(1);

    }
    //telemetry_mode_Main = 0;

    if (wait>0)
        wait --;
}


int cnt = 0;
void ram_logger_event(void) {
    if (wait==0) {
        if (disable_datalink && cnt < MAXBUFFERSIZE) {
            if (cnt % ESPBUFFERSIZE == 0) {
                COM_PORT->put_byte(COM_PORT->periph, 0, 'l');
                COM_PORT->put_byte(COM_PORT->periph, 0, 'o');
                COM_PORT->put_byte(COM_PORT->periph, 0, 'g');
                COM_PORT->put_byte(COM_PORT->periph, 0, '_');
                COM_PORT->put_byte(COM_PORT->periph, 0, 's');
                COM_PORT->put_byte(COM_PORT->periph, 0, 't');
                //data1[cnt] = '1' +  cnt / ESPBUFFERSIZE;
            }
            COM_PORT->put_byte(COM_PORT->periph, 0, data1[cnt]);
            cnt++;
        }
        if (cnt >= MAXBUFFERSIZE && downcount < 0) {
            downcount = 0;
        }
        if (cnt % ESPBUFFERSIZE == 0)
            wait = 200;
    }

}

void ram_logger_download_handle(int enable) {
    LED_OFF(1);
    if (enable) {
        cnt = 0;
        disable_datalink = true;
        downcount = -1;
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
