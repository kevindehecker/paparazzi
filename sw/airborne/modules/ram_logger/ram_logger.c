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
 * Logs data directly into RAM mem.
 */

#include "modules/ram_logger/ram_logger.h"
#define MAXBUFFERSIZE 32767
#include "std.h"
//#include "subsystems/datalink/telemetry.h"
#include "led.h"
#include "led_hw.h"
#include "autopilot.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/uart_arch.h"

#define COM_PORT   (&uart3.device)

static unsigned char data1[MAXBUFFERSIZE];
int count =0;
bool started;
bool circular_buffer_mode = false; // TODO: make this a setting;

void ram_logger_init(void){
    for (int i = 0; i< MAXBUFFERSIZE; i++)
        data1[0] = 0;
    started = false;
}


void ram_logger_start(void) {
    count = 0;

    started = true;
}
void ram_logger_stop(void) {
    started = false;
}

bool sentnow = false;
void ram_logger_periodic(void){
//    if (started) {
//        data1[count] = count; // tmp test
//        count +=1;
//        if (count > MAXBUFFERSIZE) {
//            if (circular_buffer_mode)
//                count = 0;
//            else
//                started = false;
//        }
//    }

    static int divider = 0;
    if (divider++ % 100 == 0) {
        sentnow = true;

    }
}


void ram_logger_event(void) {

    if (sentnow) {
        sentnow = false;
        //LED_TOGGLE(SYS_TIME_LED);
        COM_PORT->put_byte(COM_PORT->periph, 0, 0x99);
        COM_PORT->put_byte(COM_PORT->periph, 0, 7);
        COM_PORT->put_byte(COM_PORT->periph, 0, 0);
        COM_PORT->put_byte(COM_PORT->periph, 0, 0);

        COM_PORT->put_byte(COM_PORT->periph, 0, 66);

        COM_PORT->put_byte(COM_PORT->periph, 0, 66+7);
        COM_PORT->put_byte(COM_PORT->periph, 0, 66+7);

//        for (int i = 0; i < 255; i++) {
//          COM_PORT->put_byte(COM_PORT->periph, 0, i);
//        }
//          COM_PORT->put_byte(COM_PORT->periph, 0, '\n');
//          COM_PORT->put_byte(COM_PORT->periph, 0, '\0');
//          COM_PORT->put_byte(COM_PORT->periph, 0, 'O');
//          COM_PORT->put_byte(COM_PORT->periph, 0, 'E');
//          COM_PORT->put_byte(COM_PORT->periph, 0, 'R');
//          COM_PORT->put_byte(COM_PORT->periph, 0, '!');
//          COM_PORT->put_byte(COM_PORT->periph, 0, '\n');
//uart3.device.put_byte(&uart3.device,0,100);
          //COM_PORT->put_buffer(COM_PORT->periph,0,data1,127);
          //COM_PORT->send_message(COM_PORT->periph,0);

          //autopilot_send_version();
          //DOWNLINK_SEND_PAYLOAD(DefaultChannel, uart3, 127, data1);
//        }
    }


}
