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
#define MAXBUFFERSIZE 1024
#include "std.h"
#include "subsystems/datalink/telemetry.h"
#include "led_hw.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/uart_arch.h"
#include "datalink/pprz_dl.h"




#include "mcu_periph/uart_arch.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/gpio.h"

//#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/usart.h>
//#include <libopencm3/cm3/nvic.h>

#include "std.h"

#include BOARD_CONFIG




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

    for (int i = 6; i< MAXBUFFERSIZE; i++)
        data1[i] = 67;

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
}


int cnt = 0;
void ram_logger_event(void) {
    if (disable_datalink && cnt < MAXBUFFERSIZE) {
        //struct uart_periph *p;
        //p = COM_PORT->periph;
        //usart_send_blocking((uint32_t)p->reg_addr, 'A' + (cnt % 25));
        COM_PORT->put_byte(COM_PORT->periph, 0, data1[cnt]);
        cnt++;
    }
    if (cnt >= MAXBUFFERSIZE && downcount < 0) {
      downcount = 0;
    }
}

void ram_logger_download_handle(int enable) {
    LED_OFF(1);
    if (enable) {

//        struct uart_periph *p;
//        p = COM_PORT->periph;
        //USART_CR1((uint32_t)p->reg_addr) &= ~USART_CR1_TXEIE; // Disable TX interrupt
//        usart_send_blocking((uint32_t)p->reg_addr, data1[0]);
//        p->tx_insert_idx = 0;
//        p->tx_extract_idx = 0;
        cnt = 0;
        //for (cnt = 0; cnt < 6; cnt++) {
            //p->tx_insert_idx = i;
            //p->tx_buf[p->tx_insert_idx] = data1[i];

//            usart_send_blocking((uint32_t)p->reg_addr, data1[i]);
          //  COM_PORT->put_byte(COM_PORT->periph, 0, data1[cnt]);


        //}
//        COM_PORT->put_buffer(COM_PORT->periph, 0, data1,300);
        //p->tx_running = true;

        disable_datalink = true;
        downcount = -1;
        telemetry_mode_Main = 255;

        //USART_CR1((uint32_t)p->reg_addr) |= USART_CR1_TXEIE; // Enable TX interrupt
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
