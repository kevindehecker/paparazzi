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
 * @file "modules/pxio_flash/pxio_flash.c"
 * @author Kevin van Hecke
 * Flashes the px4io f1 through the px4 bootloader.
 */

#include "modules/pxio_flash/pxio_flash.h"
//#include "subsystems/datalink/downlink.h"
#include "modules/pxio_flash/protocol.h"
 #include "mcu_periph/sys_time_arch.h"

 // Serial Port
#include "mcu_periph/uart.h"
 PRINT_CONFIG_VAR(PXIO_UART)

// define coms link for px4io f1
#define PXIO_PORT   (&((PXIO_UART).device))
#define TELEM2_PORT   (&((TELEM2_UART).device))

 bool_t setToBootloaderMode;

 void px4ioflash_init(void) {
  setToBootloaderMode = FALSE;
}

void px4ioflash_event(void) {

  if (PXIO_PORT->char_available(PXIO_PORT->periph)) {
    if (!setToBootloaderMode) {
      unsigned char hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);

      if (hoer == PKT_CODE_SUCCESS) {

        sys_time_usleep(50000);
        PXIO_PORT->put_byte(PXIO_PORT->periph,0x21);
        PXIO_PORT->put_byte(PXIO_PORT->periph,0x20);

        unsigned char count = 48;
        while (PXIO_PORT->char_available(PXIO_PORT->periph)) {
          hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);
          char tmp[3];
          itoa(hoer,tmp,16);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,count++);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,':');
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,' ');
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[0]);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[1]);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
        }





        sys_time_usleep(50000);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
        count = 48;
        while (PXIO_PORT->char_available(PXIO_PORT->periph)) {
          hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);
          char tmp[3];
          itoa(hoer,tmp,16);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,count++);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,':');
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,' ');
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[0]);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[1]);
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
          TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
        }




        setToBootloaderMode = true;
      }
    } else {
     unsigned char hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);
     TELEM2_PORT->put_byte(TELEM2_PORT->periph,hoer);
   }
 }

 if (TELEM2_PORT->char_available(TELEM2_PORT->periph) && !setToBootloaderMode) {
//setToBootloaderMode = true; // tmp
  static struct IOPacket  dma_packet;

  dma_packet.count_code = 0x40 + 0x01;
  dma_packet.crc = 0;
  dma_packet.page = PX4IO_PAGE_SETUP;
  dma_packet.offset = PX4IO_P_SETUP_REBOOT_BL;
  dma_packet.regs[0] = PX4IO_REBOOT_BL_MAGIC;

  dma_packet.crc = crc_packet(&dma_packet);

  struct IOPacket *pkt = &dma_packet;

  uint8_t *p = (uint8_t *)pkt;

  PXIO_PORT->put_byte(PXIO_PORT->periph,p[0]);
  PXIO_PORT->put_byte(PXIO_PORT->periph,p[1]);
  PXIO_PORT->put_byte(PXIO_PORT->periph,p[2]);
  PXIO_PORT->put_byte(PXIO_PORT->periph,p[3]);
  PXIO_PORT->put_byte(PXIO_PORT->periph,p[4]);
  PXIO_PORT->put_byte(PXIO_PORT->periph,p[5]);

  sys_time_usleep(5000); // this seems to be close to the minimum delay necessary to handle this packet
  //the pixhawk IO chip should respond with:
  // 0x00 ( PKT_CODE_SUCCESS )
  // 0xe5
  // 0x32
  // 0x0a
  //After that, the IO chips reboots into bootloader mode, in which it will stay for a short period
  //The baudrate in bootloader mode ic changed to 115200 (normal operating baud is 1500000)






  unsigned char hoer;
  while (PXIO_PORT->char_available(PXIO_PORT->periph)) {
    hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);
    if (hoer == PKT_CODE_SUCCESS) {

      while (PXIO_PORT->char_available(PXIO_PORT->periph)) {
        hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);
        char tmp[3];
        itoa(hoer,tmp,16);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'u');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'c');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'c');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'e');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'s');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'s');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,':');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[0]);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[1]);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
      }
    } else{
      TELEM2_PORT->put_byte(TELEM2_PORT->periph,'E');
      TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
      TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
      TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
      TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
    }

  }

  uart_periph_set_baudrate(PXIO_PORT->periph,B115200);


  int ret = 1;
  /* look for the bootloader for 150 ms */
  for (int i = 0; i < 15; i++) {

    unsigned char count = 48;
    //while (PXIO_PORT->char_available(PXIO_PORT->periph)) {
    for (int j = 0 ; j<10; j++) {
      if (PXIO_PORT->char_available(PXIO_PORT->periph)) {
        hoer = PXIO_PORT->get_byte(PXIO_PORT->periph);
        char tmp[3];
        itoa(hoer,tmp,16);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,i+48);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'-');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,count++);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,':');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,' ');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[0]);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[1]);
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
        TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
      }
    }


    PXIO_PORT->put_byte(PXIO_PORT->periph,0x21);
    PXIO_PORT->put_byte(PXIO_PORT->periph,0x20);

    if (ret==0) {
      setToBootloaderMode= true;
      break;

    } else {
      sys_time_usleep(10000);
    }
  }

  TELEM2_PORT->get_byte(TELEM2_PORT->periph); // tmp ignore input keyboard!

  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'e');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'n');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'t');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,' ');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'e');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'b');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'o');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'o');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'t');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');


} else if(TELEM2_PORT->char_available(TELEM2_PORT->periph)) {
 unsigned char hoer = TELEM2_PORT->get_byte(TELEM2_PORT->periph);
 PXIO_PORT->put_byte(PXIO_PORT->periph,hoer);
 TELEM2_PORT->put_byte(TELEM2_PORT->periph,hoer);
}

}

