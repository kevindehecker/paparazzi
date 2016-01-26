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
 * Assumes the telem2 port on the Pixhawk is connected to a ttyACM device (blackmagic probe)
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

// weird that these below are not in protocol.h, which is from the firmware px4 repo
//below is copied from qgroundcontrol:
#define PROTO_INSYNC            0x12   ///< 'in sync' byte sent before status
#define PROTO_EOC               0x20   ///< end of command
// Reply bytes
#define PROTO_OK                0x10   ///< INSYNC/OK      - 'ok' response
#define PROTO_FAILED            0x11   ///< INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID           0x13 ///< INSYNC/INVALID - 'invalid' response for bad commands
// Command bytes
#define PROTO_GET_SYNC          0x21   ///< NOP for re-establishing sync
#define PROTO_GET_DEVICE        0x22   ///< get device ID bytes
#define PROTO_CHIP_ERASE        0x23   ///< erase program area and reset program address
#define PROTO_LOAD_ADDRESS      0x24 ///< set next programming address
#define PROTO_PROG_MULTI        0x27   ///< write bytes at program address and increment
#define PROTO_GET_CRC           0x29 ///< compute & return a CRC
#define PROTO_BOOT              0x30   ///< boot the application

bool_t setToBootloaderMode;

void px4ioflash_init(void) {
    setToBootloaderMode = FALSE;
}

void px4ioflash_event(void) {
    if (PXIO_PORT->char_available(PXIO_PORT->periph)) {
        if (!setToBootloaderMode) {
            //ignore anything coming from IO if not in bootloader mode (which should be nothing)
        } else {
            //relay everything from IO to the laptop
            unsigned char b = PXIO_PORT->get_byte(PXIO_PORT->periph);
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,b);
        }
    }

    if (TELEM2_PORT->char_available(TELEM2_PORT->periph) && !setToBootloaderMode) {
        //data was received on the pc uart, so
        //send the reboot to bootloader command:
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

        sys_time_usleep(5000); // this seems to be close to the minimum delay necessary to process this packet at the IO side
        //the pixhawk IO chip should respond with:
        // 0x00 ( PKT_CODE_SUCCESS )
        // 0xe5
        // 0x32
        // 0x0a
        //After that, the IO chips reboots into bootloader mode, in which it will stay for a short period
        //The baudrate in bootloader mode ic changed to 115200 (normal operating baud is 1500000, at least for original pixhawk fmu firmware)

        //state machine
        int state =0;
        while (state<4 && PXIO_PORT->char_available(PXIO_PORT->periph)) {

            unsigned char b = PXIO_PORT->get_byte(PXIO_PORT->periph);
            switch (state) {
            case (0) :
                if (b == PKT_CODE_SUCCESS) state++; else  state=0;
                break;
            case (1) :
                if (b == 0xe5) state++; else  state=0;
                break;
            case (2) :
                if (b == 0x32) state++; else  state=0;
                break;
            case (3) :
                if (b == 0x0a) state++; else  state=0;
                break;
            default :
                TELEM2_PORT->put_byte(TELEM2_PORT->periph,'b');
                break;
            }
        }

        if (state == 4) {
            uart_periph_set_baudrate(PXIO_PORT->periph,B115200);
            /* look for the bootloader for 150 ms */
            int ret = 0;
            for (int i = 0; i < 15 && !ret ; i++) {
                //unsigned char count = 48;
                sys_time_usleep(10000);


                //send a get_sync command in order to keep the io in bootloader mode
                PXIO_PORT->put_byte(PXIO_PORT->periph,PROTO_GET_SYNC);
                PXIO_PORT->put_byte(PXIO_PORT->periph,PROTO_EOC);

                //get_sync should be replied with, so check if that happens and
                //all other bytes are discarded, hopefully those were not important
                //(they may be caused by sending multiple syncs)
                while (PXIO_PORT->char_available(PXIO_PORT->periph)) {
                    unsigned char b = PXIO_PORT->get_byte(PXIO_PORT->periph);
                    if (b == PROTO_INSYNC) {
                        ret = 1;
                        setToBootloaderMode= true;
                        break;
                    }
                }
            }

            if (setToBootloaderMode) {
                //if successfully entered bootloader mode, clear any remaining bytes (which may have a function, but I did not check)
                while (PXIO_PORT->char_available(PXIO_PORT->periph)) {PXIO_PORT->get_byte(PXIO_PORT->periph);}
            }

        } else {
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'E');
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'o');
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
            TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
        }
    } else if(TELEM2_PORT->char_available(TELEM2_PORT->periph)) {
        //already in bootloader mode, just directly relay data
        unsigned char b = TELEM2_PORT->get_byte(TELEM2_PORT->periph);
        PXIO_PORT->put_byte(PXIO_PORT->periph,b);
    }

}

