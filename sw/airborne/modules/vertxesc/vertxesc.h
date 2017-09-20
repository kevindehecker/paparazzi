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
 * @file "modules/vertxesc/vertxesc.h"
 * @author Kevin van Hecke
 * Driver for the vert-x vtol escs
 */
#include "std.h"

#ifndef VERTXESC_H
#define VERTXESC_H

extern void vertx_esc_init(void);
extern void vertx_esc_periodic(void);
extern void vertx_esc_event(void);

#define ESCS_START_BYTE 0xFE
#define ESCS_DATA_FLIPBIT 16384
#define ESCS_DATA_MYSTERYBIT 32768

struct EscData {
    unsigned char start; //0xfe
    unsigned char len; //8
    unsigned char id; //2

    //1200 - 1800, maybe 1100-1900
    //1160 = off, max ~1880
    //in beide data ints, 2e byte toggled de 64 bit  (15e bit in totaal). Als ie aan staat in de een, uit in de ander
    //in d2 altijd 16e bit aan

    uint32_t d1 ;
    uint32_t d2 ;
    unsigned char crc;
}__attribute__((__packed__));


#endif

