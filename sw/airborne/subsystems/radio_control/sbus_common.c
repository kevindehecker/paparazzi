/*
 * Copyright (C) 2013 Alexandre Bustico, Gautier Hattenberger
 *
 * This file is part of paparazzi.
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

/** @file subsystems/radio_control/sbus_common.c
 *
 * Futaba SBUS decoder
 */

#include "subsystems/radio_control.h"
#include "subsystems/radio_control/sbus_common.h"
#include BOARD_CONFIG
#include "mcu_periph/gpio.h"
#include <string.h>

/*
 * SBUS protocol and state machine status
 */
#define SBUS_START_BYTE 0x0f
#define SBUS_BIT_PER_CHANNEL 11
#define SBUS_BIT_PER_BYTE 8
#define SBUS_FLAGS_BYTE 23
#define SBUS_FRAME_LOST_BIT 2
#define SBUS_FAILSAFE_BIT	3

#define SBUS_STATUS_UNINIT      0
#define SBUS_STATUS_GOT_START   1

/** Set polarity using RC_POLARITY_GPIO.
 * SBUS signal has a reversed polarity compared to normal UART
 * this allows to using hardware UART peripheral by changing
 * the input signal polarity.
 * Setting this gpio ouput high inverts the signal,
 * output low sets it to normal polarity.
 */
#ifndef RC_SET_POLARITY
#define RC_SET_POLARITY gpio_set
#endif



/*
 * S.bus decoder matrix. (extracted from PX4)
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
  uint8_t byte;
  uint8_t rshift;
  uint8_t mask;
  uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_NB_CHANNEL][3] = {
  /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
  /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
  /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
  /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
  /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
  /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 0000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

static bool sbus_decode(uint8_t *frame, uint16_t *values, uint16_t *num_values,
      bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{

  if (!(frame[24] == 0x00 || frame[24] == 0x04 || frame[24] == 0x14 || frame[24] == 0x24 || frame[24] == 0x34))
    return false;

  /* we have received something we think is a frame */

  unsigned chancount = (max_values > SBUS_NB_CHANNEL) ?
           SBUS_NB_CHANNEL : max_values;

  /* use the decoder matrix to extract channel data */
  for (unsigned channel = 0; channel < chancount; channel++) {
    unsigned value = 0;

    for (unsigned pick = 0; pick < 3; pick++) {
      const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

      if (decode->mask != 0) {
        unsigned piece = frame[1 + decode->byte];
        piece >>= decode->rshift;
        piece &= decode->mask;
        piece <<= decode->lshift;

        value |= piece;
      }
    }

    /* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
    values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

  }

  /* decode switch channels if data fields are wide enough */
  if (max_values > 17 && chancount > 15) {
    chancount = 18;

    /* channel 17 (index 16) */
    values[16] = (((frame[SBUS_FLAGS_BYTE] & (1 << 0)) > 0) ? 1 : 0) * 1000 + 998;
    /* channel 18 (index 17) */
    values[17] = (((frame[SBUS_FLAGS_BYTE] & (1 << 1)) > 0) ? 1 : 0) * 1000 + 998;
  }

  /* note the number of channels decoded */
  *num_values = chancount;

  /* decode and handle failsafe and frame-lost flags */
  if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
    /* report that we failed to read anything valid off the receiver */
    *sbus_failsafe = true;
    *sbus_frame_drop = true;
     return false;
  } else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAME_LOST_BIT)) { /* a frame was lost */
    /* set a special warning flag
     *
     * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
     * condition as fail-safe greatly reduces the reliability and range of the radio link,
     * e.g. by prematurely issuing return-to-launch!!! */

    *sbus_failsafe = false;
    *sbus_frame_drop = true;
  } else {
    *sbus_failsafe = false;
    *sbus_frame_drop = false;
  }

  return true;
}

void sbus_common_init(struct Sbus *sbus_p, struct uart_periph *dev)
{
  sbus_p->frame_available = false;
  sbus_p->status = SBUS_STATUS_UNINIT;

  // Set UART parameters (100K, 8 bits, 2 stops, even parity)
  uart_periph_set_baudrate(dev, B100000);
  uart_periph_set_bits_stop_parity(dev, UBITS_8, USTOP_2, UPARITY_EVEN);

  // Set polarity
#ifdef RC_POLARITY_GPIO_PORT
  gpio_setup_output(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
  RC_SET_POLARITY(RC_POLARITY_GPIO_PORT, RC_POLARITY_GPIO_PIN);
#endif

}

// Decoding event function
// Reading from UART
void sbus_common_decode_event(struct Sbus *sbus_p, struct uart_periph *dev)
{
  uint8_t rbyte;
  if (uart_char_available(dev)) {    
    do {
      rbyte = uart_getch(dev);
      switch (sbus_p->status) {
        case SBUS_STATUS_UNINIT:
          // Wait for the start byte
          if (rbyte == SBUS_START_BYTE) {
            sbus_p->status++;            
            sbus_p->idx = 0;
            sbus_p->buffer[sbus_p->idx] = rbyte;
            sbus_p->idx++;
          }
          break;
        case SBUS_STATUS_GOT_START:
          // Store buffer        
          sbus_p->buffer[sbus_p->idx] = rbyte;
          sbus_p->idx++;
          if (sbus_p->idx == SBUS_BUF_LENGTH ) {            
            bool sbus_failsafe, sbus_frame_drop;
            uint16_t num_values;
            sbus_p->frame_available = sbus_decode(sbus_p->buffer, sbus_p->pulses,&num_values,&sbus_failsafe,&sbus_frame_drop,18);
            sbus_p->status = SBUS_STATUS_UNINIT;
          }
          break;
        default:
          break;
      }
    } while (uart_char_available(dev));
  }
}
