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
 * @file "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.c"
 * @author Kevin van Hecke
 * Puts Spektrum in binding mode through software
 */

#include "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.h"
#include "subsystems/intermcu/intermcu_fbw.h"
#include "mcu.h"
#include "subsystems/radio_control.h"
#include "mcu_periph/sys_time_arch.h"

#include "led.h"
#include "mcu_periph/gpio.h"

void spektrum_soft_bind_init(void) {
    gpio_setup_output(SPEKTRUM_POWER_PIN_PORT, SPEKTRUM_POWER_PIN);
    gpio_set(SPEKTRUM_POWER_PIN_PORT, SPEKTRUM_POWER_PIN);
    //radio_control_impl_init();


}

void received_spektrum_soft_bind(void) {

    //power cycle the spektrum
    LED_OFF(1);    
    gpio_clear(SPEKTRUM_POWER_PIN_PORT, SPEKTRUM_POWER_PIN);
    sys_time_usleep(100000);
    gpio_set(SPEKTRUM_POWER_PIN_PORT, SPEKTRUM_POWER_PIN);
    LED_ON(1);

    //put to bind mode
    RADIO_CONTROL_BIND_IMPL_FUNC();    //basically  = radio_control_spektrum_try_bind()

    SpektrumUartInit();

}
