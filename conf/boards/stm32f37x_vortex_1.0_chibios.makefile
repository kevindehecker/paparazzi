# Hey Emacs, this is a -*- makefile -*-
#
# stm32f37_vortex_chibios.makefile
#
#

BOARD=stm32f37x_vortex
BOARD_VERSION=1.0
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

## FPU on F3
USE_FPU=hard

$(TARGET).CFLAGS += -DSTM32F3 -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F37x/platform.mk
CHIBIOS_BOARD_LINKER = STM32F373xC.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f3xx.mk

##############################################################################
# Compiler settings
#
MCU  = cortex-m4


# default flash mode is via DFU-UTIL
# possibilities: DFU-UTIL, SWD, STLINK
FLASH_MODE ?= SWD_NOPWR

HAS_LUFTBOOT = FALSE

#
# default LED configuration
#
SYS_TIME_LED       ?= 1
RADIO_CONTROL_LED  ?= 2
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 3
GPS_LED            ?= 4


#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
GPS_BAUD ?= B38400

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART3
SBUS_PORT ?= UART3


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
