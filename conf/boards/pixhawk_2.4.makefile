# Hey Emacs, this is a -*- makefile -*-
#
# pixhawk_2.4.makefile
#
# This is for the main MCU (STM32F427) on the pixhawk board
# See https://pixhawk.org/modules/pixhawk for details
#

BOARD=pixhawk
BOARD_VERSION=2.4
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/pixhawk_2.4.ld

HARD_FLOAT=yes

# default flash mode is the pixhawk bootloader
# possibilities: DFU, SWD, pixhawk bootloader
FLASH_MODE ?= PIXHAWK_BOOTLOADER
PIXHAWK_PROTOTYPE ?= "/home/houjebek/px4/Firmware/Images/px4fmu-v2.prototype"
PIXHAWK_BL_PORT ?= "/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/pci-3D_Robotics*"
$(TARGET).MAKEFILE = pixhawk

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART4
GPS_BAUD ?= B38400

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART2


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
