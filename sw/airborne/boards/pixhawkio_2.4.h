#ifndef CONFIG_PIXHAWK_2_4_H
#define CONFIG_PIXHAWK_2_4_H

#define BOARD_PIXHAWKIO

/* Pixhawk board (PX4FIOv2 has a 24MHz external clock and 24MHz internal. */
#define EXT_CLK 24000000
#define AHB_CLK 24000000


/*
 * Onboard LEDs
 */
/* blue led, a.k.a. AC */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

//led Amber a.k.a power led?
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_PIN GPIO15
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)

//safety led, red???
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_GPIO GPIOB
#define LED_3_GPIO_PIN GPIO13
#define LED_3_GPIO_ON gpio_clear
#define LED_3_GPIO_OFF gpio_set
#define LED_3_AFIO_REMAP ((void)0)

/*
 * UART
*/

// fmu debug
#define UART1_GPIO_AF 0
#define UART1_GPIO_PORT_RX GPIOA
#define UART1_GPIO_RX GPIO10
#define UART1_GPIO_PORT_TX GPIOA
#define UART1_GPIO_TX GPIO9

// intermcu fmu
#define UART2_GPIO_AF 0
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

#if USE_ADC_1
#define AD1_1_CHANNEL 13
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT GPIOC
#define ADC_1_GPIO_PIN GPIO3
#endif

/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
/* GPIO_EXT1 on PX4FMU */

#define SPEKTRUM_BIND_PIN_PORT GPIOC
#define SPEKTRUM_BIND_PIN GPIO13

#define SPEKTRUM_UART1_RCC RCC_USART1
#define SPEKTRUM_UART1_BANK GPIOA
#define SPEKTRUM_UART1_PIN GPIO10
#define SPEKTRUM_UART1_AF 0
#define SPEKTRUM_UART1_IRQ NVIC_USART1_IRQ
#define SPEKTRUM_UART1_ISR usart1_isr
#define SPEKTRUM_UART1_DEV USART1

/*
 * PWM
 *
 */
#define PWM_USE_TIM3 1
#define PWM_USE_TIM5 1

#define USE_PWM1 1
#define USE_PWM2 1
#define USE_PWM3 1
#define USE_PWM4 1

#if DUAL_PWM_ON
#define DUAL_PWM_USE_TIM5 1

#define USE_DUAL_PWM5 1
#define USE_DUAL_PWM6 1
#else
#define USE_PWM5 1
#define USE_PWM6 1
#endif



#if USE_SERVOS_7AND8
#if USE_I2C1
#error "You cannot USE_SERVOS_7AND8 and USE_I2C1 at the same time"
#else
#define ACTUATORS_PWM_NB 8
#define USE_PWM7 1
#define USE_PWM8 1
#define PWM_USE_TIM4 1
#endif
#else
#define ACTUATORS_PWM_NB 6
#endif

// Servo numbering on LisaM silkscreen/docs starts with 1

// PWM_SERVO_x is the index of the servo in the actuators_pwm_values array
#if USE_PWM1
#define PWM_SERVO_1 0
#define PWM_SERVO_1_TIMER TIM3
#define PWM_SERVO_1_GPIO GPIOC
#define PWM_SERVO_1_PIN GPIO6
#define PWM_SERVO_1_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_1_OC TIM_OC1
#define PWM_SERVO_1_OC_BIT (1<<0)
#else
#define PWM_SERVO_1_OC_BIT 0
#endif

#if USE_PWM2
#define PWM_SERVO_2 1
#define PWM_SERVO_2_TIMER TIM3
#define PWM_SERVO_2_GPIO GPIOC
#define PWM_SERVO_2_PIN GPIO7
#define PWM_SERVO_2_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_2_OC TIM_OC2
#define PWM_SERVO_2_OC_BIT (1<<1)
#else
#define PWM_SERVO_2_OC_BIT 0
#endif

#if USE_PWM3
#define PWM_SERVO_3 2
#define PWM_SERVO_3_TIMER TIM3
#define PWM_SERVO_3_GPIO GPIOC
#define PWM_SERVO_3_PIN GPIO8
#define PWM_SERVO_3_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_3_OC TIM_OC3
#define PWM_SERVO_3_OC_BIT (1<<2)
#else
#define PWM_SERVO_3_OC_BIT 0
#endif

#if USE_PWM4
#define PWM_SERVO_4 3
#define PWM_SERVO_4_TIMER TIM3
#define PWM_SERVO_4_GPIO GPIOC
#define PWM_SERVO_4_PIN GPIO9
#define PWM_SERVO_4_AF AFIO_MAPR_TIM3_REMAP_FULL_REMAP
#define PWM_SERVO_4_OC TIM_OC4
#define PWM_SERVO_4_OC_BIT (1<<3)
#else
#define PWM_SERVO_4_OC_BIT 0
#endif

#if USE_PWM5
#define PWM_SERVO_5 4
#define PWM_SERVO_5_TIMER TIM5
#define PWM_SERVO_5_GPIO GPIOA
#define PWM_SERVO_5_PIN GPIO0
#define PWM_SERVO_5_AF 0
#define PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#elif USE_DUAL_PWM5
#define DUAL_PWM_SERVO_5 4

#define DUAL_PWM_SERVO_5_P1 0
#define DUAL_PWM_SERVO_5_P2 1

#define DUAL_PWM_SERVO_5_TIMER TIM5
#define DUAL_PWM_SERVO_5_GPIO GPIOA
#define DUAL_PWM_SERVO_5_PIN GPIO0
#define DUAL_PWM_SERVO_5_AF 0
#define DUAL_PWM_SERVO_5_OC TIM_OC1
#define PWM_SERVO_5_OC_BIT (1<<0)
#else
#define PWM_SERVO_5_OC_BIT 0
#endif

#if USE_PWM6
#define PWM_SERVO_6 5
#define PWM_SERVO_6_TIMER TIM5
#define PWM_SERVO_6_GPIO GPIOA
#define PWM_SERVO_6_PIN GPIO1
#define PWM_SERVO_6_AF 0
#define PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#elif USE_DUAL_PWM6
#define DUAL_PWM_SERVO_6 5

#define DUAL_PWM_SERVO_6_P1 0
#define DUAL_PWM_SERVO_6_P2 1

#define DUAL_PWM_SERVO_6_TIMER TIM5
#define DUAL_PWM_SERVO_6_GPIO GPIOA
#define DUAL_PWM_SERVO_6_PIN GPIO1
#define DUAL_PWM_SERVO_6_AF 0
#define DUAL_PWM_SERVO_6_OC TIM_OC2
#define PWM_SERVO_6_OC_BIT (1<<1)
#else
#define PWM_SERVO_6_OC_BIT 0
#endif






#if USE_PWM7
#define PWM_SERVO_7 6
#define PWM_SERVO_7_TIMER TIM4
#define PWM_SERVO_7_GPIO GPIOB
#define PWM_SERVO_7_PIN GPIO6
#define PWM_SERVO_7_AF 0
#define PWM_SERVO_7_OC TIM_OC1
#define PWM_SERVO_7_OC_BIT (1<<0)
#else
#define PWM_SERVO_7_OC_BIT 0
#endif

#if USE_PWM8
#define PWM_SERVO_8 7
#define PWM_SERVO_8_TIMER TIM4
#define PWM_SERVO_8_GPIO GPIOB
#define PWM_SERVO_8_PIN GPIO7
#define PWM_SERVO_8_AF 0
#define PWM_SERVO_8_OC TIM_OC2
#define PWM_SERVO_8_OC_BIT (1<<1)
#else
#define PWM_SERVO_8_OC_BIT 0
#endif

/* servos 1-4 on TIM3 */
#define PWM_TIM3_CHAN_MASK (PWM_SERVO_1_OC_BIT|PWM_SERVO_2_OC_BIT|PWM_SERVO_3_OC_BIT|PWM_SERVO_4_OC_BIT)
/* servos 5-6 on TIM5 */
#define PWM_TIM5_CHAN_MASK (PWM_SERVO_5_OC_BIT|PWM_SERVO_6_OC_BIT)
/* servos 7-8 on TIM4 if USE_SERVOS_7AND8 */
#define PWM_TIM4_CHAN_MASK (PWM_SERVO_7_OC_BIT|PWM_SERVO_8_OC_BIT)

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()


#endif /* CONFIG_PIXHAWK_2_4_H */
