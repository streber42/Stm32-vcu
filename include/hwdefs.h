#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//For testing on STM32-P107 prototyping board, uncomment the following line
#define TEST_P107

//Maximum value for over current limit timer
#define GAUGEMAX 4096
#define USART_BAUDRATE 115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK      ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_25mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMRST    RST_TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define FUELGAUGE_TIMER TIM4

//Address of parameter block in flash for 105
#define FLASH_PAGE_SIZE 2048
#define PARAM_BLKNUM   1
#define PARAM_BLKSIZE  FLASH_PAGE_SIZE
#define CAN_BLKNUM     2
#define CAN_BLKSIZE    FLASH_PAGE_SIZE

typedef enum
{
    HW_REV1, HW_REV2, HW_REV3, HW_TESLA
} HWREV;

extern HWREV hwRev;

#endif // HWDEFS_H_INCLUDED
