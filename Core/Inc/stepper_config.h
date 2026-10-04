#ifndef STEPPER_CONFIG_H
#define STEPPER_CONFIG_H

#include "stm32f1xx_hal.h"

/* ==================== PIN DEFINITIONS ==================== */
/* Primary Output Pins (Set 1) */
#define STEP_PORT              GPIOA
#define STEP_PIN               GPIO_PIN_0    /* PWM / STEP pulse 1: PA0, TIM2 Channel 1 */

#define DIR_PORT               GPIOA
#define DIR_PIN                GPIO_PIN_1    /* DIR 1: PA1 */

#define ENA_PORT               GPIOA
#define ENA_PIN                GPIO_PIN_3    /* ENA 1: PA3 */

/* Secondary Simultaneous Output Pins (Set 2) */
#define STEP2_PORT             GPIOB
#define STEP2_PIN              GPIO_PIN_13   /* PWM / STEP pulse 2: PB13, TIM1 Channel 1N */

#define DIR2_PORT              GPIOB
#define DIR2_PIN               GPIO_PIN_12   /* DIR 2: PB12 */

#define ENA2_PORT              GPIOA
#define ENA2_PIN               GPIO_PIN_7    /* ENA 2: PA7 */

/* ==================== MOTOR & GEARBOX CONSTANTS ==================== */
#define GEAR_RATIO             10U         /* 1:10 gearbox */
#define DEFAULT_PULSES_PER_REV 1600U       /* Default microstep PPR (motor shaft) */

/* ==================== DEFAULT MOTION PARAMETERS (OUTPUT) ==================== */
#define DEFAULT_OUTPUT_ACCEL   0.5f        /* rad/s^2 at gearbox output */
#define DEFAULT_OUTPUT_SPEED   0.5f        /* rad/s   at gearbox output */

/* ==================== TIMER CONSTANTS ==================== */
/*  TIM2 on APB1: PCLK1=36MHz, timer clk = 2*PCLK1 = 72MHz  */
/*  TIM1 on APB2: PCLK2=72MHz, timer clk = 1*PCLK2 = 72MHz  */
#define TIM_CLK_FREQ           72000000UL
#define RAMP_UPDATE_FREQ       10000U      /* TIM3 ISR rate in Hz (100us) */

/* ==================== MOTION LIMITS ==================== */
#define MIN_STEP_FREQ          100.0f      /* Hz – starting / stopping speed */
#define MAX_STEP_FREQ          100000.0f   /* Hz – absolute ceiling */

/* ==================== N-MOTION ROTARY MODULO 360 CONTROL ==================== */
/*
 * 1: Shortest-Path Modulo 360: Rotates along the shortest circular path (<= 180 deg)
 *    to reach target orientation (e.g. from 0 to 270 moves -90 CCW).
 * 0: Direct Modulo 360: Preserves commanded direction for single-turn moves, but
 *    suppresses redundant full 360-degree rotations.
 *
 * In BOTH modes: If (current_angle - target_angle) is any integer multiple of 360 degrees
 * (e.g. 0 to 360, 0 to -360, 90 to -270, 90 to 450), the motor does NOT move.
 */
#define N_MOTION_SHORTEST_PATH 0U

/* ==================== MATH ==================== */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define PI_F 3.14159265f

/* ==================== DM860H MACROS ==================== */
/* For DM860H driver:
 * Common-Cathode wiring (signal on ENA+, DIR+, PUL+ pins; all '-' pins to GND):
 *   - ENA optocoupler OFF (PA3/PA7 = LOW / 0V)  -> Motor is ENABLED (energized with holding torque).
 *   - ENA optocoupler ON  (PA3/PA7 = HIGH / 3.3V) -> Motor is DISABLED (free-spinning).
 *
 * If your wiring uses Common-Anode (all '+' pins to +5V and '-' pins to STM32):
 * Swap GPIO_PIN_RESET and GPIO_PIN_SET below.
 */
#define MOTOR_ENABLE()   do { \
    HAL_GPIO_WritePin(ENA_PORT,  ENA_PIN,  GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(ENA2_PORT, ENA2_PIN, GPIO_PIN_RESET); \
} while(0)

#define MOTOR_DISABLE()  do { \
    HAL_GPIO_WritePin(ENA_PORT,  ENA_PIN,  GPIO_PIN_SET); \
    HAL_GPIO_WritePin(ENA2_PORT, ENA2_PIN, GPIO_PIN_SET); \
} while(0)

#define DIR_CW()         do { \
    HAL_GPIO_WritePin(DIR_PORT,  DIR_PIN,  GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_RESET); \
} while(0)

#define DIR_CCW()        do { \
    HAL_GPIO_WritePin(DIR_PORT,  DIR_PIN,  GPIO_PIN_SET); \
    HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, GPIO_PIN_SET); \
} while(0)

/* ==================== SAFE MICROSECOND DELAY ==================== */
/* Cycle-accurate assembly delay loop (18 iterations @ 72MHz = 1 microsecond).
 * 100% immune to DWT lockup, missing trace peripherals, or clone MCU quirks.
 */
static inline void DWT_Init(void) {
    /* No hardware dependency needed */
}

static inline void delay_us(uint32_t us) {
    uint32_t count = us * 18U;
    __asm__ volatile (
        "1: subs %0, %0, #1 \n"
        "   bne 1b          \n"
        : "+r" (count)
        :
        : "cc"
    );
}

#endif /* STEPPER_CONFIG_H */
