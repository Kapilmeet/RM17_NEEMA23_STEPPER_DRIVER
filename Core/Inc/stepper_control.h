#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <stdint.h>

/* ==================== TYPES ==================== */

typedef enum {
    STEPPER_IDLE  = 0,
    STEPPER_ACCEL,
    STEPPER_CRUISE,
    STEPPER_DECEL
} StepperState;

typedef struct {
    /* --- Configuration (set before a move, stable during motion) --- */
    uint32_t  pulses_per_rev;        /* Microstep PPR on motor shaft          */
    float     output_accel;          /* rad/s^2 at output (default)           */
    float     output_speed;          /* rad/s   at output (default)           */

    /* --- Move parameters (computed in Stepper_MoveTo) --- */
    int32_t   target_steps;          /* Total motor steps for this move       */
    int32_t   accel_steps;           /* Steps in accel phase                  */
    int32_t   decel_start_step;      /* Step index where decel begins         */
    float     step_accel;            /* Acceleration in steps/s^2             */
    float     max_step_speed;        /* Cruise speed   in steps/s             */
    float     min_step_speed;        /* Start/end speed in steps/s            */
    float     degrees_per_step;      /* Output degrees per motor step         */
    float     move_target_deg;       /* Target position for this move         */

    /* --- ISR-modified state (volatile) --- */
    volatile int32_t      steps_done;
    volatile StepperState  state;
    volatile uint8_t       moving;

    /* --- Ramp engine internal (touched only in TIM3 ISR) --- */
    float     current_speed;         /* Current step frequency (steps/s)      */

    /* --- Position tracking --- */
    volatile float  current_position_deg;  /* Absolute output shaft position */
    uint8_t         direction;             /* 0 = CW,  1 = CCW               */
    float           move_sign;             /* +1.0 or -1.0                    */
} StepperMotor;

/* ==================== PUBLIC API ==================== */

void   Stepper_Init          (StepperMotor *m);
void   Stepper_MoveTo        (StepperMotor *m, float target_deg,
                               uint32_t ppr, float accel, float speed);
void   Stepper_Stop          (StepperMotor *m);
float  Stepper_GetPosition   (StepperMotor *m);
void   Stepper_SetHome       (StepperMotor *m);

/* --- Called from ISR context only --- */
void   Stepper_RampUpdate    (StepperMotor *m);   /* TIM3 ISR – 10 kHz */
void   Stepper_PulseCallback (StepperMotor *m);   /* TIM2 ISR          */

#endif /* STEPPER_CONTROL_H */
