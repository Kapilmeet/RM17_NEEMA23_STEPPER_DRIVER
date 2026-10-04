/**
 * @file  stepper_direction.c
 * @brief DEFAULT direction decision – simple sign check.
 *
 *        *** MODIFY THIS FILE FOR CUSTOM DIRECTION LOGIC ***
 *        Examples:
 *          - Shortest-path (modulo 360)
 *          - Always-CW / Always-CCW
 *          - Zone-based constraints
 */

#include "stepper_direction.h"

uint8_t Stepper_DecideDirection(float delta_deg)
{
    /* Positive delta  → CW  (direction = 0)
       Negative delta  → CCW (direction = 1) */
    if (delta_deg >= 0.0f) {
        return 0U;   /* CW  */
    } else {
        return 1U;   /* CCW */
    }
}
