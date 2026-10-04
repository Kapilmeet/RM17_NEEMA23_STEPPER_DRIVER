#ifndef STEPPER_DIRECTION_H
#define STEPPER_DIRECTION_H

#include <stdint.h>

/**
 * @brief  Decide motor rotation direction.
 *         *** THIS MODULE IS INTENTIONALLY ISOLATED ***
 *         Modify stepper_direction.c to change the strategy
 *         (e.g. shortest-path, wrap-around, zone-based, etc.)
 *
 * @param  delta_deg  (target - current) angle in degrees at output shaft
 * @retval 0 = CW (positive),  1 = CCW (negative)
 */
uint8_t Stepper_DecideDirection(float delta_deg);

#endif /* STEPPER_DIRECTION_H */
