#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

/**
 * @brief  Initialise the USB peripheral as a CDC ACM virtual COM port.
 *         Forces a bus reconnect so the host re-enumerates.
 */
void USB_CDC_Init(void);

/**
 * @brief  Transmit data to the host (blocking until previous TX finishes).
 * @param  data  Pointer to bytes
 * @param  len   Number of bytes (max 64 per call)
 * @retval 1 = success, 0 = not configured / timeout
 */
uint8_t USB_CDC_Transmit(const uint8_t *data, uint16_t len);

/**
 * @brief  Transmit a null-terminated string.
 */
uint8_t USB_CDC_Print(const char *str);

/**
 * @brief  Number of bytes waiting in the receive ring-buffer.
 */
uint16_t USB_CDC_Available(void);

/**
 * @brief  Read one byte from the receive buffer.
 * @retval 0-255 = byte,  -1 = buffer empty
 */
int16_t USB_CDC_Read(void);

/**
 * @brief  USB low-priority interrupt handler.  Call from USB_LP_CAN1_RX0_IRQHandler.
 */
void USB_CDC_IRQHandler(void);

#endif /* USB_CDC_H */
