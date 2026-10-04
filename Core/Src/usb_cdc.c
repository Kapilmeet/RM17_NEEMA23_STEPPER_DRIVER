/**
 * @file    usb_cdc.c
 * @brief   Lightweight Register-Level USB CDC (Virtual COM Port) Driver for STM32F103
 *          Fully self-contained, no external middleware dependencies.
 */

#include "usb_cdc.h"
#include "stm32f1xx.h"
#include <string.h>

/* ==================== PACKET MEMORY AREA (PMA) LAYOUT ==================== */
#ifndef USB_PMAADDR
#define USB_PMAADDR            0x40006000UL
#endif

/* PMA Buffer Offsets (in bytes from PMA base) */
#define EP0_TX_ADDR            0x40
#define EP0_RX_ADDR            0x80
#define EP1_TX_ADDR            0xC0   /* CDC Bulk IN (Device to Host) */
#define EP2_RX_ADDR            0x100  /* CDC Bulk OUT (Host to Device) */
#define EP3_TX_ADDR            0x140  /* CDC Interrupt IN (Notification) */

#define CDC_DATA_PACKET_SIZE   64
#define RX_BUFFER_SIZE         256

/* BTABLE descriptor pointers (table at offset 0 in PMA) */
#define PMA_ENTRY(ep, member)  ((volatile uint32_t *)(USB_PMAADDR + 4 * ((ep) * 4 + (member))))
#define PMA_ADDR_TX(ep)        PMA_ENTRY(ep, 0)
#define PMA_COUNT_TX(ep)       PMA_ENTRY(ep, 1)
#define PMA_ADDR_RX(ep)        PMA_ENTRY(ep, 2)
#define PMA_COUNT_RX(ep)       PMA_ENTRY(ep, 3)

/* Macro to access EP0R..EP7R dynamically by index (each register spaced by 4 bytes) */
#define USB_EP(ep)             (*((__IO uint16_t *)((uint32_t)&(USB->EP0R) + ((ep) * 4))))

/* Status manipulation according to STM32 Reference Manual RM0008:
 * STAT_TX and STAT_RX are toggle bits (writing 1 toggles, writing 0 does nothing).
 * CTR_RX and CTR_TX are rc_w0 bits (writing 0 clears, writing 1 does nothing).
 * Therefore, when writing to EPnR, CTR_RX and CTR_TX MUST be written as 1 to avoid accidental clearing!
 */
static inline void set_tx_stat(uint8_t ep, uint16_t stat) {
    uint16_t reg = USB_EP(ep);
    uint16_t toggle = (reg ^ stat) & USB_EPTX_STAT;
    USB_EP(ep) = (uint16_t)((reg & (USB_EPREG_MASK & ~USB_EPTX_STAT)) | toggle | USB_EP_CTR_RX | USB_EP_CTR_TX);
}

static inline void set_rx_stat(uint8_t ep, uint16_t stat) {
    uint16_t reg = USB_EP(ep);
    uint16_t toggle = (reg ^ stat) & USB_EPRX_STAT;
    USB_EP(ep) = (uint16_t)((reg & (USB_EPREG_MASK & ~USB_EPRX_STAT)) | toggle | USB_EP_CTR_RX | USB_EP_CTR_TX);
}

static inline void clear_ctr_tx(uint8_t ep) {
    USB_EP(ep) = (uint16_t)((USB_EP(ep) & USB_EPREG_MASK & ~USB_EP_CTR_TX) | USB_EP_CTR_RX);
}

static inline void clear_ctr_rx(uint8_t ep) {
    USB_EP(ep) = (uint16_t)((USB_EP(ep) & USB_EPREG_MASK & ~USB_EP_CTR_RX) | USB_EP_CTR_TX);
}

/* ==================== PMA COPY FUNCTIONS ==================== */

static void pma_write(uint16_t pma_offset, const uint8_t *src, uint16_t len)
{
    volatile uint32_t *pma = (volatile uint32_t *)(USB_PMAADDR + (pma_offset * 2));
    uint16_t words = (len + 1) / 2;
    for (uint16_t i = 0; i < words; i++) {
        uint16_t val = src[2 * i];
        if ((2 * i + 1) < len) {
            val |= ((uint16_t)src[2 * i + 1]) << 8;
        }
        *pma++ = val;
    }
}

static void pma_read(uint16_t pma_offset, uint8_t *dst, uint16_t len)
{
    volatile uint32_t *pma = (volatile uint32_t *)(USB_PMAADDR + (pma_offset * 2));
    uint16_t words = (len + 1) / 2;
    for (uint16_t i = 0; i < words; i++) {
        uint16_t val = (uint16_t)(*pma++);
        dst[2 * i] = (uint8_t)(val & 0xFF);
        if ((2 * i + 1) < len) {
            dst[2 * i + 1] = (uint8_t)((val >> 8) & 0xFF);
        }
    }
}

/* ==================== USB DESCRIPTORS ==================== */

static const uint8_t device_descriptor[] = {
    18, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 64,
    0x83, 0x04, 0x40, 0x57, 0x00, 0x02,
    1, 2, 3, 1
};

static const uint8_t config_descriptor[] = {
    /* Configuration Descriptor */
    9, 0x02, 67, 0x00, 2, 1, 0, 0xC0, 50,

    /* Interface 0: CDC Communication */
    9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,
    /* Header Functional Descriptor */
    5, 0x24, 0x00, 0x10, 0x01,
    /* Call Management Functional Descriptor */
    5, 0x24, 0x01, 0x00, 1,
    /* ACM Functional Descriptor */
    4, 0x24, 0x02, 0x02,
    /* Union Functional Descriptor */
    5, 0x24, 0x06, 0, 1,
    /* Endpoint 3 IN (Interrupt) */
    7, 0x05, 0x83, 0x03, 8, 0, 16,

    /* Interface 1: CDC Data */
    9, 0x04, 1, 0, 2, 0x0A, 0x00, 0x00, 0,
    /* Endpoint 1 IN (Bulk IN) */
    7, 0x05, 0x81, 0x02, CDC_DATA_PACKET_SIZE, 0, 0,
    /* Endpoint 2 OUT (Bulk OUT) */
    7, 0x05, 0x02, 0x02, CDC_DATA_PACKET_SIZE, 0, 0
};

static const uint8_t string_lang_id[] = { 4, 0x03, 0x09, 0x04 };
static const uint8_t string_vendor[]  = { 18, 0x03, 'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'1',0,'0',0 };
static const uint8_t string_product[] = { 26, 0x03, 'S',0,'t',0,'e',0,'p',0,'p',0,'e',0,'r',0,' ',0,'C',0,'D',0,'C',0 };
static const uint8_t string_serial[]  = { 14, 0x03, '0',0,'0',0,'0',0,'0',0,'0',0,'1',0 };

/* CDC Line Coding: 115200 baud, 1 stop bit, no parity, 8 data bits */
static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };

/* ==================== DRIVER STATE ==================== */

static volatile uint8_t  usb_configured = 0;
static volatile uint8_t  tx_busy = 0;
static volatile uint8_t  ep0_stage = 0;
static volatile uint8_t  ep0_req_type = 0;
static const uint8_t    *ep0_data_ptr = NULL;
static uint16_t          ep0_data_len = 0;
static uint8_t           usb_dev_addr = 0;

/* Circular Receive Buffer */
static uint8_t  rx_buf[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* ==================== CONTROL ENDPOINT (EP0) HANDLING ==================== */

static void ep0_send_data(const uint8_t *data, uint16_t len, uint16_t req_len)
{
    if (len > req_len) len = req_len;
    ep0_data_ptr = data;
    ep0_data_len = len;

    uint16_t chunk = (len > 64) ? 64 : len;
    pma_write(EP0_TX_ADDR, ep0_data_ptr, chunk);
    *PMA_COUNT_TX(0) = chunk;
    ep0_data_ptr += chunk;
    ep0_data_len -= chunk;

    set_tx_stat(0, USB_EP_TX_VALID);
}

static void handle_setup(void)
{
    uint8_t req[8];
    pma_read(EP0_RX_ADDR, req, 8);

    uint8_t  bmRequestType = req[0];
    uint8_t  bRequest      = req[1];
    uint16_t wValue        = (uint16_t)req[2] | ((uint16_t)req[3] << 8);
    uint16_t wLength       = (uint16_t)req[6] | ((uint16_t)req[7] << 8);

    ep0_req_type = 0;

    if ((bmRequestType & 0x60) == 0x00) {
        /* Standard Request */
        switch (bRequest) {
            case 0x06: /* GET_DESCRIPTOR */
                switch (wValue >> 8) {
                    case 0x01: ep0_send_data(device_descriptor, sizeof(device_descriptor), wLength); break;
                    case 0x02: ep0_send_data(config_descriptor, sizeof(config_descriptor), wLength); break;
                    case 0x03:
                        switch (wValue & 0xFF) {
                            case 0: ep0_send_data(string_lang_id, sizeof(string_lang_id), wLength); break;
                            case 1: ep0_send_data(string_vendor,  sizeof(string_vendor),  wLength); break;
                            case 2: ep0_send_data(string_product, sizeof(string_product), wLength); break;
                            case 3: ep0_send_data(string_serial,  sizeof(string_serial),  wLength); break;
                            default: set_tx_stat(0, USB_EP_TX_STALL); break;
                        }
                        break;
                    default: set_tx_stat(0, USB_EP_TX_STALL); break;
                }
                break;

            case 0x05: /* SET_ADDRESS */
                usb_dev_addr = (uint8_t)(wValue & 0x7F);
                *PMA_COUNT_TX(0) = 0;
                set_tx_stat(0, USB_EP_TX_VALID);
                ep0_stage = 1; /* Assign address after status handshake */
                break;

            case 0x09: /* SET_CONFIGURATION */
                usb_configured = 1;
                /* Configure EP1 (Bulk IN) */
                *PMA_ADDR_TX(1) = EP1_TX_ADDR;
                *PMA_COUNT_TX(1) = 0;
                USB_EP(1) = USB_EP_BULK | 1;
                set_tx_stat(1, USB_EP_TX_NAK);
                set_rx_stat(1, USB_EP_RX_DIS);

                /* Configure EP2 (Bulk OUT) */
                *PMA_ADDR_RX(2) = EP2_RX_ADDR;
                *PMA_COUNT_RX(2) = (1 << 15) | (1 << 10); /* 64 bytes */
                USB_EP(2) = USB_EP_BULK | 2;
                set_rx_stat(2, USB_EP_RX_VALID);
                set_tx_stat(2, USB_EP_TX_DIS);

                /* Configure EP3 (Interrupt IN) */
                *PMA_ADDR_TX(3) = EP3_TX_ADDR;
                *PMA_COUNT_TX(3) = 0;
                USB_EP(3) = USB_EP_INTERRUPT | 3;
                set_tx_stat(3, USB_EP_TX_NAK);
                set_rx_stat(3, USB_EP_RX_DIS);

                *PMA_COUNT_TX(0) = 0;
                set_tx_stat(0, USB_EP_TX_VALID);
                break;

            case 0x00: /* GET_STATUS */
                {
                    uint8_t status[2] = {0, 0};
                    ep0_send_data(status, 2, wLength);
                }
                break;

            default:
                *PMA_COUNT_TX(0) = 0;
                set_tx_stat(0, USB_EP_TX_VALID);
                break;
        }
    } else if ((bmRequestType & 0x60) == 0x20) {
        /* CDC Class-Specific Requests */
        switch (bRequest) {
            case 0x20: /* SET_LINE_CODING */
                /* Expect 7 data bytes on EP0 OUT stage */
                ep0_req_type = 0x20;
                set_rx_stat(0, USB_EP_RX_VALID);
                break;

            case 0x21: /* GET_LINE_CODING */
                ep0_send_data(line_coding, sizeof(line_coding), wLength);
                break;

            case 0x22: /* SET_CONTROL_LINE_STATE */
                /* No data stage (wLength == 0). Send zero-length status handshake! */
                *PMA_COUNT_TX(0) = 0;
                set_tx_stat(0, USB_EP_TX_VALID);
                break;

            default:
                set_tx_stat(0, USB_EP_TX_STALL);
                break;
        }
    } else {
        set_tx_stat(0, USB_EP_TX_STALL);
    }
}

/* ==================== USB INTERRUPT HANDLER ==================== */

void USB_CDC_IRQHandler(void)
{
    uint32_t istr = USB->ISTR;

    /* 1. USB Bus Reset */
    if (istr & USB_ISTR_RESET) {
        USB->ISTR = (uint16_t)(~USB_ISTR_RESET);
        USB->BTABLE = 0;

        /* Reset EP0 */
        *PMA_ADDR_TX(0) = EP0_TX_ADDR;
        *PMA_COUNT_TX(0) = 0;
        *PMA_ADDR_RX(0) = EP0_RX_ADDR;
        *PMA_COUNT_RX(0) = (1 << 15) | (1 << 10); /* 64 bytes */

        USB_EP(0) = USB_EP_CONTROL | 0;
        set_rx_stat(0, USB_EP_RX_VALID);
        set_tx_stat(0, USB_EP_TX_NAK);

        USB->DADDR = USB_DADDR_EF | 0;
        usb_configured = 0;
        tx_busy = 0;
        return;
    }

    /* Clear unhandled flags so ISR does not re-trigger */
    if (istr & (USB_ISTR_SUSP | USB_ISTR_WKUP | USB_ISTR_ERR | USB_ISTR_PMAOVR)) {
        USB->ISTR = (uint16_t)(~(USB_ISTR_SUSP | USB_ISTR_WKUP | USB_ISTR_ERR | USB_ISTR_PMAOVR));
    }

    /* 2. Correct Transfer (CTR) */
    while ((USB->ISTR & USB_ISTR_CTR) != 0) {
        uint8_t ep = (uint8_t)(USB->ISTR & USB_ISTR_EP_ID);

        if (ep == 0) {
            /* Control Endpoint */
            if (USB_EP(0) & USB_EP_CTR_RX) {
                clear_ctr_rx(0);
                if (USB_EP(0) & USB_EP_SETUP) {
                    handle_setup();
                } else {
                    /* Control DATA OUT stage */
                    if (ep0_req_type == 0x20) { /* SET_LINE_CODING data payload */
                        uint16_t count = (uint16_t)(*PMA_COUNT_RX(0) & 0x3FF);
                        if (count > 7) count = 7;
                        pma_read(EP0_RX_ADDR, line_coding, count);
                        ep0_req_type = 0;
                        /* Send Status Handshake ACK to host */
                        *PMA_COUNT_TX(0) = 0;
                        set_tx_stat(0, USB_EP_TX_VALID);
                    }
                }
                set_rx_stat(0, USB_EP_RX_VALID);
            }
            if (USB_EP(0) & USB_EP_CTR_TX) {
                clear_ctr_tx(0);
                if (ep0_stage == 1) {
                    /* Assign USB address after status handshake */
                    USB->DADDR = USB_DADDR_EF | usb_dev_addr;
                    ep0_stage = 0;
                } else if (ep0_data_len > 0) {
                    uint16_t chunk = (ep0_data_len > 64) ? 64 : ep0_data_len;
                    pma_write(EP0_TX_ADDR, ep0_data_ptr, chunk);
                    *PMA_COUNT_TX(0) = chunk;
                    ep0_data_ptr += chunk;
                    ep0_data_len -= chunk;
                    set_tx_stat(0, USB_EP_TX_VALID);
                }
            }
        } else if (ep == 1) {
            /* EP1: CDC Bulk IN (TX completed) */
            if (USB_EP(1) & USB_EP_CTR_TX) {
                clear_ctr_tx(1);
                tx_busy = 0;
            }
        } else if (ep == 2) {
            /* EP2: CDC Bulk OUT (RX received data from host) */
            if (USB_EP(2) & USB_EP_CTR_RX) {
                clear_ctr_rx(2);
                uint16_t count = (uint16_t)(*PMA_COUNT_RX(2) & 0x3FF);
                uint8_t temp[CDC_DATA_PACKET_SIZE];
                if (count > CDC_DATA_PACKET_SIZE) count = CDC_DATA_PACKET_SIZE;
                pma_read(EP2_RX_ADDR, temp, count);

                /* Store into ring buffer */
                for (uint16_t i = 0; i < count; i++) {
                    uint16_t next = (rx_head + 1) % RX_BUFFER_SIZE;
                    if (next != rx_tail) {
                        rx_buf[rx_head] = temp[i];
                        rx_head = next;
                    }
                }
                set_rx_stat(2, USB_EP_RX_VALID);
            }
        } else if (ep == 3) {
            /* EP3: CDC Notification */
            if (USB_EP(3) & USB_EP_CTR_TX) {
                clear_ctr_tx(3);
            }
        } else {
            /* Safety clear for any unexpected endpoint */
            clear_ctr_rx(ep);
            clear_ctr_tx(ep);
            break;
        }
    }
}

/* ==================== PUBLIC API ==================== */

void USB_CDC_Init(void)
{
    /* Blue Pill USB Re-enumeration trick:
     * Pull PA12 (USB DP) low for 15ms to force host (Windows) to detect disconnect.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(25);

    /* Release PA12 back to input floating so USB peripheral takes over */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Enable USB peripheral clock */
    __HAL_RCC_USB_CLK_ENABLE();

    /* NVIC configuration for USB Low Priority Interrupt */
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3);
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    /* Power up USB macrocell */
    USB->CNTR = USB_CNTR_FRES;
    for (volatile int i = 0; i < 1000; i++);
    USB->CNTR = 0;
    USB->ISTR = 0;

    /* Enable Reset, Correct Transfer interrupts */
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
}

uint8_t USB_CDC_Transmit(const uint8_t *data, uint16_t len)
{
    if (!usb_configured) return 0;

    /* Wait for previous transmission to finish (timeout ~50ms) */
    uint32_t timeout = 50000;
    while (tx_busy && --timeout);
    if (timeout == 0) return 0;

    while (len > 0) {
        uint16_t chunk = (len > CDC_DATA_PACKET_SIZE) ? CDC_DATA_PACKET_SIZE : len;
        tx_busy = 1;
        pma_write(EP1_TX_ADDR, data, chunk);
        *PMA_COUNT_TX(1) = chunk;
        set_tx_stat(1, USB_EP_TX_VALID);

        data += chunk;
        len -= chunk;

        timeout = 50000;
        while (tx_busy && --timeout);
        if (timeout == 0) return 0;
    }

    return 1;
}

uint8_t USB_CDC_Print(const char *str)
{
    return USB_CDC_Transmit((const uint8_t *)str, (uint16_t)strlen(str));
}

uint16_t USB_CDC_Available(void)
{
    if (rx_head >= rx_tail) {
        return rx_head - rx_tail;
    } else {
        return RX_BUFFER_SIZE + rx_head - rx_tail;
    }
}

int16_t USB_CDC_Read(void)
{
    if (rx_head == rx_tail) {
        return -1;
    }
    uint8_t byte = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return byte;
}
