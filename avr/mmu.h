#ifndef __AVR_MMU_H
#define __AVR_MMU_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "command.h"
#include "sched.h"
#include "autoconf.h"

#define MMU_BAUD 115200
#define MMU_BAUD_SELECT(baudRate, xtalCpu) \
    (((float)(xtalCpu))/(((float)(baudRate))*8.0) - 1.0 + 0.5)

#define MMU_RX_SIZE 256
#define MMU_RX_MASK (MMU_RX_SIZE - 1)

void mmu_uart_init(void);
void mmu_uart_send(uint8_t *data, uint8_t len);
uint8_t mmu_uart_read(uint8_t *out, uint8_t maxlen);

DECL_INIT(mmu_uart_init);
DECL_COMMAND(command_mmu_write, "mmu_write data=%*s");
DECL_COMMAND(command_mmu_read, "mmu_read max=%c");

#endif
