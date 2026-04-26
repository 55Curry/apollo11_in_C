/**
 * @file apollo_io.h
 * @brief Input/Output
 *
 * From: KEYRUPT_UPRUPT.agc, INTERRUPT_LEAD_INS.agc
 */

#ifndef APOLLO_IO_H
#define APOLLO_IO_H

#include "apollo_types.h"

#define APOLLO_MAX_CHANNELS 64
#define APOLLO_MAX_INTERRUPTS 16

typedef void (*apollo_interrupt_handler_t)(void);

typedef struct {
    uint16_t channels[APOLLO_MAX_CHANNELS];
    apollo_interrupt_handler_t handlers[APOLLO_MAX_INTERRUPTS];
    uint16_t interrupt_enable;
    uint16_t interrupt_pending;
} apollo_io_state_t;

void apollo_io_init(void);
uint16_t apollo_read_channel(int channel);
void apollo_write_channel(int channel, uint16_t value);
void apollo_enable_interrupt(int irq);
void apollo_disable_interrupt(int irq);

#endif /* APOLLO_IO_H */
