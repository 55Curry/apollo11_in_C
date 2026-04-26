/**
 * @file apollo_io.c
 * @brief I/O interface implementation
 *
 * From: CHANNEL.agc, INPUT_OUTPUT_PACKAGE.agc
 */

#include "apollo_io.h"
#include <string.h>

/* =========================================================================
 * IO STATE (GLOBAL)
 * ========================================================================= */

static apollo_io_state_t g_io = {0};

/* =========================================================================
 * CHANNEL I/O
 * ========================================================================= */

void apollo_io_init(void)
{
    memset(&g_io, 0, sizeof(apollo_io_state_t));
}

uint16_t apollo_read_channel(int channel)
{
    if (channel >= 0 && channel < APOLLO_MAX_CHANNELS) {
        return g_io.channels[channel];
    }
    return 0;
}

void apollo_write_channel(int channel, uint16_t value)
{
    if (channel >= 0 && channel < APOLLO_MAX_CHANNELS) {
        g_io.channels[channel] = value;
    }
}

/* =========================================================================
 * INTERRUPTS
 * ========================================================================= */

void apollo_enable_interrupt(int irq)
{
    if (irq >= 0 && irq < APOLLO_MAX_INTERRUPTS) {
        g_io.interrupt_enable |= (1 << irq);
    }
}

void apollo_disable_interrupt(int irq)
{
    if (irq >= 0 && irq < APOLLO_MAX_INTERRUPTS) {
        g_io.interrupt_enable &= ~(1 << irq);
    }
}

/* =========================================================================
 * TEST DRIVER
 * ========================================================================= */

#ifdef APOLLO_IO_TEST

#include <stdio.h>

int main(void)
{
    printf("=== Apollo I/O Module Tests ===\n");

    apollo_io_init();
    printf("I/O initialized\n");

    apollo_write_channel(10, 12345);
    printf("Channel 10: %u\n", apollo_read_channel(10));

    apollo_enable_interrupt(3);
    printf("Interrupt 3 enabled\n");

    return 0;
}

#endif /* APOLLO_IO_TEST */
