/**
 * @file apollo_display.c
 * @brief Display interface implementation (Pinball)
 *
 * From: PINBALL.agc, KEY_ROUTINES.agc, INTER-BANK_COMMUNICATION.agc
 */

#include "apollo_display.h"
#include <string.h>

/* =========================================================================
 * DISPLAY DATA (GLOBAL)
 * ========================================================================= */

static apollo_display_data_t g_display = {0};

/* =========================================================================
 * DISPLAY OPERATIONS
 * ========================================================================= */

void apollo_display_init(void)
{
    memset(&g_display, 0, sizeof(apollo_display_data_t));
}

void apollo_display_update(void)
{
    /* Simplified: would update actual DSKY hardware */
}

void apollo_set_verb_noun(int verb, int noun)
{
    g_display.verb = verb;
    g_display.noun = noun;
}

void apollo_set_register(int reg, const int16_t *data)
{
    switch (reg) {
        case 1:
            g_display.r1[0] = data[0];
            g_display.r1[1] = data[1];
            g_display.r1[2] = data[2];
            break;
        case 2:
            g_display.r2[0] = data[0];
            g_display.r2[1] = data[1];
            g_display.r2[2] = data[2];
            break;
        case 3:
            g_display.r3[0] = data[0];
            g_display.r3[1] = data[1];
            g_display.r3[2] = data[2];
            break;
    }
}

void apollo_clear_display(void)
{
    memset(&g_display, 0, sizeof(apollo_display_data_t));
}

/* =========================================================================
 * TEST DRIVER
 * ========================================================================= */

#ifdef APOLLO_DISPLAY_TEST

#include <stdio.h>

int main(void)
{
    printf("=== Apollo Display Module Tests ===\n");

    apollo_display_init();
    printf("Display initialized\n");

    apollo_set_verb_noun(6, 5);
    printf("Set verb=%d, noun=%d\n", g_display.verb, g_display.noun);

    int16_t data1[] = {123, 456, 789};
    apollo_set_register(1, data1);
    printf("R1: %d %d %d\n", g_display.r1[0], g_display.r1[1], g_display.r1[2]);

    return 0;
}

#endif /* APOLLO_DISPLAY_TEST */
