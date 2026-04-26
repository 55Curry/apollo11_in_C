/**
 * @file apollo_interpreter.c
 * @brief Interpreter implementation
 *
 * From: INTERPRETER.agc, BASIC_INTERPRETER.agc
 */

#include "apollo_interpreter.h"
#include <string.h>

/* =========================================================================
 * INTERPRETER STATE (GLOBAL)
 * ========================================================================= */

static apollo_interp_state_t g_interp = {0};

/* =========================================================================
 * INTERPRETER OPERATIONS
 * ========================================================================= */

void apollo_interp_init(void)
{
    memset(&g_interp, 0, sizeof(apollo_interp_state_t));
    g_interp.loc = 0;
    g_interp.q = 0;
    g_interp.z = 0;
    g_interp.a = 0;
    g_interp.l = 0;
    g_interp.g = 0;
}

void apollo_interp_run(void)
{
    /* Run interpreter until halt */
    while (apollo_interp_step() == 0) {
        /* Continue until step returns non-zero (halt) */
    }
}

int16_t apollo_interp_step(void)
{
    /* Simplified: fetch and execute one instruction */
    uint16_t instruction = g_interp.eraseable[g_interp.loc];

    /* Extract opcode */
    apollo_opcode_t opcode = (instruction >> 10) & 0x3F;

    /* Execute instruction */
    switch (opcode) {
        case OP_LXCH:
            /* LXCH - exchange L with a memory location */
            break;
        case OP_INCR:
            /* INCR - increment memory */
            g_interp.eraseable[g_interp.loc]++;
            break;
        case OP_ADD:
            /* ADD - add memory to A register */
            g_interp.a += g_interp.eraseable[g_interp.loc];
            break;
        case OP_AD:
            /* AD - add memory to A (double precision) */
            break;
        case OP_TS:
            /* TS - transfer to storage */
            break;
        case OP_XCH:
            /* XCH - exchange A with memory */
            {
                int16_t temp = g_interp.a;
                g_interp.a = g_interp.eraseable[g_interp.loc];
                g_interp.eraseable[g_interp.loc] = temp;
            }
            break;
        default:
            /* Unknown or unimplemented opcode */
            break;
    }

    /* Advance location */
    g_interp.loc++;

    /* Return 0 to continue, non-zero to halt */
    return 0;
}

#ifdef APOLLO_INTERPRETER_TEST

#include <stdio.h>

int main(void)
{
    printf("=== Apollo Interpreter Module Tests ===\n");

    apollo_interp_init();
    printf("Interpreter initialized\n");

    /* Set a simple program: ADD at location 0 */
    g_interp.eraseable[0] = (OP_ADD << 10) | 100;  /* ADD from address 100 */
    g_interp.eraseable[100] = 42;

    printf("Set up ADD instruction\n");

    apollo_interp_step();
    printf("After step: A=%d\n", g_interp.a);

    return 0;
}

#endif /* APOLLO_INTERPRETER_TEST */
