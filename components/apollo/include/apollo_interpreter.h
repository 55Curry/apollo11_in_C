/**
 * @file apollo_interpreter.h
 * @brief AGC Interpretive Language
 *
 * From: INTERPRETER.agc
 */

#ifndef APOLLO_INTERPRETER_H
#define APOLLO_INTERPRETER_H

#include "apollo_types.h"

/* =========================================================================
 * INTERPRETIVE INSTRUCTIONS
 * ========================================================================= */

#define APOLLO_INTERP_BANK 0x10

/* Op codes */
typedef enum {
    OP_LXCH = 0,
    OP_INCR = 1,
    OP_AD = 2,
    OP_ADD = 3,
    OP_BOTH = 4,
    OP_BZMF = 5,
    OP_BOV = 6,
    OP_BNO = 7,
    OP_DAS = 8,
    OP_DCA = 9,
    OP_DCOM = 10,
    OP_DIM = 11,
    OP_DOUBLE = 12,
    OP_DTCB = 13,
    OP_DTCF = 14,
    OP_DV = 15,
    OP_ECA = 16,
    OP_EXTEND = 17,
    OP_INDEX = 18,
    OP_JMP = 19,
    OP_JSTEER = 20,
    OP_MASK = 21,
    OP_MP = 22,
    OP_MS = 23,
    OP_MSK = 24,
    OP_QXCH = 25,
    OP_RAND = 26,
    OP_READ = 27,
    OP_RECYCLE = 28,
    OP_RESUME = 29,
    OP_ROR = 30,
    OP_RXOR = 31,
    OP_SETLOC = 32,
    OP_SQUARE = 33,
    OP_SU = 34,
    OP_TCF = 35,
    OP_TCR = 36,
    OP_TD = 37,
    OP_TE = 38,
    OP_TS = 39,
    OP_TXI = 40,
    OP_XCH = 41,
    OP_XSQ = 42,
    OP_ZQ = 43
} apollo_opcode_t;

/* =========================================================================
 * INTERPRETIVE STATE
 * ========================================================================= */

typedef struct {
    int16_t loc;         /* Location counter */
    int16_t q;           /* Q register (return) */
    int16_t z;           /* Z register (next) */
    int16_t a;           /* A register */
    int16_t l;           /* L register */
    int16_t g;           /* G register */
    int16_t eraseable[2048]; /* Erasable memory */
} apollo_interp_state_t;

void apollo_interp_init(void);
void apollo_interp_run(void);
int16_t apollo_interp_step(void);

#endif /* APOLLO_INTERPRETER_H */
