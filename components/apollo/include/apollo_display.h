/**
 * @file apollo_display.h
 * @brief Display and Keyboard Interface
 *
 * From: PINBALL_GAME_BUTTONS_AND_LIGHTS.agc, DISPLAY_INTERFACE_ROUTINES.agc
 */

#ifndef APOLLO_DISPLAY_H
#define APOLLO_DISPLAY_H

#include "apollo_types.h"

/* =========================================================================
 * DISPLAY CONSTANTS
 * ========================================================================= */

#define APOLLO_NUMERIC_WIDTH  8
#define APOLLO_NUM_NOUN_REG   6
#define APOLLO_NUM_VERB_REG   2
#define APOLLO_CHARS_PER_LINE 26

/* =========================================================================
 * NOUN TYPES
 * ========================================================================= */

typedef enum {
    APOLLO_NOUN_TYPE_3FIXED = 0,
    APOLLO_NOUN_TYPE_3DEC = 1,
    APOLLO_NOUN_TYPE_2INT = 2,
    APOLLO_NOUN_TYPE_3INT = 3,
    APOLLO_NOUN_TYPE_ANG = 4,
    APOLLO_NOUN_TYPE_1COMP = 5,
    APOLLO_NOUN_TYPE_2COMP = 6
} apollo_noun_type_t;

/* =========================================================================
 * VERBS
 * ========================================================================= */

typedef enum {
    APOLLO_VERB_09 = 9,
    APOLLO_VERB_37 = 37,
    APOLLO_VERB_41 = 41,
    APOLLO_VERB_44 = 44,
    APOLLO_VERB_45 = 45,
    APOLLO_VERB_46 = 46,
    APOLLO_VERB_47 = 47,
    APOLLO_VERB_49 = 49,
    APOLLO_VERB_50 = 50,
    APOLLO_VERB_54 = 54,
    APOLLO_VERB_56 = 56,
    APOLLO_VERB_64 = 64
} apollo_verb_t;

/* =========================================================================
 * NOUNS
 * ========================================================================= */

typedef enum {
    APOLLO_NOUN_01 = 1,
    APOLLO_NOUN_20 = 20,
    APOLLO_NOUN_21 = 21,
    APOLLO_NOUN_22 = 22,
    APOLLO_NOUN_30 = 30,
    APOLLO_NOUN_31 = 31,
    APOLLO_NOUN_32 = 32,
    APOLLO_NOUN_33 = 33,
    APOLLO_NOUN_38 = 38,
    APOLLO_NOUN_40 = 40,
    APOLLO_NOUN_41 = 41,
    APOLLO_NOUN_42 = 42,
    APOLLO_NOUN_43 = 43,
    APOLLO_NOUN_44 = 44,
    APOLLO_NOUN_45 = 45,
    APOLLO_NOUN_46 = 46,
    APOLLO_NOUN_47 = 47,
    APOLLO_NOUN_48 = 48,
    APOLLO_NOUN_49 = 49,
    APOLLO_NOUN_50 = 50,
    APOLLO_NOUN_51 = 51,
    APOLLO_NOUN_52 = 52,
    APOLLO_NOUN_53 = 53,
    APOLLO_NOUN_54 = 54,
    APOLLO_NOUN_55 = 55,
    APOLLO_NOUN_56 = 56,
    APOLLO_NOUN_57 = 57,
    APOLLO_NOUN_58 = 58,
    APOLLO_NOUN_59 = 59,
    APOLLO_NOUN_60 = 60,
    APOLLO_NOUN_61 = 61,
    APOLLO_NOUN_62 = 62,
    APOLLO_NOUN_63 = 63,
    APOLLO_NOUN_64 = 64,
    APOLLO_NOUN_65 = 65,
    APOLLO_NOUN_66 = 66,
    APOLLO_NOUN_67 = 67,
    APOLLO_NOUN_68 = 68,
    APOLLO_NOUN_69 = 69
} apollo_noun_t;

/* =========================================================================
 * DISPLAY DATA
 * ========================================================================= */

typedef struct {
    int16_t verb;
    int16_t noun;
    int16_t r1[3];
    int16_t r2[3];
    int16_t r3[3];
} apollo_display_data_t;

void apollo_display_init(void);
void apollo_display_update(void);
void apollo_set_verb_noun(int verb, int noun);
void apollo_set_register(int reg, const int16_t *data);
void apollo_clear_display(void);

#endif /* APOLLO_DISPLAY_H */
