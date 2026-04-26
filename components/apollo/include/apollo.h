/**
 * @file apollo.h
 * @brief Main header for Apollo Guidance Computer C Port
 *
 * =========================================================================
 * Apollo-11 Command Module Software (Comanche 055)
 * =========================================================================
 *
 * This is a C port of the original Apollo 11 Command Module guidance computer
 * source code. The original was written in AGC assembly language for the
 * MIT Instrumentation Laboratory's Apollo Guidance Computer (AGC).
 *
 * Project Structure:
 *   include/     - Public headers
 *   src/         - Implementation files
 *   tests/       - Test programs
 *
 * Modules:
 *   apollo_types  - Base types and constants
 *   apollo_math  - Mathematical operations
 *   apollo_vecmath - Vector and matrix operations
 *   apollo_orbit  - Orbital mechanics
 *   apollo_attitude - Attitude determination and control
 *   apollo_nav    - Navigation functions
 *   apollo_guidance - Guidance algorithms
 *   apollo_entry  - Reentry control
 *   apollo_tvc    - Thrust Vector Control
 *   apollo_rcs    - Reaction Control System
 *   apollo_dap    - Digital Autopilot
 *   apollo_display - Display and keyboard interface
 *   apollo_io     - Input/Output
 *   apollo_exec   - Executive (task scheduling)
 *   apollo_waitlist - Waitlist (timing)
 *   apollo interpreter - Interpretive language
 *
 * Original Source:
 *   NASA, Public Domain
 *   Comanche 055 - Command Module AGC Software
 *   Assemble revision 055 of AGC program Comanche by NASA
 *   2021113-051. 10:28 APR. 1, 1969
 *
 * Digitized by:
 *   Virtual AGC - http://www.ibiblio.org/apollo/
 *   MIT Museum
 */

#ifndef APOLLO_H
#define APOLLO_H

/*
 * Include all module headers
 */
#include "apollo_types.h"
#include "apollo_math.h"
#include "apollo_vecmath.h"
#include "apollo_orbit.h"
#include "apollo_attitude.h"
#include "apollo_nav.h"
#include "apollo_guidance.h"
#include "apollo_entry.h"
#include "apollo_tvc.h"
#include "apollo_rcs.h"
#include "apollo_dap.h"
#include "apollo_display.h"
#include "apollo_io.h"
#include "apollo_exec.h"
#include "apollo_waitlist.h"
#include "apollo_interpreter.h"

#endif /* APOLLO_H */
