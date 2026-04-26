/**
 * @file apollo_types.h
 * @brief Base types and constants for Apollo Guidance Computer
 *
 * =========================================================================
 * ORIGINAL AGC DATA REPRESENTATION
 * =========================================================================
 *
 * The AGC used several data representations:
 *
 * 1. SINGLE PRECISION (SP) - 15-bit signed integer + 1 sign bit
 *    Range: -32768 to +32767 (approximately 4 decimal digits)
 *
 * 2. DOUBLE PRECISION (DP) - Two SP words
 *    Range: approximately 10 decimal digits
 *
 * 3. VECTOR - Three consecutive DP words (X, Y, Z)
 *
 * 4. MATRIX - Nine consecutive DP words (3x3)
 *
 * All data was stored in core memory with specific addressing.
 *
 * =========================================================================
 * C PORT DESIGN
 * =========================================================================
 *
 * This C port uses:
 *   - int16_t/int32_t for AGC single/double precision integers
 *   - double for floating point (IEEE 754)
 *   - Structures for vectors and matrices
 *
 * Scale factors are preserved as constants for documentation.
 */

#ifndef APOLLO_TYPES_H
#define APOLLO_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * AGC CONSTANTS
 * ========================================================================= */

/**
 * @name AGC Physical Constants
 * @{
 */

/** @brief Earth gravitational parameter (m^3/s^2) */
#define APOLLO_MU_EARTH  3.986004418e14

/** @brief Moon gravitational parameter (m^3/s^2) */
#define APOLLO_MU_MOON   4.9048695e12

/** @brief Earth equatorial radius (meters) */
#define APOLLO_R_EARTH   6.378137e6

/** @brief Moon radius (meters) */
#define APOLLO_R_MOON    1.7371e6

/** @brief Speed of light (meters/second) */
#define APOLLO_C         2.997925e8

/** @brief Astronomical Unit (meters) */
#define APOLLO_AU        1.4959787066e11

/** @} */

/**
 * @name AGC Time System
 * @{
 */

/** @brief Centiseconds per second */
#define APOLLO_CSEC_PER_SEC  100

/** @brief Seconds per minute */
#define APOLLO_SEC_PER_MIN   60

/** @brief Minutes per hour */
#define APOLLO_MIN_PER_HR    60

/** @brief Hours per day */
#define APOLLO_HR_PER_DAY    24

/** @brief Centiseconds per minute */
#define APOLLO_CSEC_PER_MIN   (APOLLO_CSEC_PER_SEC * APOLLO_SEC_PER_MIN)

/** @brief Centiseconds per hour */
#define APOLLO_CSEC_PER_HR    (APOLLO_CSEC_PER_MIN * APOLLO_MIN_PER_HR)

/** @brief Centiseconds per day */
#define APOLLO_CSEC_PER_DAY   (APOLLO_CSEC_PER_HR * APOLLO_HR_PER_DAY)

/** @} */

/**
 * @name Angular Conversions
 * @{
 */

/** @brief Degrees to radians */
#define APOLLO_D2R  0.017453292519943295

/** @brief Radians to degrees */
#define APOLLO_R2D  57.295779513082323

/** @brief AGC units per degree (2^14 / 360) */
#define APOLLO_UNITS_PER_DEG  182.04444444444444

/** @brief Arcseconds to radians */
#define APOLLO_ARCSEC2RAD  4.848136811095360e-6

/** @} */

/**
 * @name AGC Scale Factors (Powers of 2)
 * @{
 */

/** Scale factor for position vectors (Earth): +29 */
#define APOLLO_SF_POS_EARTH  29

/** Scale factor for position vectors (Moon): +27 */
#define APOLLO_SF_POS_MOON   27

/** Scale factor for velocity vectors (Earth): +7 */
#define APOLLO_SF_VEL_EARTH  7

/** Scale factor for velocity vectors (Moon): +5 */
#define APOLLO_SF_VEL_MOON   5

/** @brief Full circle in AGC units (2^14) */
#define APOLLO_2PI_FIXED  61440

/** @brief PI in AGC fixed point (half of 2^14) */
#define APOLLO_PI_FIXED   32768

/** @brief PI/2 in AGC fixed point */
#define APOLLO_HALF_PI_FIXED 16384

/** @} */

/* =========================================================================
 * DATA TYPES
 * ========================================================================= */

/**
 * @brief Single precision integer (AGC 15-bit + sign)
 *
 * Range: -32768 to +32767
 */
typedef int16_t apollo_sp_t;

/**
 * @brief Double precision structure (two SP words)
 */
typedef struct {
    int16_t high;   /**< High word */
    int16_t low;    /**< Low word */
} apollo_dp_t;

/**
 * @brief Union for DP access
 */
typedef union {
    apollo_dp_t words;
    int32_t full;
    double value;
} apollo_dp_union_t;

/**
 * @brief AGC index register
 *
 * Used for bank switching and addressing.
 * Range: -32 to +32 typically
 */
typedef int8_t apollo_index_t;

/**
 * @name AGC Bank Numbers
 * @{
 */

/** @brief Erasable banks (0-7) */
#define APOLLO_ERASE_BANKS  8

/** @brief Fixed banks (10-37 octal) */
#define APOLLO_FIXED_BANKS   24

/** @} */

/* =========================================================================
 * TIME REPRESENTATION
 * ========================================================================= */

/**
 * @brief AGC time representation (centiseconds)
 *
 * Stored as two double-precision words: TIME2 (high) and TIME1 (low)
 */
typedef struct {
    int16_t time2;   /**< High part (centiseconds) */
    int16_t time1;   /**< Low part (centiseconds) */
} apollo_time_t;

/**
 * @brief Convert centiseconds to seconds
 */
static inline double apollo_csec_to_sec(int32_t csec)
{
    return (double)csec / 100.0;
}

/**
 * @brief Convert seconds to centiseconds
 */
static inline int32_t apollo_sec_to_csec(double sec)
{
    return (int32_t)(sec * 100.0);
}

/* =========================================================================
 * SWITCHES AND FLAGS
 * ========================================================================= */

/**
 * @brief Interpreter switch
 *
 * Single-bit flags used by the interpreter.
 */
typedef struct {
    bool flag1 : 1;
    bool flag2 : 1;
    bool flag3 : 1;
    bool flag4 : 1;
    bool flag5 : 1;
    bool flag6 : 1;
    bool flag7 : 1;
    bool flag8 : 1;
    bool flag9 : 1;
    bool flag10 : 1;
    bool flag11 : 1;
    bool flag12 : 1;
    bool flag13 : 1;
    bool flag14 : 1;
    bool flag15 : 1;
    bool flag16 : 1;
} apollo_switches_t;

/**
 * @name AGC Input Channels
 * @{
 */

/** @brief Channel 0 - Keyboard/Display */
#define APOLLO_CHAN0  0

/** @brief Channel 1 - Interrupt enables */
#define APOLLO_CHAN1  1

/** @brief Channel 2 - Counter/Timer */
#define APOLLO_CHAN2  2

/** @brief Channel 3 - Analog inputs */
#define APOLLO_CHAN3  3

/** @brief Channel 4 - Discrete outputs */
#define APOLLO_CHAN4  4

/** @brief Channel 5 - Telemetry */
#define APOLLO_CHAN5  5

/** @} */

/* =========================================================================
 * AGC MEMORY SECTIONS
 * ========================================================================= */

/**
 * @name Erasable Memory Layout
 * @{
 */

/** @brief Start of common block */
#define APOLLO_COMMON_START  0x0000

/** @brief End of common block */
#define APOLLO_COMMON_END    0x17FF

/** @brief Start of EBANK (erasable bank) */
#define APOLLO_EBANK_START   0x1400

/** @brief End of EBANK */
#define APOLLO_EBANK_END     0x17FF

/** @} */

/* =========================================================================
 * STATUS AND ERROR CODES
 * ========================================================================= */

/**
 * @name AGC Alarm Codes
 * @{
 */

/** @brief Alarm: No alarm (normal) */
#define APOLLO_ALARM_NONE       0

/** @brief Alarm: IMU not operational */
#define APOLLO_ALARM_IMU_INOP   00220

/** @brief Alarm: No-attitude alarm */
#define APOLLO_ALARM_NO_ATT     00606

/** @brief Alarm: Tracker alarm */
#define APOLLO_ALARM_TRACKER    01103

/** @brief Alarm: Navigation alarm */
#define APOLLO_ALARM_NAV        01505

/** @brief Alarm: Guidance alarm */
#define APOLLO_ALARM_GUIDANCE   02107

/** @} */

/* =========================================================================
 * MATHEMATICAL CONSTANTS
 * ========================================================================= */

/** @brief PI to maximum precision */
#define APOLLO_PI        3.14159265358979323846264338327950288

/** @brief 2*PI */
#define APOLLO_TWO_PI    6.28318530717958647692528676655900576

/** @brief PI/2 */
#define APOLLO_HALF_PI   1.57079632679489661923132169163975144

/** @brief PI/4 */
#define APOLLO_QUARTER_PI 0.78539816339744830961566084581987572

/** @brief Square root of 2 */
#define APOLLO_SQRT2     1.41421356237309504880168872420969808

/** @brief Natural log of 2 */
#define APOLLO_LN2       0.69314718055994530941723212145817657

/** @brief Euler's number */
#define APOLLO_E         2.71828182845904523536028747135266250

/** @brief Golden ratio */
#define APOLLO_PHI       1.61803398874989484820458683436563812

#endif /* APOLLO_TYPES_H */
