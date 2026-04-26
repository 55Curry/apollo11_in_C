/**
 * @file apollo_math.h
 * @brief Apollo Guidance Computer mathematical routines - C port
 * @author Ported from AGC assembly (Comanche 055)
 * @note Original code: NASA, Public Domain
 *
 * This is a C port of the single-precision sine and cosine routines
 * from the Apollo 11 Command Module computer (AGC).
 *
 * The original AGC implementation (page 1207):
 *   SPCOS  AD      HALF          # Cosine = sine shifted by PI/2
 *   SPSIN  TS      TEMK          # Store argument
 *          ...
 *   POLLEY polynomial approximation
 *
 * This C port preserves the original algorithm structure:
 *   1. Argument reduction to [0, 2*PI]
 *   2. Quadrant mapping to [0, PI/2]
 *   3. Complementary angle formula near PI/2
 *
 * Usage:
 *   double s = apollo_spsin(angle_radians);  // sine
 *   double c = apollo_spcos(angle_radians);  // cosine
 */

#ifndef APOLLO_MATH_H
#define APOLLO_MATH_H

/**
 * @brief Compute single-precision sine (SPSIN)
 * @param radians Angle in radians
 * @return Sine value
 *
 * Original AGC: Argument scaled at PI (full circle = 2^14 units)
 * This C port: Argument in radians (standard convention)
 */
double apollo_spsin(double radians);

/**
 * @brief Compute single-precision cosine (SPCOS)
 * @param radians Angle in radians
 * @return Cosine value
 *
 * Original AGC: Implemented as sine with PI/2 shift
 * This C port: cos(x) = sin(x + PI/2)
 */
double apollo_spcos(double radians);

#endif /* APOLLO_MATH_H */
