/**
 * @file apollo_vecmath.h
 * @brief Vector and Matrix mathematics for Apollo Guidance Computer
 *
 * =========================================================================
 * ORIGINAL AGC VECTOR/MATRIX OPERATIONS (INTERPRETER.agc, CSM_GEOMETRY.agc)
 * =========================================================================
 *
 * The AGC used a vector instruction set implemented in the interpreter.
 * Key operations:
 *   VAD    - Vector addition
 *   VSUB   - Vector subtraction
 *   VXSC   - Vector times scalar
 *   DOT    - Dot product
 *   VXV    - Vector cross product
 *   UNIT   - Unit vector
 *   MXV    - Matrix times vector
 *   MXM    - Matrix times matrix
 *   TRANSP - Transpose matrix
 *
 * AGC used DP fixed-point with scaling. Vectors were 3-component (X,Y,Z).
 * Matrices were 3x3.
 *
 * =========================================================================
 * C PORT
 * =========================================================================
 *
 * This port uses double-precision floating point for accuracy.
 * Vectors and matrices are represented as arrays for cache efficiency.
 *
 * Naming convention follows original AGC:
 *   - Functions prefixed with apollo_v3_ (3-component vector)
 *   - Functions prefixed with apollo_mat3_ (3x3 matrix)
 */

#ifndef APOLLO_VECMATH_H
#define APOLLO_VECMATH_H

#include <stddef.h>
#include <stdbool.h>

/* =========================================================================
 * TYPE DEFINITIONS
 * ========================================================================= */

/**
 * @brief 3-component vector
 *
 * Layout: [X, Y, Z]
 */
typedef struct {
    double x;
    double y;
    double z;
} apollo_v3_t;

/**
 * @brief 3x3 matrix in row-major order
 *
 * Layout:
 *   [m[0] m[1] m[2]]   <- row 0
 *   [m[3] m[4] m[5]]   <- row 1
 *   [m[6] m[7] m[8]]   <- row 2
 *
 * Element access: m[row * 3 + col]
 */
typedef struct {
    double m[9];
} apollo_mat3_t;

/* =========================================================================
 * VECTOR ARITHMETIC
 * ========================================================================= */

/**
 * @brief Vector addition: c = a + b
 */
apollo_v3_t apollo_v3_add(const apollo_v3_t *a, const apollo_v3_t *b);

/**
 * @brief Vector subtraction: c = a - b
 */
apollo_v3_t apollo_v3_sub(const apollo_v3_t *a, const apollo_v3_t *b);

/**
 * @brief Vector scaled by scalar: c = a * s
 */
apollo_v3_t apollo_v3_scale(const apollo_v3_t *a, double s);

/**
 * @brief Vector scaled and added: c = a * s + b
 */
apollo_v3_t apollo_v3_scale_add(const apollo_v3_t *a, double s, const apollo_v3_t *b);

/**
 * @brief Dot product: a · b
 */
double apollo_v3_dot(const apollo_v3_t *a, const apollo_v3_t *b);

/**
 * @brief Cross product: c = a × b
 *
 * Note: Cross product is NOT commutative.
 *   a × b = -(b × a)
 */
apollo_v3_t apollo_v3_cross(const apollo_v3_t *a, const apollo_v3_t *b);

/**
 * @brief Vector squared magnitude: |a|²
 */
double apollo_v3_mag_squared(const apollo_v3_t *a);

/**
 * @brief Vector magnitude: |a|
 */
double apollo_v3_mag(const apollo_v3_t *a);

/**
 * @brief Normalize vector to unit length
 *
 * Returns zero vector if input magnitude is zero.
 * Uses hypot() for numerical stability.
 */
apollo_v3_t apollo_v3_unit(const apollo_v3_t *a);

/**
 * @brief Check if two vectors are approximately equal
 */
bool apollo_v3_equal(const apollo_v3_t *a, const apollo_v3_t *b, double epsilon);

/* =========================================================================
 * VECTOR CONSTANTS
 * ========================================================================= */

/**
 * @brief Unit vectors
 */
extern const apollo_v3_t apollo_v3_zero;    /* (0, 0, 0) */
extern const apollo_v3_t apollo_v3_unit_x;  /* (1, 0, 0) */
extern const apollo_v3_t apollo_v3_unit_y;  /* (0, 1, 0) */
extern const apollo_v3_t apollo_v3_unit_z;  /* (0, 0, 1) */

/* =========================================================================
 * MATRIX OPERATIONS
 * ========================================================================= */

/**
 * @brief Matrix-vector multiply: y = A * x
 *
 * Computes:
 *   y[0] = A[0]*x[0] + A[1]*x[1] + A[2]*x[2]
 *   y[1] = A[3]*x[0] + A[4]*x[1] + A[5]*x[2]
 *   y[2] = A[6]*x[0] + A[7]*x[1] + A[8]*x[2]
 */
apollo_v3_t apollo_mat3_mul_v3(const apollo_mat3_t *A, const apollo_v3_t *x);

/**
 * @brief Matrix-matrix multiply: C = A * B
 *
 * Note: Matrix multiplication is NOT commutative.
 *   A * B ≠ B * A in general
 */
apollo_mat3_t apollo_mat3_mul(const apollo_mat3_t *A, const apollo_mat3_t *B);

/**
 * @brief Transpose matrix: B = A^T
 *
 * For 3x3 matrix: B[i][j] = A[j][i]
 */
apollo_mat3_t apollo_mat3_transpose(const apollo_mat3_t *A);

/**
 * @brief Matrix scaled by scalar
 */
apollo_mat3_t apollo_mat3_scale(const apollo_mat3_t *A, double s);

/**
 * @brief Matrix plus matrix: C = A + B
 */
apollo_mat3_t apollo_mat3_add(const apollo_mat3_t *A, const apollo_mat3_t *B);

/**
 * @brief Identity matrix
 */
extern const apollo_mat3_t apollo_mat3_identity;

/**
 * @brief Create rotation matrix around X axis
 * @param angle_rad Rotation angle in radians
 */
apollo_mat3_t apollo_mat3_rot_x(double angle_rad);

/**
 * @brief Create rotation matrix around Y axis
 * @param angle_rad Rotation angle in radians
 */
apollo_mat3_t apollo_mat3_rot_y(double angle_rad);

/**
 * @brief Create rotation matrix around Z axis
 * @param angle_rad Rotation angle in radians
 */
apollo_mat3_t apollo_mat3_rot_z(double angle_rad);

/* =========================================================================
 * UTILITY FUNCTIONS
 * ========================================================================= */

/**
 * @brief Copy vector
 */
void apollo_v3_copy(apollo_v3_t *dest, const apollo_v3_t *src);

/**
 * @brief Copy matrix
 */
void apollo_mat3_copy(apollo_mat3_t *dest, const apollo_mat3_t *src);

/**
 * @brief Print vector to stdout
 */
void apollo_v3_print(const apollo_v3_t *v, const char *name);

/**
 * @brief Print matrix to stdout
 */
void apollo_mat3_print(const apollo_mat3_t *m, const char *name);

#endif /* APOLLO_VECMATH_H */
