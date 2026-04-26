/**
 * @file apollo_vecmath.c
 * @brief Implementation of vector and matrix operations
 *
 * =========================================================================
 * ORIGINAL AGC VECTOR INSTRUCTION SET
 * =========================================================================
 *
 * The AGC interpreter implemented vector operations using fixed-point
 * DP arithmetic. Key characteristics:
 *
 *   - Vectors: 3 components (X, Y, Z) stored consecutively
 *   - Matrices: 3x3, stored in row-major order
 *   - Scaling: DP with various scale factors depending on context
 *   - Precision: 14-bit mantissa (approximately 4 decimal digits)
 *
 * =========================================================================
 * C PORT DESIGN
 * =========================================================================
 *
 * This implementation:
 *   - Uses double-precision floating point
 *   - Follows original AGC semantics where applicable
 *   - Prioritizes numerical stability (e.g., hypot for magnitude)
 *   - Maintains readability and correspondence to original code
 *
 * =========================================================================
 */

#include "apollo_vecmath.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

/* =========================================================================
 * VECTOR CONSTANTS
 * ========================================================================= */

const apollo_v3_t apollo_v3_zero  = { .x = 0.0, .y = 0.0, .z = 0.0 };
const apollo_v3_t apollo_v3_unit_x = { .x = 1.0, .y = 0.0, .z = 0.0 };
const apollo_v3_t apollo_v3_unit_y = { .x = 0.0, .y = 1.0, .z = 0.0 };
const apollo_v3_t apollo_v3_unit_z = { .x = 0.0, .y = 0.0, .z = 1.0 };

/* =========================================================================
 * MATRIX CONSTANTS
 * ========================================================================= */

const apollo_mat3_t apollo_mat3_identity = {
    .m = { 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0 }
};

/* =========================================================================
 * VECTOR ARITHMETIC
 * ========================================================================= */

apollo_v3_t apollo_v3_add(const apollo_v3_t *a, const apollo_v3_t *b)
{
    apollo_v3_t c;
    c.x = a->x + b->x;
    c.y = a->y + b->y;
    c.z = a->z + b->z;
    return c;
}

apollo_v3_t apollo_v3_sub(const apollo_v3_t *a, const apollo_v3_t *b)
{
    apollo_v3_t c;
    c.x = a->x - b->x;
    c.y = a->y - b->y;
    c.z = a->z - b->z;
    return c;
}

apollo_v3_t apollo_v3_scale(const apollo_v3_t *a, double s)
{
    apollo_v3_t c;
    c.x = a->x * s;
    c.y = a->y * s;
    c.z = a->z * s;
    return c;
}

apollo_v3_t apollo_v3_scale_add(const apollo_v3_t *a, double s, const apollo_v3_t *b)
{
    apollo_v3_t c;
    c.x = a->x * s + b->x;
    c.y = a->y * s + b->y;
    c.z = a->z * s + b->z;
    return c;
}

double apollo_v3_dot(const apollo_v3_t *a, const apollo_v3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

apollo_v3_t apollo_v3_cross(const apollo_v3_t *a, const apollo_v3_t *b)
{
    apollo_v3_t c;
    /*
     * Original AGC: Cross product
     *   c.x = a.y * b.z - a.z * b.y
     *   c.y = a.z * b.x - a.x * b.z
     *   c.z = a.x * b.y - a.y * b.x
     *
     * Using the right-hand rule:
     *   X × Y = Z, Y × Z = X, Z × X = Y
     */
    c.x = a->y * b->z - a->z * b->y;
    c.y = a->z * b->x - a->x * b->z;
    c.z = a->x * b->y - a->y * b->x;
    return c;
}

double apollo_v3_mag_squared(const apollo_v3_t *a)
{
    return apollo_v3_dot(a, a);
}

double apollo_v3_mag(const apollo_v3_t *a)
{
    /*
     * Use hypot for numerical stability.
     * hypot(x, y, z) = sqrt(x² + y² + z²) without overflow
     *
     * Original AGC used iterative method which was more expensive
     * but handled edge cases.
     */
    return hypot(hypot(a->x, a->y), a->z);
}

apollo_v3_t apollo_v3_unit(const apollo_v3_t *a)
{
    apollo_v3_t u;
    double mag = apollo_v3_mag(a);

    if (mag == 0.0) {
        /* Zero vector - return zero unit (same as AGC behavior) */
        u.x = 0.0;
        u.y = 0.0;
        u.z = 0.0;
    } else {
        double inv_mag = 1.0 / mag;
        u.x = a->x * inv_mag;
        u.y = a->y * inv_mag;
        u.z = a->z * inv_mag;
    }

    return u;
}

bool apollo_v3_equal(const apollo_v3_t *a, const apollo_v3_t *b, double epsilon)
{
    return fabs(a->x - b->x) < epsilon
        && fabs(a->y - b->y) < epsilon
        && fabs(a->z - b->z) < epsilon;
}

void apollo_v3_copy(apollo_v3_t *dest, const apollo_v3_t *src)
{
    dest->x = src->x;
    dest->y = src->y;
    dest->z = src->z;
}

void apollo_v3_print(const apollo_v3_t *v, const char *name)
{
    if (name) {
        printf("%s = (%.6f, %.6f, %.6f)\n", name, v->x, v->y, v->z);
    } else {
        printf("(%.6f, %.6f, %.6f)\n", v->x, v->y, v->z);
    }
}

/* =========================================================================
 * MATRIX OPERATIONS
 * ========================================================================= */

apollo_v3_t apollo_mat3_mul_v3(const apollo_mat3_t *A, const apollo_v3_t *x)
{
    apollo_v3_t y;
    /*
     * Original AGC MXV instruction:
     *   y[i] = sum(A[i][j] * x[j]) for j = 0,1,2
     *
     * Matrix layout (row-major):
     *   A[0] A[1] A[2]   Row 0
     *   A[3] A[4] A[5]   Row 1
     *   A[6] A[7] A[8]   Row 2
     */
    y.x = A->m[0] * x->x + A->m[1] * x->y + A->m[2] * x->z;
    y.y = A->m[3] * x->x + A->m[4] * x->y + A->m[5] * x->z;
    y.z = A->m[6] * x->x + A->m[7] * x->y + A->m[8] * x->z;
    return y;
}

apollo_mat3_t apollo_mat3_mul(const apollo_mat3_t *A, const apollo_mat3_t *B)
{
    apollo_mat3_t C;
    /*
     * Original AGC MXM instruction:
     *   C[i][j] = sum(A[i][k] * B[k][j]) for k = 0,1,2
     */
    C.m[0] = A->m[0] * B->m[0] + A->m[1] * B->m[3] + A->m[2] * B->m[6];
    C.m[1] = A->m[0] * B->m[1] + A->m[1] * B->m[4] + A->m[2] * B->m[7];
    C.m[2] = A->m[0] * B->m[2] + A->m[1] * B->m[5] + A->m[2] * B->m[8];

    C.m[3] = A->m[3] * B->m[0] + A->m[4] * B->m[3] + A->m[5] * B->m[6];
    C.m[4] = A->m[3] * B->m[1] + A->m[4] * B->m[4] + A->m[5] * B->m[7];
    C.m[5] = A->m[3] * B->m[2] + A->m[4] * B->m[5] + A->m[5] * B->m[8];

    C.m[6] = A->m[6] * B->m[0] + A->m[7] * B->m[3] + A->m[8] * B->m[6];
    C.m[7] = A->m[6] * B->m[1] + A->m[7] * B->m[4] + A->m[8] * B->m[7];
    C.m[8] = A->m[6] * B->m[2] + A->m[7] * B->m[5] + A->m[8] * B->m[8];

    return C;
}

apollo_mat3_t apollo_mat3_transpose(const apollo_mat3_t *A)
{
    apollo_mat3_t B;
    /*
     * Original AGC TRANSP instruction:
     *   B[i][j] = A[j][i]
     *
     * For 3x3 matrix:
     *   B[1] = A[3], B[3] = A[1]
     *   B[2] = A[6], B[6] = A[2]
     *   B[5] = A[7], B[7] = A[5]
     */
    B.m[0] = A->m[0];
    B.m[1] = A->m[3];
    B.m[2] = A->m[6];

    B.m[3] = A->m[1];
    B.m[4] = A->m[4];
    B.m[5] = A->m[7];

    B.m[6] = A->m[2];
    B.m[7] = A->m[5];
    B.m[8] = A->m[8];

    return B;
}

apollo_mat3_t apollo_mat3_scale(const apollo_mat3_t *A, double s)
{
    apollo_mat3_t B;
    for (int i = 0; i < 9; i++) {
        B.m[i] = A->m[i] * s;
    }
    return B;
}

apollo_mat3_t apollo_mat3_add(const apollo_mat3_t *A, const apollo_mat3_t *B)
{
    apollo_mat3_t C;
    for (int i = 0; i < 9; i++) {
        C.m[i] = A->m[i] + B->m[i];
    }
    return C;
}

apollo_mat3_t apollo_mat3_rot_x(double angle_rad)
{
    apollo_mat3_t R;
    double c = cos(angle_rad);
    double s = sin(angle_rad);
    /*
     * Rotation around X axis (right-hand rule):
     *   1    0    0
     *   0   cos  -sin
     *   0   sin   cos
     */
    R.m[0] = 1.0;  R.m[1] = 0.0;  R.m[2] = 0.0;
    R.m[3] = 0.0;  R.m[4] = c;    R.m[5] = -s;
    R.m[6] = 0.0;  R.m[7] = s;    R.m[8] = c;
    return R;
}

apollo_mat3_t apollo_mat3_rot_y(double angle_rad)
{
    apollo_mat3_t R;
    double c = cos(angle_rad);
    double s = sin(angle_rad);
    /*
     * Rotation around Y axis (right-hand rule):
     *   cos   0    sin
     *   0     1    0
     *  -sin   0    cos
     */
    R.m[0] = c;    R.m[1] = 0.0;  R.m[2] = s;
    R.m[3] = 0.0;  R.m[4] = 1.0;  R.m[5] = 0.0;
    R.m[6] = -s;   R.m[7] = 0.0;  R.m[8] = c;
    return R;
}

apollo_mat3_t apollo_mat3_rot_z(double angle_rad)
{
    apollo_mat3_t R;
    double c = cos(angle_rad);
    double s = sin(angle_rad);
    /*
     * Rotation around Z axis (right-hand rule):
     *   cos  -sin   0
     *   sin   cos   0
     *   0     0     1
     */
    R.m[0] = c;    R.m[1] = -s;   R.m[2] = 0.0;
    R.m[3] = s;    R.m[4] = c;    R.m[5] = 0.0;
    R.m[6] = 0.0;  R.m[7] = 0.0;  R.m[8] = 1.0;
    return R;
}

void apollo_mat3_copy(apollo_mat3_t *dest, const apollo_mat3_t *src)
{
    for (int i = 0; i < 9; i++) {
        dest->m[i] = src->m[i];
    }
}

void apollo_mat3_print(const apollo_mat3_t *m, const char *name)
{
    if (name) {
        printf("%s:\n", name);
    }
    printf("  [%.6f  %.6f  %.6f]\n", m->m[0], m->m[1], m->m[2]);
    printf("  [%.6f  %.6f  %.6f]\n", m->m[3], m->m[4], m->m[5]);
    printf("  [%.6f  %.6f  %.6f]\n", m->m[6], m->m[7], m->m[8]);
}

/* =========================================================================
 * TEST DRIVER
 * ========================================================================= */

#ifdef APOLLO_VECMATH_TEST

#include <stdio.h>
#include <math.h>

#define TEST_PRECISION 1e-10

static int test_v3_add(void)
{
    apollo_v3_t a = { .x = 1.0, .y = 2.0, .z = 3.0 };
    apollo_v3_t b = { .x = 4.0, .y = 5.0, .z = 6.0 };
    apollo_v3_t c = apollo_v3_add(&a, &b);

    printf("v3_add: (%.2f, %.2f, %.2f) + (%.2f, %.2f, %.2f) = (%.2f, %.2f, %.2f)\n",
           a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);

    return (fabs(c.x - 5.0) < TEST_PRECISION &&
            fabs(c.y - 7.0) < TEST_PRECISION &&
            fabs(c.z - 9.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_v3_sub(void)
{
    apollo_v3_t a = { .x = 4.0, .y = 5.0, .z = 6.0 };
    apollo_v3_t b = { .x = 1.0, .y = 2.0, .z = 3.0 };
    apollo_v3_t c = apollo_v3_sub(&a, &b);

    printf("v3_sub: (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f) = (%.2f, %.2f, %.2f)\n",
           a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);

    return (fabs(c.x - 3.0) < TEST_PRECISION &&
            fabs(c.y - 3.0) < TEST_PRECISION &&
            fabs(c.z - 3.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_v3_scale(void)
{
    apollo_v3_t a = { .x = 1.0, .y = 2.0, .z = 3.0 };
    apollo_v3_t c = apollo_v3_scale(&a, 2.0);

    printf("v3_scale: (%.2f, %.2f, %.2f) * 2 = (%.2f, %.2f, %.2f)\n",
           a.x, a.y, a.z, c.x, c.y, c.z);

    return (fabs(c.x - 2.0) < TEST_PRECISION &&
            fabs(c.y - 4.0) < TEST_PRECISION &&
            fabs(c.z - 6.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_v3_dot(void)
{
    apollo_v3_t a = { .x = 1.0, .y = 2.0, .z = 3.0 };
    apollo_v3_t b = { .x = 4.0, .y = 5.0, .z = 6.0 };
    double d = apollo_v3_dot(&a, &b);

    printf("v3_dot: (%.2f, %.2f, %.2f) . (%.2f, %.2f, %.2f) = %.2f\n",
           a.x, a.y, a.z, b.x, b.y, b.z, d);

    /* 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32 */
    return fabs(d - 32.0) < TEST_PRECISION ? 0 : 1;
}

static int test_v3_cross(void)
{
    apollo_v3_t x = { .x = 1.0, .y = 0.0, .z = 0.0 };
    apollo_v3_t y = { .x = 0.0, .y = 1.0, .z = 0.0 };
    apollo_v3_t z = apollo_v3_cross(&x, &y);

    printf("v3_cross: (1,0,0) x (0,1,0) = (%.2f, %.2f, %.2f)\n", z.x, z.y, z.z);

    /* X × Y = Z */
    return (fabs(z.x - 0.0) < TEST_PRECISION &&
            fabs(z.y - 0.0) < TEST_PRECISION &&
            fabs(z.z - 1.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_v3_mag(void)
{
    apollo_v3_t v = { .x = 3.0, .y = 4.0, .z = 0.0 };
    double mag = apollo_v3_mag(&v);

    printf("v3_mag: |(3,4,0)| = %.2f\n", mag);

    /* sqrt(9 + 16) = 5 */
    return fabs(mag - 5.0) < TEST_PRECISION ? 0 : 1;
}

static int test_v3_unit(void)
{
    apollo_v3_t v = { .x = 3.0, .y = 4.0, .z = 0.0 };
    apollo_v3_t u = apollo_v3_unit(&v);

    printf("v3_unit: (3,4,0) -> (%.6f, %.6f, %.6f)\n", u.x, u.y, u.z);

    /* Should be (0.6, 0.8, 0) */
    return (fabs(u.x - 0.6) < TEST_PRECISION &&
            fabs(u.y - 0.8) < TEST_PRECISION &&
            fabs(u.z - 0.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_mat3_mul_v3(void)
{
    apollo_mat3_t A = {
        .m = { 1.0, 2.0, 3.0,
               4.0, 5.0, 6.0,
               7.0, 8.0, 9.0 }
    };
    apollo_v3_t x = { .x = 1.0, .y = 0.0, .z = 0.0 };
    apollo_v3_t y = apollo_mat3_mul_v3(&A, &x);

    printf("mat3_mul_v3: A * (1,0,0) = (%.2f, %.2f, %.2f)\n", y.x, y.y, y.z);

    /* First column of A */
    return (fabs(y.x - 1.0) < TEST_PRECISION &&
            fabs(y.y - 4.0) < TEST_PRECISION &&
            fabs(y.z - 7.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_mat3_mul(void)
{
    apollo_mat3_t A = {
        .m = { 1.0, 2.0, 3.0,
               4.0, 5.0, 6.0,
               7.0, 8.0, 9.0 }
    };
    apollo_mat3_t I = apollo_mat3_identity;
    apollo_mat3_t C = apollo_mat3_mul(&A, &I);

    printf("mat3_mul: A * I = A (identity check)\n");

    /* A * I = A */
    return (fabs(C.m[0] - 1.0) < TEST_PRECISION &&
            fabs(C.m[4] - 5.0) < TEST_PRECISION &&
            fabs(C.m[8] - 9.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_mat3_transpose(void)
{
    apollo_mat3_t A = {
        .m = { 1.0, 2.0, 3.0,
               4.0, 5.0, 6.0,
               7.0, 8.0, 9.0 }
    };
    apollo_mat3_t B = apollo_mat3_transpose(&A);

    printf("mat3_transpose:\n");
    printf("  Original: row0=(1,2,3), row1=(4,5,6), row2=(7,8,9)\n");
    printf("  Transpose: row0=(1,4,7), row1=(2,5,8), row2=(3,6,9)\n");

    return (fabs(B.m[1] - 4.0) < TEST_PRECISION &&
            fabs(B.m[2] - 7.0) < TEST_PRECISION &&
            fabs(B.m[3] - 2.0) < TEST_PRECISION &&
            fabs(B.m[6] - 3.0) < TEST_PRECISION) ? 0 : 1;
}

static int test_rotation_matrices(void)
{
    /* Test 90-degree rotation around Z */
    apollo_mat3_t Rz = apollo_mat3_rot_z(M_PI_2);
    apollo_v3_t x = { .x = 1.0, .y = 0.0, .z = 0.0 };
    apollo_v3_t y = apollo_mat3_mul_v3(&Rz, &x);

    printf("rot_z(90deg): (1,0,0) -> (%.6f, %.6f, %.6f)\n", y.x, y.y, y.z);

    /* (1,0,0) rotated 90deg around Z should become (0,1,0) */
    return (fabs(y.x - 0.0) < TEST_PRECISION &&
            fabs(y.y - 1.0) < TEST_PRECISION &&
            fabs(y.z - 0.0) < TEST_PRECISION) ? 0 : 1;
}

int main(void)
{
    int failures = 0;

    printf("=== Apollo Vector/Matrix Library Tests ===\n\n");

    printf("-- Vector operations --\n");
    failures += test_v3_add();
    failures += test_v3_sub();
    failures += test_v3_scale();
    failures += test_v3_dot();
    failures += test_v3_cross();
    failures += test_v3_mag();
    failures += test_v3_unit();

    printf("\n-- Matrix operations --\n");
    failures += test_mat3_mul_v3();
    failures += test_mat3_mul();
    failures += test_mat3_transpose();
    failures += test_rotation_matrices();

    printf("\n=== Summary: %d failures ===\n", failures);

    return failures > 0 ? 1 : 0;
}

#endif /* APOLLO_VECMATH_TEST */
