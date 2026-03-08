#ifndef SIMPLE_MATRIX_H
#define SIMPLE_MATRIX_H

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MATRIX_OK = 0,       /* operation succeeded */
    MATRIX_ERR_DIM,      /* incompatible dimensions */
    MATRIX_ERR_NULL,     /* null pointer or uninitialized data */
    MATRIX_ERR_OOB,      /* singular/invalid numeric condition */
    MATRIX_ERR_NO_MEM    /* allocation failure */
} matrix_status_t;

typedef struct {
    size_t rows;
    size_t cols;
    float *data;
    bool owns_data;
} matrix_t;

/* Bind an existing float buffer to a matrix view (no allocation, no ownership). */
matrix_status_t matrix_wrap(matrix_t *m, size_t rows, size_t cols, float *buffer);
#ifdef SIMPLE_MATRIX_USE_MALLOC
/* Allocate a matrix and own its buffer (requires SIMPLE_MATRIX_USE_MALLOC). */
matrix_status_t matrix_create(matrix_t *m, size_t rows, size_t cols);
/* Free owned storage (safe to call on non-owned buffers). */
void matrix_free(matrix_t *m);
#endif

/* Fill all entries with a constant. */
matrix_status_t matrix_fill(matrix_t *m, float value);
/* Make an identity matrix (square only). */
matrix_status_t matrix_identity(matrix_t *m);
/* Copy contents between same-shaped matrices. */
matrix_status_t matrix_copy(const matrix_t *src, matrix_t *dst);
/* Shape equality helper. */
bool matrix_same_shape(const matrix_t *a, const matrix_t *b);

/* Elementwise arithmetic: out = a (+|-) b. */
matrix_status_t matrix_add(const matrix_t *a, const matrix_t *b, matrix_t *out);
matrix_status_t matrix_sub(const matrix_t *a, const matrix_t *b, matrix_t *out);
/* Scale: out = k * a. */
matrix_status_t matrix_scalar(const matrix_t *a, float k, matrix_t *out);
/* Multiply: out = a * b. */
matrix_status_t matrix_mul(const matrix_t *a, const matrix_t *b, matrix_t *out);
/* Transpose: out = a^T. */
matrix_status_t matrix_transpose(const matrix_t *a, matrix_t *out);

static inline float matrix_get(const matrix_t *m, size_t r, size_t c)
{
    return m->data[r * m->cols + c];
}

static inline void matrix_set(matrix_t *m, size_t r, size_t c, float v)
{
    m->data[r * m->cols + c] = v;
}

#ifdef __cplusplus
}
#endif

#endif /* SIMPLE_MATRIX_H */
