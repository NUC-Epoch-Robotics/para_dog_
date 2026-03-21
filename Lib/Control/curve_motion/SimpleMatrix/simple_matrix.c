#include "simple_matrix.h"
#include <string.h>
#ifdef SIMPLE_MATRIX_USE_MALLOC
#include <stdlib.h>
#endif

/* Quick sanity check for shape and buffer presence. */
static bool matrix_is_valid(const matrix_t *m)
{
    return m != NULL && m->data != NULL && m->rows > 0 && m->cols > 0;
}

/* Wrap an external buffer into a matrix view (no allocation). */
matrix_status_t matrix_wrap(matrix_t *m, size_t rows, size_t cols, float *buffer)
{
    if (m == NULL || buffer == NULL)
    {
        return MATRIX_ERR_NULL;
    }
    if (rows == 0 || cols == 0)
    {
        return MATRIX_ERR_DIM;
    }

    m->rows = rows;
    m->cols = cols;
    m->data = buffer;
    m->owns_data = false;
    return MATRIX_OK;
}

#ifdef SIMPLE_MATRIX_USE_MALLOC
/* Allocate storage for a matrix; caller frees with matrix_free. */
matrix_status_t matrix_create(matrix_t *m, size_t rows, size_t cols)
{
    if (m == NULL)
    {
        return MATRIX_ERR_NULL;
    }
    if (rows == 0 || cols == 0)
    {
        return MATRIX_ERR_DIM;
    }

    size_t count = rows * cols;
    float *buf = (float *)malloc(count * sizeof(float));
    if (buf == NULL)
    {
        return MATRIX_ERR_NO_MEM;
    }

    m->rows = rows;
    m->cols = cols;
    m->data = buf;
    m->owns_data = true;
    return MATRIX_OK;
}

/* Free owned storage (no-op if wrapper or NULL). */
void matrix_free(matrix_t *m)
{
    if (m == NULL)
    {
        return;
    }
    if (m->owns_data && m->data != NULL)
    {
        free(m->data);
    }
    m->rows = 0;
    m->cols = 0;
    m->data = NULL;
    m->owns_data = false;
}
#endif

/* Set every element to value. */
matrix_status_t matrix_fill(matrix_t *m, float value)
{
    if (!matrix_is_valid(m))
    {
        return MATRIX_ERR_NULL;
    }
    size_t count = m->rows * m->cols;
    for (size_t i = 0; i < count; ++i)
    {
        m->data[i] = value;
    }
    return MATRIX_OK;
}

/* Make an identity matrix; requires square. */
matrix_status_t matrix_identity(matrix_t *m)
{
    if (!matrix_is_valid(m))
    {
        return MATRIX_ERR_NULL;
    }
    if (m->rows != m->cols)
    {
        return MATRIX_ERR_DIM;
    }

    matrix_fill(m, 0.0f);
    for (size_t i = 0; i < m->rows; ++i)
    {
        m->data[i * m->cols + i] = 1.0f;
    }
    return MATRIX_OK;
}

/* Copy src into dst when shapes match. */
matrix_status_t matrix_copy(const matrix_t *src, matrix_t *dst)
{
    if (!matrix_is_valid(src) || !matrix_is_valid(dst))
    {
        return MATRIX_ERR_NULL;
    }
    if (!matrix_same_shape(src, dst))
    {
        return MATRIX_ERR_DIM;
    }

    size_t count = src->rows * src->cols;
    memcpy(dst->data, src->data, count * sizeof(float));
    return MATRIX_OK;
}

/* Shapes equal helper (rows and cols). */
bool matrix_same_shape(const matrix_t *a, const matrix_t *b)
{
    if (a == NULL || b == NULL)
    {
        return false;
    }
    return a->rows == b->rows && a->cols == b->cols;
}

/* Elementwise add: out = a + b. */
matrix_status_t matrix_add(const matrix_t *a, const matrix_t *b, matrix_t *out)
{
    if (!matrix_is_valid(a) || !matrix_is_valid(b) || !matrix_is_valid(out))
    {
        return MATRIX_ERR_NULL;
    }
    if (!matrix_same_shape(a, b) || !matrix_same_shape(a, out))
    {
        return MATRIX_ERR_DIM;
    }

    size_t count = a->rows * a->cols;
    for (size_t i = 0; i < count; ++i)
    {
        out->data[i] = a->data[i] + b->data[i];
    }
    return MATRIX_OK;
}

/* Elementwise subtract: out = a - b. */
matrix_status_t matrix_sub(const matrix_t *a, const matrix_t *b, matrix_t *out)
{
    if (!matrix_is_valid(a) || !matrix_is_valid(b) || !matrix_is_valid(out))
    {
        return MATRIX_ERR_NULL;
    }
    if (!matrix_same_shape(a, b) || !matrix_same_shape(a, out))
    {
        return MATRIX_ERR_DIM;
    }

    size_t count = a->rows * a->cols;
    for (size_t i = 0; i < count; ++i)
    {
        out->data[i] = a->data[i] - b->data[i];
    }
    return MATRIX_OK;
}

/* Scale: out = k * a. */
matrix_status_t matrix_scalar(const matrix_t *a, float k, matrix_t *out)
{
    if (!matrix_is_valid(a) || !matrix_is_valid(out))
    {
        return MATRIX_ERR_NULL;
    }
    if (!matrix_same_shape(a, out))
    {
        return MATRIX_ERR_DIM;
    }

    size_t count = a->rows * a->cols;
    for (size_t i = 0; i < count; ++i)
    {
        out->data[i] = a->data[i] * k;
    }
    return MATRIX_OK;
}

/* Matrix multiply: out = a * b. */
matrix_status_t matrix_mul(const matrix_t *a, const matrix_t *b, matrix_t *out)
{
    if (!matrix_is_valid(a) || !matrix_is_valid(b) || !matrix_is_valid(out))
    {
        return MATRIX_ERR_NULL;
    }
    if (a->cols != b->rows)
    {
        return MATRIX_ERR_DIM;
    }
    if (out->rows != a->rows || out->cols != b->cols)
    {
        return MATRIX_ERR_DIM;
    }

    for (size_t r = 0; r < a->rows; ++r)
    {
        for (size_t c = 0; c < b->cols; ++c)
        {
            float sum = 0.0f;
            const size_t a_row_offset = r * a->cols;
            for (size_t k = 0; k < a->cols; ++k)
            {
                sum += a->data[a_row_offset + k] * b->data[k * b->cols + c];
            }
            out->data[r * out->cols + c] = sum;
        }
    }
    return MATRIX_OK;
}

/* Transpose: out = a^T. */
matrix_status_t matrix_transpose(const matrix_t *a, matrix_t *out)
{
    if (!matrix_is_valid(a) || !matrix_is_valid(out))
    {
        return MATRIX_ERR_NULL;
    }
    if (out->rows != a->cols || out->cols != a->rows)
    {
        return MATRIX_ERR_DIM;
    }

    for (size_t r = 0; r < a->rows; ++r)
    {
        for (size_t c = 0; c < a->cols; ++c)
        {
            out->data[c * out->cols + r] = a->data[r * a->cols + c];
        }
    }
    return MATRIX_OK;
}
