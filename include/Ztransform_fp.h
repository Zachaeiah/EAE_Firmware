#ifndef CONTROL_ZTRANSFORM_H_
#define CONTROL_ZTRANSFORM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of filter struct
typedef struct ZFilter_fp ZFilter_fp;

/**
 * @brief Create a Z-domain filter.
 *
 * @param b_in Numerator coefficients
 * @param nb Number of numerator coefficients
 * @param a_in Denominator coefficients
 * @param na Number of denominator coefficients
 * @return Pointer to filter, or NULL on failure
 */
ZFilter_fp* ZFilter_fp_ctor(const float *b_in, uint32_t nb,
                         const float *a_in, uint32_t na);

/**
 * @brief Update filter with one new input sample.
 *
 * @param f Filter pointer
 * @param x Input value
 * @return Filter output
 */
float ZFilter_fp_update(ZFilter_fp *f, float x);

/**
 * @brief Reset internal history.
 *
 * @param f Filter pointer
 */
void ZFilter_fp_reset(ZFilter_fp *f);

/**
 * @brief Destroy filter.
 *
 * @param f Filter pointer
 */
void ZFilter_fp_dtor(ZFilter_fp *f);

#ifdef __cplusplus
}
#endif

#endif