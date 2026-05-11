#ifndef CONTROL_ZTRANSFORM_FX_H_
#define CONTROL_ZTRANSFORM_FX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Real value = raw / 2^16

typedef int64_t zfix_t;

// Q16.16 fixed point format
#define ZFIX_FRAC_BITS   16
#define ZFIX_ONE         ((zfix_t)1LL << ZFIX_FRAC_BITS)

// Helper macros for converting between integers/doubles and fixed-point
#define ZFIX_FROM_INT(x)     ((zfix_t)(x) << ZFIX_FRAC_BITS)
#define ZFIX_TO_INT(x)       ((int32_t)((x) >> ZFIX_FRAC_BITS))

// For conversion from double, we add 0.5 before truncation to achieve rounding instead of flooring.
#define ZFIX_FROM_DOUBLE(x)  ((zfix_t)((x) * (double)ZFIX_ONE + ((x) >= 0.0 ? 0.5 : -0.5)))
#define ZFIX_TO_DOUBLE(x)    ((double)(x) / (double)ZFIX_ONE)

// Forward declaration of filter struct
typedef struct ZFilter_fx ZFilter_fx;

/**
 * @brief Create a fixed-point Z-domain filter.
 *
 * @param b_in Numerator coefficients in Q16.16
 * @param nb Number of numerator coefficients
 * @param a_in Denominator coefficients in Q16.16
 * @param na Number of denominator coefficients
 * @return Pointer to filter, or NULL on failure
 */
ZFilter_fx* ZFilter_fx_ctor(const zfix_t *b_in, uint32_t nb,
                            const zfix_t *a_in, uint32_t na);

/**
 * @brief Update filter with one new fixed-point input sample.
 *
 * @param f Filter pointer
 * @param x Input value in Q16.16
 * @return Output value in Q16.16
 */
zfix_t ZFilter_fx_update(ZFilter_fx *f, zfix_t x);

/**
 * @brief Reset internal history.
 *
 * @param f Filter pointer
 */
void ZFilter_fx_reset(ZFilter_fx *f);

/**
 * @brief Destroy filter.
 *
 * @param f Filter pointer
 */
void ZFilter_fx_dtor(ZFilter_fx *f);

#ifdef __cplusplus
}
#endif

#endif