#include "Ztransform_fx.h"

#include <stdlib.h>
#include <string.h>
#include <limits.h>

struct ZFilter_fx {
    zfix_t *b;      // numerator coefficients
    zfix_t *a;      // denominator coefficients

    uint32_t nb;    // number of b coeffs
    uint32_t na;    // number of a coeffs
    uint32_t ny;

    zfix_t *x_hist; // input history
    zfix_t *y_hist; // output history
};

//---------------------------------------------------------------------------------------------------------------------
static zfix_t zfix_saturate_i64(int64_t value)
{
    if (value > INT64_MAX) {
        return INT64_MAX;
    }

    if (value < INT64_MIN) {
        return INT64_MIN;
    }

    return (zfix_t)value;
}

//---------------------------------------------------------------------------------------------------------------------
static zfix_t zfix_mul(zfix_t lhs, zfix_t rhs)
{
    // Shift right by 16 to return to Q16.16.
    int64_t product = lhs * rhs;

    product = product >> ZFIX_FRAC_BITS;

    return zfix_saturate_i64(product);
}

//---------------------------------------------------------------------------------------------------------------------
static zfix_t zfix_div(zfix_t numerator, zfix_t denominator)
{

    // Shift numerator left by 16 before dividing to preserve scale.
    if (denominator == 0) {
        if (numerator >= 0) {
            return INT64_MAX;
        } else {
            return INT64_MIN;
        }
    }

    int64_t scaled = numerator << ZFIX_FRAC_BITS;

    return scaled / denominator;
}

//---------------------------------------------------------------------------------------------------------------------
ZFilter_fx* ZFilter_fx_ctor(const zfix_t *b_in, uint32_t nb,
                            const zfix_t *a_in, uint32_t na)
{
    // basic validation of inputs
    if (!b_in || !a_in) {
        return NULL;
    }

    if (nb == 0 || na == 0) {
        return NULL;
    }

    if (a_in[0] == 0) {
        return NULL;
    }

    // allocate memory for filter struct
    ZFilter_fx *filter = calloc(1, sizeof(ZFilter_fx));
    if (!filter) {
        return NULL;
    }

    // set filter parameters
    filter->nb = nb;
    filter->na = na;

    // calculate number of output history samples needed (na - 1)
    filter->ny = (na > 1) ? (na - 1) : 0;

    filter->b = calloc(nb, sizeof(zfix_t));
    filter->a = calloc(na, sizeof(zfix_t));
    filter->x_hist = calloc(nb, sizeof(zfix_t));

    // only allocate y_hist if needed (if na > 1)
    if (filter->ny > 0) {
        filter->y_hist = calloc(filter->ny, sizeof(zfix_t));
    }


    // check allocations
    if (!filter->b || !filter->a || !filter->x_hist) {
        ZFilter_fx_dtor(filter);
        return NULL;
    }

    // y_hist is optional based on na, so only check if allocation was attempted
    if (filter->ny > 0 && !filter->y_hist) {
        ZFilter_fx_dtor(filter);
        return NULL;
    }

    // copy coefficients
    memcpy(filter->b, b_in, nb * sizeof(zfix_t));
    memcpy(filter->a, a_in, na * sizeof(zfix_t));

    return filter;
}

//---------------------------------------------------------------------------------------------------------------------
zfix_t ZFilter_fx_update(ZFilter_fx *f, zfix_t x)
{
    if (!f) {
        return 0;
    }


    // shift the history of the array by 1
    if (f->nb > 1) {
        memmove(&f->x_hist[1],
                &f->x_hist[0],
                (f->nb - 1) * sizeof(zfix_t));
    }

    f->x_hist[0] = x;

    int64_t accumulator = 0;

    
    // calculate the Numerator part of the diff equ
    for (uint32_t i = 0; i < f->nb; i++) {
        accumulator += zfix_mul(f->b[i], f->x_hist[i]);
    }

    // calculate the Denominator feedback part of the diff equ
    for (uint32_t i = 1; i < f->na; i++) {
        accumulator -= zfix_mul(f->a[i], f->y_hist[i - 1]);
    }

    // calculate the final output by dividing by a[0]
    zfix_t y = zfix_div(accumulator, f->a[0]);

    //Shift output history. y_hist[i] = y[n - i]
    if (f->ny > 0) {
        if (f->ny > 1) {
            memmove(&f->y_hist[1],
                    &f->y_hist[0],
                    (f->ny - 1) * sizeof(zfix_t));
        }

        f->y_hist[0] = y;
    }

    return y;
}

//---------------------------------------------------------------------------------------------------------------------

void ZFilter_fx_reset(ZFilter_fx *f)
{
    if (!f) {
        return;
    }
    // reset input history
    memset(f->x_hist, 0, f->nb * sizeof(zfix_t));

    // reset output history if it exists
    if (f->ny > 0) {
        memset(f->y_hist, 0, f->ny * sizeof(zfix_t));
    }
}

//---------------------------------------------------------------------------------------------------------------------

void ZFilter_fx_dtor(ZFilter_fx *f)
{
    if (!f) {
        return;
    }

    // free allocated memory for coefficients and history
    free(f->b);
    free(f->a);
    free(f->x_hist);
    free(f->y_hist);

    free(f);
}