#include "Ztransform_fp.h"
#include <stdlib.h>
#include <string.h>


struct ZFilter_fp {
    float *b;     // numerator coefficients
    float *a;     // denominator coefficients

    uint32_t nb;  // number of b coeff
    uint32_t na;  // number of a coeff

    float *x_hist; // input history
    float *y_hist; // output history
};

//---------------------------------------------------------------------------------------------------------------------
ZFilter_fp* ZFilter_fp_ctor(const float *b_in, uint32_t nb,
                      const float *a_in, uint32_t na)
{
    // basic validation of inputs
    if (!b_in || !a_in) return NULL;
    if (nb == 0 || na == 0) return NULL;
    if (a_in[0] == 0.0f) return NULL;

    // allocate memory for filter struct
    ZFilter_fp *filter = calloc(1, sizeof(ZFilter_fp));
    if (!filter) return NULL;

    filter->nb = nb;
    filter->na = na;

    // allocate memory for coefficients and history
    filter->b = calloc(nb, sizeof(float));
    filter->a = calloc(na, sizeof(float));
    filter->x_hist = calloc(nb, sizeof(float));
    filter->y_hist = calloc(na, sizeof(float));

    // check allocations 
    if (!filter->b || !filter->a || !filter->x_hist || !filter->y_hist) {
        ZFilter_fp_dtor(filter);
        return NULL;
    }

    // copy coefficients
    memcpy(filter->b, b_in, nb * sizeof(float));
    memcpy(filter->a, a_in, na * sizeof(float));

    return filter;
}

//---------------------------------------------------------------------------------------------------------------------
float ZFilter_fp_update(ZFilter_fp *f, float x)
{
    if (!f) return 0.0f;

    // shift input history
    memmove(&f->x_hist[1], 
            &f->x_hist[0], 
            (f->nb - 1) * sizeof(float));
            
    f->x_hist[0] = x;

    float y = 0.0f;

    // numerator (input side)
    for (uint32_t i = 0; i < f->nb; i++) {
        y += f->b[i] * f->x_hist[i];
    }

    // denominator (feedback side) — skip a[0]
    for (uint32_t i = 1; i < f->na; i++) {
        y -= f->a[i] * f->y_hist[i - 1];
    }

    // shift output history
    memmove(&f->y_hist[1], &f->y_hist[0], (f->na - 1) * sizeof(float));
    f->y_hist[0] = y;

    return y;
}

//---------------------------------------------------------------------------------------------------------------------
void ZFilter_fp_reset(ZFilter_fp *f)
{
    if (!f) return;

    memset(f->x_hist, 0, f->nb * sizeof(float));
    memset(f->y_hist, 0, f->na * sizeof(float));
}

//---------------------------------------------------------------------------------------------------------------------
void ZFilter_fp_dtor(ZFilter_fp *f)
{
    if (!f) return;

    // free allocated memory for coefficients and history
    free(f->b);
    free(f->a);
    free(f->x_hist);
    free(f->y_hist);

    free(f);
}