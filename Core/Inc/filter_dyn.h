/*
 * filter_dyn.h
 *
 *  Created on: Feb 23, 2026
 *      Author: Tick
 */

#ifndef INC_FILTER_DYN_H_
#define INC_FILTER_DYN_H_
#ifndef FILTER_DYN_H
#define FILTER_DYN_H

/**
 * filter_dyn.h  -  Real-time biquad coefficient computation for DSP Lab
 *
 * Replaces the static iir_lpf.h coefficients with on-the-fly calculation
 * driven by UART commands (FTYPE / FCUT / FQ / FBYPASS) from the ESP32.
 *
 * All filters are single biquad (2nd order).  Add more stages here if needed.
 *
 * Coefficient layout for CMSIS arm_biquad_cascade_df1:
 *   {b0, b1, b2, -a1_norm, -a2_norm}   (negated, normalised by a0)
 */

#include "arm_math.h"
#include <math.h>

/* ── filter types (must match browser FilterType enum) ── */
#define FTYPE_BYPASS   0
#define FTYPE_LOWPASS  1
#define FTYPE_HIGHPASS 2
#define FTYPE_BANDPASS 3
#define FTYPE_NOTCH    4

#define DYN_IIR_STAGES 1
#define DYN_COEFF_LEN  (DYN_IIR_STAGES * 5)
#define DYN_STATE_LEN  (DYN_IIR_STAGES * 4)

/* Mutable coefficient & state arrays used by the biquad instance */
extern float32_t dyn_coeffs[DYN_COEFF_LEN];
extern float32_t dyn_state [DYN_STATE_LEN];

/**
 * @brief  Compute biquad coefficients for the requested filter.
 * @param  type    FTYPE_* constant
 * @param  fc_hz   Cutoff/centre frequency in Hz
 * @param  q10     Q factor × 10  (e.g. 7 → Q = 0.707)
 * @param  fs_hz   Sample rate in Hz (must be the actual ADC rate)
 *
 * Call this whenever type, fc, Q or sample-rate changes.
 * Resets dyn_state to zero (clears filter memory).
 */
static inline void filter_dyn_compute(uint8_t type,
                                       uint32_t fc_hz,
                                       uint8_t  q10,
                                       uint32_t fs_hz)
{
    /* --- Clamp inputs -------------------------------------------------- */
    if (fc_hz < 10)        fc_hz = 10;
    if (fc_hz > fs_hz / 2) fc_hz = fs_hz / 2 - 1;
    if (q10 < 1)   q10 = 1;
    if (q10 > 100) q10 = 100;

    float Q   = q10 / 10.0f;
    float w0  = 2.0f * (float)M_PI * (float)fc_hz / (float)fs_hz;
    float cw  = cosf(w0);
    float sw  = sinf(w0);
    float alp = sw / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;

    switch (type) {
        default:
        case FTYPE_BYPASS:
            /* Identity: H(z) = 1 */
            dyn_coeffs[0] = 1.0f; dyn_coeffs[1] = 0.0f; dyn_coeffs[2] = 0.0f;
            dyn_coeffs[3] = 0.0f; dyn_coeffs[4] = 0.0f;
            memset(dyn_state, 0, sizeof(float32_t) * DYN_STATE_LEN);
            return;

        case FTYPE_LOWPASS:
            b0 = (1.0f - cw) / 2.0f;
            b1 =  1.0f - cw;
            b2 = (1.0f - cw) / 2.0f;
            a0 =  1.0f + alp;
            a1 = -2.0f * cw;
            a2 =  1.0f - alp;
            break;

        case FTYPE_HIGHPASS:
            b0 =  (1.0f + cw) / 2.0f;
            b1 = -(1.0f + cw);
            b2 =  (1.0f + cw) / 2.0f;
            a0 =   1.0f + alp;
            a1 =  -2.0f * cw;
            a2 =   1.0f - alp;
            break;

        case FTYPE_BANDPASS:
            /* Constant 0 dB at peak */
            b0 =  alp;
            b1 =  0.0f;
            b2 = -alp;
            a0 =  1.0f + alp;
            a1 = -2.0f * cw;
            a2 =  1.0f - alp;
            break;

        case FTYPE_NOTCH:
            b0 =  1.0f;
            b1 = -2.0f * cw;
            b2 =  1.0f;
            a0 =  1.0f + alp;
            a1 = -2.0f * cw;
            a2 =  1.0f - alp;
            break;
    }

    /* Normalise and convert to CMSIS sign convention {b0,b1,b2,-a1,-a2} */
    dyn_coeffs[0] =  b0 / a0;
    dyn_coeffs[1] =  b1 / a0;
    dyn_coeffs[2] =  b2 / a0;
    dyn_coeffs[3] = -a1 / a0;   /* CMSIS stores -a1 */
    dyn_coeffs[4] = -a2 / a0;   /* CMSIS stores -a2 */

    /* Clear filter memory after coefficient change */
    memset(dyn_state, 0, sizeof(float32_t) * DYN_STATE_LEN);
}

#endif /* FILTER_DYN_H */


#endif /* INC_FILTER_DYN_H_ */
