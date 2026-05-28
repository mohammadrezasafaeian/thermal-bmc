#ifndef IIR_LPF_H
#define IIR_LPF_H

// Butterworth LPF: order 6, fc = 50000 Hz, fs = 500000 Hz
// Attenuation at 100 kHz: 41.9 dB

#define IIR_NUM_STAGES 3

static float iir_coeffs[5 * IIR_NUM_STAGES] = {
    // Section 0
    0.0003405377f, 0.0006810753f, 0.0003405377f, 1.0320694053f, -0.2757079425f,
    // Section 1
    1.0000000000f, 2.0000000000f, 1.0000000000f, 1.1429805025f, -0.4128015981f,
    // Section 2
    1.0000000000f, 2.0000000000f, 1.0000000000f, 1.4043848905f, 0.7359151912f,
};

static float iir_state[4 * IIR_NUM_STAGES] = {0};

#endif
