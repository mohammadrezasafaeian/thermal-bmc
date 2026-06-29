#include "pid.h"

/* --------------------------------------------------------------------------
 * PID_Init – one‑time setup
 *
 * Stores the gains and initialises the internal states.  The most important
 * details are:
 *  1. meas_prev is set to the current temperature so the derivative term
 *     does not see a huge step from zero – this avoids a "derivative kick"
 *     on the very first call.
 *  2. The derivative filter coefficient alpha = Ts / (tau_f + Ts).
 *     With Ts=1 s and tau_f=10 s, alpha ≈ 0.0909.  This means the
 *     derivative filter acts like a gentle exponential moving average
 *     with a time constant of about 11 seconds.
 * -------------------------------------------------------------------------- */
void PID_Init(PID_Handle *pid,
              float Kp, float Ki, float Kd,
              float Ts, float tau_f, float init_meas)
{
    pid->Kp   = Kp;
    pid->Ki   = Ki;
    pid->Kd   = Kd;
    pid->Ts   = Ts;
    pid->tau_f = tau_f;
    pid->alpha = Ts / (tau_f + Ts);
    pid->e_prev     = 0.0f;
    pid->meas_prev  = init_meas;   /* start from true ambient to avoid kick */
    pid->integral   = 0.0f;
    pid->deriv      = 0.0f;
}
float out_min;   /* clamp low  (fan: 0.0)  */
float out_max;   /* clamp high (fan: 1.0)  */
/* --------------------------------------------------------------------------
 * PID_Update – call once per sample period
 *
 * Implements the discrete‑time parallel PID:
 *   u[n] = Kp * e[n]  +  I[n]  +  D[n]
 * where:
 *   I[n] = I[n-1] + (Ki*Ts/2)*(e[n] + e[n-1])   (trapezoidal integrator)
 *   D[n] = alpha * Kd * ( -(y[n]-y[n-1])/Ts ) + (1-alpha) * D[n-1]
 *        (filtered derivative on measurement, no derivative kick)
 *
 * The output is clamped to [0, 1] (PWM duty for our heater).
 *
 * Anti‑windup: if the output saturates and the error is still pushing it
 * further into saturation, the integrator increment that was just added is
 * undone.  This prevents the integrator from "winding up" and causing large
 * overshoots when the actuator eventually leaves saturation.
 * -------------------------------------------------------------------------- */
float PID_Update(PID_Handle *pid, float setpoint, float measured)
{
    float e = setpoint - measured;          /* STANDARD — never negated here  */

    float P = pid->Kp * e;                  /* sign of Kp sets plant direction */

    if (pid->Ki != 0.0f)
        pid->integral += 0.5f * pid->Ki * pid->Ts * (e + pid->e_prev);
    float I = pid->integral;

    float d_raw = -(measured - pid->meas_prev) / pid->Ts;
    pid->deriv = pid->alpha * pid->Kd * d_raw + (1.0f - pid->alpha) * pid->deriv;
    float D = pid->deriv;

    float u = P + I + D;

    /* per-handle output limits (fan: 0..1; future loops set their own) */
    float u_clamped = u;
    if (u > pid->out_max) u_clamped = pid->out_max;
    if (u < pid->out_min) u_clamped = pid->out_min;

    /* anti-windup: freeze when clamped AND error still pushes further out.
     * generic — works for either limit, either gain sign. */
    if (pid->Ki != 0.0f && u_clamped != u) {
        float push = pid->Ki * e;           /* direction integrator is moving  */
        if ((u > pid->out_max && push > 0.0f) ||
            (u < pid->out_min && push < 0.0f)) {
            pid->integral -= 0.5f * pid->Ki * pid->Ts * (e + pid->e_prev);
        }
    }

    pid->e_prev    = e;
    pid->meas_prev = measured;
    return u_clamped;
}
