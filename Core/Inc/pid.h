#ifndef PID_H
#define PID_H

/* --------------------------------------------------------------------------
 * PID Controller – parallel form with derivative on measurement,
 * trapezoidal integrator, filtered derivative, and conditional anti‑windup.
 *
 * This is the final, tuned implementation that exactly matches the Python
 * simulation we validated.  Copy this file and pid.c into your STM32CubeIDE
 * project (Inc and Src folders) and add #include "pid.h" in thermal_app.h.
 * -------------------------------------------------------------------------- */
typedef struct {
    float Kp, Ki, Kd;       /* Controller gains                           */
    float Ts;               /* Sample period (seconds) – must match loop  */
    float tau_f;            /* Derivative low‑pass filter time constant   */
    float alpha;            /* Smoothing factor for derivative filter     */
    float e_prev;           /* Previous error (for trapezoidal integrator)*/
    float meas_prev;        /* Previous measurement (derivative on meas.) */
    float integral;         /* Integrator state                           */
    float deriv;            /* Filtered derivative state                  */
    float out_max;
    float out_min;

} PID_Handle;

/* --- Initialise the PID structure --------------------------------------- *
 * Kp, Ki, Kd  : final tuned gains
 * Ts          : loop sample time (seconds)
 * tau_f       : derivative filter time constant (typically 1/10 of Td)
 * init_meas   : initial measurement value (prevents derivative kick)
 * ------------------------------------------------------------------------ */
void PID_Init(PID_Handle *pid,
              float Kp, float Ki, float Kd,
              float Ts, float tau_f, float init_meas);

/* --- Evaluate one control step ------------------------------------------ *
 * setpoint    : desired value (e.g. 30.0 °C)
 * measured    : current process variable (temperature)
 * returns     : control output (0.0 .. 1.0 for PWM duty)
 * ------------------------------------------------------------------------ */
float PID_Update(PID_Handle *pid, float setpoint, float measured);

#endif
