/* First-order-plus-dead-time thermal plant, identified on the real hardware.
 *
 *   K     = 8.2 degC per unit heater duty
 *   tau   = 111 s
 *   theta = 10 s
 *
 * Continuous form:      tau * dT/dt = -(T - T_amb) + K * u(t - theta)
 * Discretised at Ts:    T += (Ts/tau) * (K*u_delayed - (T - T_amb))
 *
 * The dead time is a shift register of duty commands: what the plant reacts
 * to now is what was commanded theta seconds ago. That delay is the whole
 * reason the loop can oscillate, so it has to be modelled, not averaged away.
 *
 * The fan is a second input. It removes heat in proportion to its duty, which
 * is what gives the fan PID something to push against.
 */

#include "plant.h"
#include <string.h>
#include <math.h>

/* theta steps of delay: the ring is read before it is written, so N slots
   hold the command back by exactly N steps. */
#define DELAY_SLOTS (PLANT_THETA_N)

static struct {
    float temp_c;
    float heater_delay[DELAY_SLOTS];
    int   delay_head;
    int   noise_on;
    int   fan_on;
    uint32_t rng;
} s_p;

/* Small deterministic PRNG. Deterministic matters: a test that fails only on
   some runs is worse than no test. Same seed, same sequence, every time. */
static float noise_lsb(void)
{
    s_p.rng = s_p.rng * 1664525u + 1013904223u;
    return (float)((int)((s_p.rng >> 16) % 3u) - 1);     /* -1, 0, +1, mean 0 */
}

void plant_init(float ambient_c, int with_noise, int with_fan)
{
    memset(&s_p, 0, sizeof(s_p));
    s_p.temp_c   = ambient_c;
    s_p.noise_on = with_noise;
    s_p.fan_on   = with_fan;
    s_p.rng      = 12345u;
}

void plant_step(float heater_duty, float fan_duty)
{
    /* What the plant feels now was commanded theta seconds ago. */
    float u = s_p.heater_delay[s_p.delay_head];
    s_p.heater_delay[s_p.delay_head] = heater_duty;
    s_p.delay_head = (s_p.delay_head + 1) % DELAY_SLOTS;

    float rise = PLANT_K * u;

    /* Fan cooling: extra heat loss on top of the natural one. */
    float cooling = s_p.fan_on ? (PLANT_FAN_K * fan_duty) : 0.0f;

    float dT = (PLANT_TS / PLANT_TAU_S)
             * (rise - (s_p.temp_c - PLANT_AMBIENT_C) * (1.0f + cooling));

    s_p.temp_c += dT;
}

float plant_temp_c(void)
{
    return s_p.temp_c;
}

/* ---- inverse sensor chain: degC -> ADC counts ---------------------------
 * The firmware decodes counts -> R -> degC. To hand it a temperature we have
 * to run that backwards, so the decode itself stays under test.
 *
 *   R    = R0 * exp(BETA * (1/T - 1/T0))
 *   x    = R / (330 + R)
 *   d    = x - 10/340
 *   cnt  = d * 5120
 */
int plant_temp_to_counts(float temp_c)
{
    float t_k = temp_c + 273.15f;
    float r   = PLANT_NTC_R0 * expf(PLANT_NTC_BETA * (1.0f / t_k - 1.0f / PLANT_NTC_T0_K));
    float x   = r / (330.0f + r);
    float d   = x - (10.0f / 340.0f);
    float cnt = d * 5120.0f;

    if (s_p.noise_on) cnt += noise_lsb();

    return (int)(cnt + (cnt >= 0.0f ? 0.5f : -0.5f));
}

uint16_t plant_counts_to_wire(int counts)
{
    /* Contract A: the ATmega sends counts + 512 so the value stays unsigned. */
    int wire = counts + 512;
    if (wire < 0)     wire = 0;
    if (wire > 65535) wire = 65535;
    return (uint16_t)wire;
}

uint16_t plant_fan_rpm_to_tach(float fan_duty)
{
    /* The fan free-runs off its own rail, so the tach never reads zero even at
       zero duty - which is what keeps detect_fault() from calling FR_FAN_OPEN
       the moment the board powers up. Firmware does rpm = pulses * 30. */
    float rpm = PLANT_FAN_IDLE_RPM
              + (PLANT_FAN_MAX_RPM - PLANT_FAN_IDLE_RPM) * fan_duty;
    return (uint16_t)(rpm / 30.0f);
}
