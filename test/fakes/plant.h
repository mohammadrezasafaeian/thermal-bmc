/* FOPDT thermal plant for closed-loop host tests.
 * Parameters identified from a step test on the real hardware. */
#ifndef FAKE_PLANT_H
#define FAKE_PLANT_H

#include <stdint.h>

#define PLANT_K            8.2f      /* degC per unit heater duty */
#define PLANT_TAU_S      111.0f      /* time constant, s          */
#define PLANT_THETA_S     10.0f      /* dead time, s              */
#define PLANT_THETA_N     10        /* dead time in whole steps  */
#define PLANT_TS           1.0f      /* step, matches PID_TS      */
#define PLANT_AMBIENT_C   25.0f
#define PLANT_FAN_K        1.5f      /* extra loss at full fan    */
#define PLANT_FAN_IDLE_RPM 1200.0f   /* fan free-runs; tach never reads 0 */
#define PLANT_FAN_MAX_RPM 13000.0f

#define PLANT_NTC_BETA  3500.0f
#define PLANT_NTC_R0      10.0f
#define PLANT_NTC_T0_K   298.15f

void  plant_init(float ambient_c, int with_noise, int with_fan);
void  plant_step(float heater_duty, float fan_duty);
float plant_temp_c(void);

/* Inverse sensor chain, so the firmware's own decode stays under test. */
int      plant_temp_to_counts(float temp_c);
uint16_t plant_counts_to_wire(int counts);
uint16_t plant_fan_rpm_to_tach(float fan_duty);

#endif
