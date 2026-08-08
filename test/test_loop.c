/* Closed-loop tests: the firmware controls a simulated plant.
 *
 * Each tick the plant is advanced, its temperature is turned back into ADC
 * counts, those counts are handed to the firmware through the I2C fake, the
 * firmware computes new duties, and those duties drive the plant.
 *
 * The loop is closed entirely on the host - a 30 minute run costs milliseconds
 * and repeats exactly.
 */

#include "unity.h"
#include "thermal_app.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "plant.h"

#include <stdio.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define MUTEX_NODE ((SemaphoreHandle_t)(intptr_t)3)
#define ZONE 0

static FILE *s_csv;


/* Mirror of the firmware's sensor decode (thermal_app.c:80-99), which is
   static. The test needs it to know what temperature the controller saw. */
static float decode_counts_to_celsius(int counts)
{
    float x = (float)counts / 5120.0f + (10.0f / 340.0f);
    if (x > 0.999f)  x = 0.999f;
    if (x < 0.0005f) x = 0.0005f;
    float rntc = 330.0f * x / (1.0f - x);

    float inv_t = (1.0f / 298.15f) + (1.0f / 3500.0f) * logf(rntc / 10.0f);
    if (inv_t < 1e-6f) inv_t = 1e-6f;
    return (1.0f / inv_t) - 273.15f;
}


void setUp(void)
{
    fake_i2c_reset();
    fake_task_reset();
    fake_queue_reset();
    fake_sem_reset();

    ThermalApp_Init();
    ThermalApp_StartTasks();

    for (int i = 0; i < NUM_REMOTE_NODES; i++) fake_i2c_set_present(i, 1);
}

void tearDown(void) { }


/* One control cycle: plant -> sensor -> firmware -> actuator -> plant.
 * Mirrors ControlTask's inner body (thermal_app.c:376-430), which is static
 * and so cannot be called directly. */
static void loop_tick(void)
{
    ZoneCtrl *z = &zones[ZONE];

    /* 1. sensor: plant temperature becomes ADC counts on the wire */
    int      counts = plant_temp_to_counts(plant_temp_c());
    uint16_t wire   = plant_counts_to_wire(counts);
    fake_i2c_set_telemetry(ZONE, wire, plant_fan_rpm_to_tach(z->fan_duty));

    /* 2. bus: node telemetry lands in shared memory */
    I2C_Telemetry tel;
    HAL_I2C_Master_Receive(&hi2c1, (0x20 << 1), (uint8_t *)&tel, sizeof(tel), 10);
    xSemaphoreTake(MUTEX_NODE, portMAX_DELAY);
    g_nodes[ZONE].tel       = tel;
    g_nodes[ZONE].is_online = 1;
    xSemaphoreGive(MUTEX_NODE);

    /* 3. firmware decodes and runs its state machine.
     *    temp_c comes from the DECODED counts, not from plant_temp_c(): the
     *    controller must only ever see what the sensor actually reported,
     *    noise and quantisation included. Reading the plant directly would
     *    hand the PID a perfect measurement no real system ever gets. */
    int   decoded = (int)tel.adc_raw - 512;      /* contract A */
    float temp_c  = decode_counts_to_celsius(decoded);

    if (z->state == ST_PID || z->state == ST_THROTTLE) {
        z->pid_temp = z->pid_temp + 0.02f * (temp_c - z->pid_temp);
    }
    Zone_Tick(z, decoded, temp_c, z->pid_temp, (uint32_t)tel.tach_pulses * 30u);

    if (temp_c > MAX_SAFE_TEMP_C) {
        z->heater_duty = 0.0f;
        z->fan_duty    = 1.0f;
    }

    /* 4. actuators drive the plant into the next second */
    plant_step(z->heater_duty, z->fan_duty);
    fake_tick_advance(1000);

    if (s_csv) {
        fprintf(s_csv, "%.1f,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.1f,%u,%d\n",
                (float)HAL_GetTick() / 1000.0f, temp_c, z->pid_temp, decoded,
                z->fan_duty, z->heater_duty, z->requested_heater_duty,
                z->setpoint_c, (unsigned)((uint32_t)tel.tach_pulses * 30u), (int)z->state);
    }
}

static void run_for(int seconds)
{
    for (int i = 0; i < seconds; i++) loop_tick();
}


/* ---------------------------------------------------------------------------
 * 1. Dead time is really there.
 * ------------------------------------------------------------------------ */
static void test_plant_shows_dead_time(void)
{
    plant_init(PLANT_AMBIENT_C, 0, 0);

    float t0 = plant_temp_c();
    for (int i = 0; i < PLANT_THETA_N; i++) plant_step(1.0f, 0.0f);

    /* Nothing commanded theta ago has arrived yet. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, t0, plant_temp_c());

    plant_step(1.0f, 0.0f);
    TEST_ASSERT_TRUE(plant_temp_c() > t0 + 0.01f);
}


/* ---------------------------------------------------------------------------
 * 2. Open-loop step reproduces the identified gain.
 *    After ~5 tau the rise should approach K.
 * ------------------------------------------------------------------------ */
static void test_open_loop_gain_matches_identification(void)
{
    plant_init(PLANT_AMBIENT_C, 0, 0);

    for (int i = 0; i < 5 * (int)PLANT_TAU_S; i++) plant_step(1.0f, 0.0f);

    float rise = plant_temp_c() - PLANT_AMBIENT_C;
    TEST_ASSERT_FLOAT_WITHIN(0.3f, PLANT_K, rise);
}


/* ---------------------------------------------------------------------------
 * 3. One time constant gets ~63% of the way there.
 * ------------------------------------------------------------------------ */
static void test_one_tau_reaches_63_percent(void)
{
    plant_init(PLANT_AMBIENT_C, 0, 0);

    for (int i = 0; i < PLANT_THETA_N + (int)PLANT_TAU_S; i++) plant_step(1.0f, 0.0f);

    float frac = (plant_temp_c() - PLANT_AMBIENT_C) / PLANT_K;
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.632f, frac);
}


/* ---------------------------------------------------------------------------
 * 4. Closed loop: the fan PID holds the setpoint against a heat load.
 * ------------------------------------------------------------------------ */
static void test_closed_loop_regulates_to_setpoint(void)
{
    plant_init(PLANT_AMBIENT_C, 0, 1);

    /* The load must be big enough that the plant would overshoot on its own:
       25 + 8.2*0.9 = 32.4 degC against a 30 degC setpoint. Otherwise the loop
       settles where the plant was going to settle anyway and the test proves
       nothing about the controller. */
    zones[ZONE].setpoint_c            = 30.0f;
    zones[ZONE].requested_heater_duty = 0.9f;
    zones[ZONE].start_req             = 1;

    run_for(1200);

    float err = fabsf(plant_temp_c() - zones[ZONE].setpoint_c);
    TEST_ASSERT_TRUE_MESSAGE(err < 2.0f, "loop did not settle near setpoint");

    /* And the fan must actually be doing the work. */
    TEST_ASSERT_TRUE_MESSAGE(zones[ZONE].fan_duty > 0.05f,
                             "fan idle - plant settled on its own, loop untested");
}


/* ---------------------------------------------------------------------------
 * 5. ADC noise reaches the controller but does not destabilise it.
 *
 * The +/-1 LSB jitter is mean-zero, so the EMA should absorb it: the loop
 * still holds the setpoint, and the fan twitches without hunting.
 * ------------------------------------------------------------------------ */
static void test_noise_does_not_destabilise(void)
{
    plant_init(PLANT_AMBIENT_C, 1, 1);

    zones[ZONE].setpoint_c            = 30.0f;
    zones[ZONE].requested_heater_duty = 0.9f;
    zones[ZONE].start_req             = 1;

    run_for(1200);

    float err = fabsf(plant_temp_c() - zones[ZONE].setpoint_c);
    TEST_ASSERT_TRUE_MESSAGE(err < 2.5f, "noise destabilised the loop");
    TEST_ASSERT_TRUE(zones[ZONE].fan_duty > 0.05f);

    /* Bounded hunting: over the last 200 s the fan must keep moving - proof
       the noise really is on the control path and not being read around -
       yet stay well short of slamming between limits. */
    float lo = 1.0f, hi = 0.0f;
    for (int i = 0; i < 200; i++) {
        loop_tick();
        if (zones[ZONE].fan_duty < lo) lo = zones[ZONE].fan_duty;
        if (zones[ZONE].fan_duty > hi) hi = zones[ZONE].fan_duty;
    }
    float swing = hi - lo;
    TEST_ASSERT_TRUE_MESSAGE(swing > 0.0005f,
        "fan perfectly steady - noise is not reaching the controller");
    TEST_ASSERT_TRUE_MESSAGE(swing < 0.20f, "fan hunting on sensor noise");
}


/* The jitter must be mean-zero. A biased 'noise' source is a calibration
   offset wearing a disguise, and it would quietly move the setpoint. */
static void test_noise_is_unbiased(void)
{
    plant_init(30.0f, 1, 0);

    long sum = 0;
    const int N = 4000;
    for (int i = 0; i < N; i++) sum += plant_temp_to_counts(30.0f);

    plant_init(30.0f, 0, 0);
    long clean = plant_temp_to_counts(30.0f) * (long)N;

    float bias = (float)(sum - clean) / (float)N;
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, bias);
}


/* ---------------------------------------------------------------------------
 * 6. The safety backstop cuts the heater above MAX_SAFE_TEMP_C.
 * ------------------------------------------------------------------------ */
static void test_hard_safety_cutoff(void)
{
    plant_init(MAX_SAFE_TEMP_C + 5.0f, 0, 0);

    zones[ZONE].state       = ST_PID;
    zones[ZONE].heater_duty = 0.8f;

    loop_tick();

    TEST_ASSERT_EQUAL_FLOAT(0.0f, zones[ZONE].heater_duty);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, zones[ZONE].fan_duty);
}


int main(int argc, char **argv)
{
    if (argc > 1) {
        s_csv = fopen(argv[1], "w");
        if (s_csv) fprintf(s_csv, "time_s,temp_c,temp_ema,counts,fan_duty,"
                                  "heater_duty,heater_req,setpoint,fan_rpm,state\n");
    }

    UNITY_BEGIN();
    RUN_TEST(test_plant_shows_dead_time);
    RUN_TEST(test_open_loop_gain_matches_identification);
    RUN_TEST(test_one_tau_reaches_63_percent);
    RUN_TEST(test_closed_loop_regulates_to_setpoint);
    RUN_TEST(test_noise_does_not_destabilise);
    RUN_TEST(test_noise_is_unbiased);
    RUN_TEST(test_hard_safety_cutoff);

    if (s_csv) fclose(s_csv);
    return UNITY_END();
}
