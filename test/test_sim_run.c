/* Reproduce the recorded hardware run in simulation, and dump it in the same
 * binary format the firmware's black box produces.
 *
 * The point is that the SAME parser renders both: if the simulated figures
 * look like the measured ones, the plant model is validated. If they don't,
 * either the model or my understanding of the loop is wrong - and that is
 * worth knowing before trusting any of it.
 *
 * Everything the hardware run had is kept: the same setpoints, the same
 * differential ADC path (degC -> counts -> wire -> decode), the same
 * quantisation, the same +/-1 LSB jitter.
 *
 *   ./build/test_sim_run sim_run          -> sim_run.bin, sim_run_events.bin
 *   python3 scripts/parse_struct_dump.py sim_run.bin --events sim_run_events.bin -o sim_run
 */

#include "thermal_app.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "plant.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define MUTEX_NODE ((SemaphoreHandle_t)(intptr_t)3)
#define ZONE 0

/* Mirror of the firmware's sensor decode (thermal_app.c:80-99), which is
   static. The controller must see exactly what the sensor reported. */
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

/* ---- the recorded run, as a script -------------------------------------
 * Timings and setpoints taken from the hardware capture so the two figures
 * can be laid side by side.                                              */
#define T_START        20      /* start request                            */
#define T_LOAD_UP     600      /* user raises the load                     */
#define T_FAN_BLOCK  1010      /* airflow degraded -> throttle should engage */
#define T_FAN_CLEAR  1660      /* airflow restored                         */
#define T_SETPOINT   1665      /* setpoint moved 32 -> 33                  */
#define T_NODE_DROP  1669      /* node goes offline (the real bring-up fault) */
#define T_NODE_BACK  1798
#define T_STOP       1890
#define T_END        1930

static void log_sample_sim(float temp_c, float ema, float vnode, float fan,
                           float heater, float req, float setpoint, uint32_t rpm)
{
    uint32_t i = thermal_log_idx % THERM_LOG_LEN;
    thermal_log[i].time_s      = (float)HAL_GetTick() * 0.001f;
    thermal_log[i].temp_c      = temp_c;
    thermal_log[i].temp_ema    = ema;
    thermal_log[i].vnode       = vnode;
    thermal_log[i].fan_duty    = fan;
    thermal_log[i].heater_duty = heater;
    thermal_log[i].heater_req  = req;
    thermal_log[i].setpoint    = setpoint;
    thermal_log[i].fan_rpm     = rpm;
    thermal_log_idx++;
}

static void event_sim(uint8_t kind, uint8_t u8, float f)
{
    uint32_t i = thermal_event_idx % THERM_EVENT_LEN;
    thermal_events[i].time_s     = (float)HAL_GetTick() * 0.001f;
    thermal_events[i].kind       = kind;
    thermal_events[i].u8_payload = u8;
    thermal_events[i]._pad       = 0;
    thermal_events[i].f_payload  = f;
    thermal_event_idx++;
}

int main(int argc, char **argv)
{
    const char *stem = (argc > 1) ? argv[1] : "sim_run";

    fake_i2c_reset(); fake_task_reset(); fake_queue_reset(); fake_sem_reset();
    ThermalApp_Init();
    ThermalApp_StartTasks();
    for (int i = 0; i < NUM_REMOTE_NODES; i++) fake_i2c_set_present(i, 1);

    /* noise on, fan cooling on - the measured configuration */
    /* Ambient taken from the recorded run: the capture starts at ~29.8 C,
       which is a Tehran summer bench, not the 25 C nominal. */
    plant_init(29.6f, 1, 1);

    ZoneCtrl *z = &zones[ZONE];
    z->setpoint_c            = 32.0f;
    z->requested_heater_duty = 0.44f;

    uint8_t last_state = 0xFF, last_fault = 0xFF, online = 1;
    float   last_setpoint = -1000.0f;
    float   airflow = 1.0f;

    for (int t = 0; t < T_END; t++) {

        /* ---- scripted disturbances ---------------------------------- */
        if (t == T_START)      z->start_req = 1;
        if (t == T_LOAD_UP)    z->requested_heater_duty = 0.66f;
        if (t == T_FAN_BLOCK)  airflow = 0.62f;   /* fan moved away       */
        if (t == T_FAN_CLEAR)  airflow = 1.0f;
        if (t == T_SETPOINT)   z->setpoint_c = 33.0f;
        if (t == T_NODE_DROP)  { fake_i2c_set_present(ZONE, 0); }
        if (t == T_NODE_BACK)  { fake_i2c_set_present(ZONE, 1); }
        if (t == T_STOP)       z->stop_req = 1;

        /* ---- sensor: plant degC -> counts -> wire -------------------- */
        int      counts = plant_temp_to_counts(plant_temp_c());
        uint16_t wire   = plant_counts_to_wire(counts);
        fake_i2c_set_telemetry(ZONE, wire, plant_fan_rpm_to_tach(z->fan_duty));

        /* ---- bus: telemetry into shared memory ----------------------- */
        I2C_Telemetry tel;
        HAL_StatusTypeDef rx = HAL_I2C_Master_Receive(&hi2c1, (0x20 << 1),
                                   (uint8_t *)&tel, sizeof(tel), 10);
        xSemaphoreTake(MUTEX_NODE, portMAX_DELAY);
        if (rx == HAL_OK) { g_nodes[ZONE].tel = tel; g_nodes[ZONE].is_online = 1; }
        else              { g_nodes[ZONE].is_online = 0; }
        online = g_nodes[ZONE].is_online;
        xSemaphoreGive(MUTEX_NODE);

        /* ---- firmware decode + FSM ----------------------------------- */
        int      decoded = 0;
        float    temp_c  = 0.0f, vnode = 0.0f;
        uint32_t rpm     = 0;

        if (online) {
            decoded = (int)tel.adc_raw - 512;
            temp_c  = decode_counts_to_celsius(decoded);
            vnode   = 5.0f * ((float)decoded / 5120.0f + 10.0f / 340.0f);
            rpm     = (uint32_t)tel.tach_pulses * 30u;

            if (z->state == ST_PID || z->state == ST_THROTTLE)
                z->pid_temp += 0.02f * (temp_c - z->pid_temp);

            Zone_Tick(z, decoded, temp_c, z->pid_temp, rpm);

            if (temp_c > MAX_SAFE_TEMP_C) { z->heater_duty = 0.0f; z->fan_duty = 1.0f; }
        } else {
            /* thermal_app.c:409-413 - no telemetry means no trust */
            z->state        = ST_FAULT;
            z->fault_reason = FR_NODE_OFFLINE;
            z->heater_duty  = 0.0f;
            z->fan_duty     = 1.0f;
        }

        /* ---- events, same conditions as emit_change_events() --------- */
        if (z->state != last_state)        { event_sim(1, (uint8_t)z->state, 0.0f);
                                             last_state = (uint8_t)z->state; }
        if (z->fault_reason != last_fault) { event_sim(z->fault_reason == FR_NONE ? 3 : 2,
                                                 z->fault_reason == FR_NONE ? last_fault
                                                                            : (uint8_t)z->fault_reason,
                                                 0.0f);
                                             last_fault = (uint8_t)z->fault_reason; }
        if (z->setpoint_c != last_setpoint){ event_sim(4, 0, z->setpoint_c);
                                             last_setpoint = z->setpoint_c; }

        log_sample_sim(temp_c, z->pid_temp, vnode, z->fan_duty, z->heater_duty,
                       z->requested_heater_duty, z->setpoint_c, rpm);

        /* ---- actuators drive the plant into the next second ---------- */
        plant_step(z->heater_duty, z->fan_duty * airflow);
        fake_tick_advance(1000);
    }

    char path[256];
    snprintf(path, sizeof(path), "%s.bin", stem);
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return 1; }
    fwrite(thermal_log, sizeof(ThermalLogEntry), THERM_LOG_LEN, f);
    fclose(f);

    snprintf(path, sizeof(path), "%s_events.bin", stem);
    f = fopen(path, "wb");
    if (!f) { perror(path); return 1; }
    fwrite(thermal_events, sizeof(ThermalEvent), THERM_EVENT_LEN, f);
    fclose(f);

    printf("wrote %s.bin (%u samples) and %s_events.bin (%u events)\n",
           stem, (unsigned)thermal_log_idx, stem, (unsigned)thermal_event_idx);
    return 0;
}
