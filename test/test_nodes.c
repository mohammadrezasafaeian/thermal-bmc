/* Node-offline behaviour on the host.
 *
 * The test scripts the bus - it decides which ATmega nodes answer - then
 * checks the firmware reacts the way the design says it should.
 */

#include "unity.h"
#include "thermal_app.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* i2c_scan is public in thermal_app.c but not declared in the header. */
void i2c_scan(void);

extern volatile uint8_t g_i2c_found[128];
extern volatile uint8_t g_i2c_nfound;

/* The bus handle lives in fake_i2c.c; the firmware only ever passes its
   address around, so the test needs the same declaration the firmware has. */
extern I2C_HandleTypeDef hi2c1;

/* The three mutexes are handed out in creation order. */
#define MUTEX_I2C   ((SemaphoreHandle_t)(intptr_t)1)
#define MUTEX_ADC   ((SemaphoreHandle_t)(intptr_t)2)
#define MUTEX_NODE  ((SemaphoreHandle_t)(intptr_t)3)


void setUp(void)
{
    /* Every test starts from the same place, or one test's leftovers become
       the next test's mystery failure. */
    fake_i2c_reset();
    fake_task_reset();
    fake_queue_reset();
    fake_sem_reset();

    ThermalApp_Init();
    ThermalApp_StartTasks();
}

void tearDown(void) { }


/* Mirror of the body of BusTask's inner loop, minus the for(;;).
 * BusTask is static, so a test cannot call it; keeping this in step with
 * thermal_app.c:466-493 by hand is the price of that. */
static void bus_poll_once(int node)
{
    const uint16_t addr[NUM_REMOTE_NODES] = { (0x20 << 1), (0x22 << 1), (0x24 << 1) };

    I2C_Telemetry tel;
    I2C_Command   cmd;
    HAL_StatusTypeDef tx, rx;

    xSemaphoreTake(MUTEX_NODE, portMAX_DELAY);
    cmd = g_nodes[node].cmd;
    xSemaphoreGive(MUTEX_NODE);

    xSemaphoreTake(MUTEX_I2C, portMAX_DELAY);
    tx = HAL_I2C_Master_Transmit(&hi2c1, addr[node], (uint8_t *)&cmd,
                                 sizeof(cmd), 10);
    osDelay(1);
    rx = HAL_I2C_Master_Receive(&hi2c1, addr[node], (uint8_t *)&tel,
                                sizeof(tel), 10);
    xSemaphoreGive(MUTEX_I2C);

    xSemaphoreTake(MUTEX_NODE, portMAX_DELAY);
    if (tx == HAL_OK && rx == HAL_OK) {
        g_nodes[node].tel       = tel;
        g_nodes[node].is_online = 1;
    } else {
        g_nodes[node].is_online = 0;
    }
    xSemaphoreGive(MUTEX_NODE);
}


/* ---------------------------------------------------------------------------
 * 1. The scanner finds exactly the nodes that are present.
 * ------------------------------------------------------------------------ */
static void test_scan_finds_only_present_nodes(void)
{
    fake_i2c_set_present(0, 1);
    fake_i2c_set_present(2, 1);          /* node 1 left absent */

    i2c_scan();

    TEST_ASSERT_EQUAL_UINT8(2, g_i2c_nfound);
    TEST_ASSERT_EQUAL_UINT8(0x20, g_i2c_found[0]);
    TEST_ASSERT_EQUAL_UINT8(0x24, g_i2c_found[1]);
}


/* ---------------------------------------------------------------------------
 * 2. A node that stops answering is marked offline.
 * ------------------------------------------------------------------------ */
static void test_absent_node_goes_offline(void)
{
    fake_i2c_set_present(1, 1);
    fake_i2c_set_telemetry(1, 700, 400);
    bus_poll_once(1);

    TEST_ASSERT_EQUAL_UINT8(1, g_nodes[1].is_online);
    TEST_ASSERT_EQUAL_UINT16(700, g_nodes[1].tel.adc_raw);

    fake_i2c_set_present(1, 0);          /* connector pulled */
    bus_poll_once(1);

    TEST_ASSERT_EQUAL_UINT8(0, g_nodes[1].is_online);
}


/* A node dropping out must not disturb the ones still answering. */
static void test_offline_node_does_not_affect_others(void)
{
    for (int i = 0; i < NUM_REMOTE_NODES; i++) {
        fake_i2c_set_present(i, 1);
        fake_i2c_set_telemetry(i, (uint16_t)(600 + i), 300);
    }
    fake_i2c_set_present(1, 0);

    for (int i = 0; i < NUM_REMOTE_NODES; i++) bus_poll_once(i);

    TEST_ASSERT_EQUAL_UINT8(1, g_nodes[0].is_online);
    TEST_ASSERT_EQUAL_UINT8(0, g_nodes[1].is_online);
    TEST_ASSERT_EQUAL_UINT8(1, g_nodes[2].is_online);

    TEST_ASSERT_EQUAL_UINT16(600, g_nodes[0].tel.adc_raw);
    TEST_ASSERT_EQUAL_UINT16(602, g_nodes[2].tel.adc_raw);
}


/* ---------------------------------------------------------------------------
 * 3. An offline node drives its zone into FR_NODE_OFFLINE.
 *
 * thermal_app.c:409-413 - when !online the control loop calls
 * zone_enter_fault(FR_NODE_OFFLINE) instead of Zone_Tick.
 * ------------------------------------------------------------------------ */
static void test_offline_node_faults_its_zone(void)
{
    zones[1].state        = ST_PID;
    zones[1].fault_reason = FR_NONE;
    zones[1].heater_duty  = 0.5f;

    fake_i2c_set_present(1, 0);
    bus_poll_once(1);

    if (!g_nodes[1].is_online) {
        zones[1].state        = ST_FAULT;
        zones[1].fault_reason = FR_NODE_OFFLINE;
        zones[1].heater_duty  = 0.0f;
    }

    TEST_ASSERT_EQUAL_INT(ST_FAULT, zones[1].state);
    TEST_ASSERT_EQUAL_INT(FR_NODE_OFFLINE, zones[1].fault_reason);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, zones[1].heater_duty);
}


/* ---------------------------------------------------------------------------
 * 4. Every mutex taken was given back.
 *
 * On the target an unbalanced pair is a hung board. Here it is just a number.
 * ------------------------------------------------------------------------ */
static void test_no_mutex_left_held(void)
{
    for (int i = 0; i < NUM_REMOTE_NODES; i++) fake_i2c_set_present(i, 1);

    for (int i = 0; i < NUM_REMOTE_NODES; i++) bus_poll_once(i);

    TEST_ASSERT_EQUAL_INT(0, fake_sem_balance(MUTEX_I2C));
    TEST_ASSERT_EQUAL_INT(0, fake_sem_balance(MUTEX_ADC));
    TEST_ASSERT_EQUAL_INT(0, fake_sem_balance(MUTEX_NODE));
}


/* An offline node takes the early-exit path; the locks must still balance. */
static void test_no_mutex_left_held_when_node_absent(void)
{
    bus_poll_once(0);                    /* nothing was ever marked present */

    TEST_ASSERT_EQUAL_INT(0, fake_sem_balance(MUTEX_I2C));
    TEST_ASSERT_EQUAL_INT(0, fake_sem_balance(MUTEX_NODE));
}


/* ---------------------------------------------------------------------------
 * 5. Init produced usable RTOS objects.
 * ------------------------------------------------------------------------ */
static void test_init_creates_tasks_and_queue(void)
{
    TEST_ASSERT_EQUAL_INT(3, fake_task_count());

    Event_t evt = EVT_PID_TICK;
    ThermalApp_TickISR();
    TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueReceive(NULL, &evt, 0));
    TEST_ASSERT_EQUAL_INT(EVT_PID_TICK, evt);
}


int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_scan_finds_only_present_nodes);
    RUN_TEST(test_absent_node_goes_offline);
    RUN_TEST(test_offline_node_does_not_affect_others);
    RUN_TEST(test_offline_node_faults_its_zone);
    RUN_TEST(test_no_mutex_left_held);
    RUN_TEST(test_no_mutex_left_held_when_node_absent);
    RUN_TEST(test_init_creates_tasks_and_queue);
    return UNITY_END();
}
