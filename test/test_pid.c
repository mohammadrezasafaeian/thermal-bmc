#include "unity.h"
#include "pid.h"

/* gains from thermal_app.h */
#define KP      (-0.68f)
#define KI      (-0.0061f)
#define KD      ( 0.0f)
#define TS      ( 1.0f)
#define TAU_F   (10.0f)

static PID_Handle pid;

void setUp(void)
{
    PID_Init(&pid, KP, KI, KD, TS, TAU_F, 25.0f);
    pid.out_min = 0.0f;
    pid.out_max = 1.0f;
}

void tearDown(void) { }

void test_init_clears_state(void)
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.deriv);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.e_prev);
    TEST_ASSERT_EQUAL_FLOAT(25.0f, pid.meas_prev);
}

void test_alpha_matches_formula(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f / 11.0f, pid.alpha);
}

void test_reverse_acting_hotter_means_more_fan(void)
{
    float u_at_setpoint = PID_Update(&pid, 34.0f, 34.0f);
    setUp();
    float u_when_hot = PID_Update(&pid, 34.0f, 40.0f);

    TEST_ASSERT_TRUE_MESSAGE(u_when_hot > u_at_setpoint,
        "Kp is negative: hot plant must raise fan duty");
}

void test_output_stays_within_limits(void)
{
    for (int k = 0; k < 500; k++) {
        float u = PID_Update(&pid, 34.0f, 90.0f);
        TEST_ASSERT_TRUE(u <= pid.out_max + 1e-6f);
        TEST_ASSERT_TRUE(u >= pid.out_min - 1e-6f);
    }
}

void test_integrator_frozen_while_saturated(void)
{
    for (int k = 0; k < 50; k++)
        (void)PID_Update(&pid, 34.0f, 90.0f);

    float i_after_50 = pid.integral;

    for (int k = 0; k < 200; k++)
        (void)PID_Update(&pid, 34.0f, 90.0f);

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-4f, i_after_50, pid.integral,
        "integrator grew while output was saturated");
}

void test_recovers_after_saturation(void)
{
    for (int k = 0; k < 200; k++)
        (void)PID_Update(&pid, 34.0f, 90.0f);

    int steps = 0;
    float u = 1.0f;
    while (u > 0.05f && steps < 400) {
        u = PID_Update(&pid, 34.0f, 20.0f);
        steps++;
    }
    TEST_ASSERT_TRUE_MESSAGE(steps < 400, "output never came back down");
}

void test_no_derivative_kick_on_setpoint_step(void)
{
    /* step the setpoint DOWN: Kp is negative, so stepping up would clamp
       at out_min and the assertion would prove nothing */
    PID_Handle p;
    PID_Init(&p, KP, KI, -0.5f, TS, TAU_F, 34.0f);
    p.out_min = -10.0f;
    p.out_max = 10.0f;

    float u_before = PID_Update(&p, 34.0f, 34.0f);
    float u_after  = PID_Update(&p, 30.0f, 34.0f);

    float e = 30.0f - 34.0f;
    float expected = KP * e + 0.5f * KI * TS * e;

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f, expected, u_after - u_before,
        "measurement never moved, so only P and I may change");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_clears_state);
    RUN_TEST(test_alpha_matches_formula);
    RUN_TEST(test_reverse_acting_hotter_means_more_fan);
    RUN_TEST(test_output_stays_within_limits);
    RUN_TEST(test_integrator_frozen_while_saturated);
    RUN_TEST(test_recovers_after_saturation);
    RUN_TEST(test_no_derivative_kick_on_setpoint_step);
    return UNITY_END();
}
