#include "icm_ins.h"
#include "icm_attitude.h"

#include <math.h>

#define INS_GRAVITY_MS2      9.80665f

/* ZUPT: both conditions must hold for ZUPT_HOLD samples (1 sample = 1 ms). */
/* Gyro threshold is intentionally relaxed to tolerate residual bias error.  */
#define ZUPT_ACC_LO          0.96f   /* g */
#define ZUPT_ACC_HI          1.04f   /* g */
#define ZUPT_GYRO_THRESH     10.0f   /* dps, per axis */
#define ZUPT_HOLD            200U    /* samples to enter stationary  = 200 ms */
#define ZUPT_EXIT_COUNT      100U    /* samples to leave stationary  = 100 ms */

typedef struct
{
    float  vel_x_ms;
    float  vel_y_ms;
    float  vel_z_ms;
    float  pos_x_m;
    float  pos_y_m;
    float  lin_ax_ms2;
    float  lin_ay_ms2;
    float  lin_az_ms2;
    uint32 zupt_count;
    uint32 zupt_exit_count;
    uint8  is_stationary;
} icm_ins_state_t;

static volatile icm_ins_state_t g_icm_ins = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0U, 0U,
    0U
};

void icm_ins_init(void)
{
    g_icm_ins.vel_x_ms       = 0.0f;
    g_icm_ins.vel_y_ms       = 0.0f;
    g_icm_ins.vel_z_ms       = 0.0f;
    g_icm_ins.pos_x_m        = 0.0f;
    g_icm_ins.pos_y_m        = 0.0f;
    g_icm_ins.lin_ax_ms2     = 0.0f;
    g_icm_ins.lin_ay_ms2     = 0.0f;
    g_icm_ins.lin_az_ms2     = 0.0f;
    g_icm_ins.zupt_count     = 0U;
    g_icm_ins.zupt_exit_count = 0U;
    g_icm_ins.is_stationary  = 0U;
}

void icm_ins_reset_velocity(void)
{
    g_icm_ins.vel_x_ms = 0.0f;
    g_icm_ins.vel_y_ms = 0.0f;
    g_icm_ins.vel_z_ms = 0.0f;
}

void icm_ins_reset_position(void)
{
    g_icm_ins.pos_x_m = 0.0f;
    g_icm_ins.pos_y_m = 0.0f;
}

void icm_ins_set_velocity(float vx_ms, float vy_ms, float vz_ms)
{
    g_icm_ins.vel_x_ms = vx_ms;
    g_icm_ins.vel_y_ms = vy_ms;
    g_icm_ins.vel_z_ms = vz_ms;
}

/* Called at 1 kHz from ISR, after icm_attitude_update().              */
/* Pipeline: body acc -> rotate to nav frame -> remove gravity -> m/s^2 */
/*           ZUPT check -> integrate velocity -> integrate position      */
void icm_ins_update(float acc_x_g, float acc_y_g, float acc_z_g,
                    float gyro_x_dps, float gyro_y_dps, float gyro_z_dps,
                    float dt_s)
{
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 0.0f;
    float acc_norm = 0.0f;

    if (dt_s <= 0.0f)
    {
        return;
    }

    if (!icm_attitude_is_gyro_bias_valid())
    {
        return;
    }

    /* ZUPT state machine with entry and exit hysteresis.
     * Entry: condition must hold for ZUPT_HOLD  consecutive samples (200 ms).
     * Exit : condition must fail  for ZUPT_EXIT_COUNT samples (100 ms) while
     *        already stationary -- prevents flickering from momentary noise. */
    acc_norm = icm_attitude_get_acc_norm_g();
    if ((acc_norm > ZUPT_ACC_LO) && (acc_norm < ZUPT_ACC_HI) &&
        (gyro_x_dps > -ZUPT_GYRO_THRESH) && (gyro_x_dps < ZUPT_GYRO_THRESH) &&
        (gyro_y_dps > -ZUPT_GYRO_THRESH) && (gyro_y_dps < ZUPT_GYRO_THRESH) &&
        (gyro_z_dps > -ZUPT_GYRO_THRESH) && (gyro_z_dps < ZUPT_GYRO_THRESH))
    {
        g_icm_ins.zupt_exit_count = 0U;
        if (g_icm_ins.zupt_count < ZUPT_HOLD)
        {
            g_icm_ins.zupt_count++;
        }
    }
    else
    {
        g_icm_ins.zupt_count = 0U;
        if (g_icm_ins.is_stationary)
        {
            g_icm_ins.zupt_exit_count++;
            if (g_icm_ins.zupt_exit_count >= ZUPT_EXIT_COUNT)
            {
                g_icm_ins.is_stationary = 0U;
                g_icm_ins.zupt_exit_count = 0U;
            }
        }
    }

    if (g_icm_ins.zupt_count >= ZUPT_HOLD)
    {
        g_icm_ins.vel_x_ms = 0.0f;
        g_icm_ins.vel_y_ms = 0.0f;
        g_icm_ins.vel_z_ms = 0.0f;
        g_icm_ins.is_stationary = 1U;
        g_icm_ins.zupt_exit_count = 0U;
        g_icm_ins.lin_ax_ms2 = 0.0f;
        g_icm_ins.lin_ay_ms2 = 0.0f;
        g_icm_ins.lin_az_ms2 = 0.0f;
        return;
    }

    /* Exit hysteresis active: still considered stationary, keep vel = 0 */
    if (g_icm_ins.is_stationary)
    {
        g_icm_ins.vel_x_ms = 0.0f;
        g_icm_ins.vel_y_ms = 0.0f;
        g_icm_ins.lin_ax_ms2 = 0.0f;
        g_icm_ins.lin_ay_ms2 = 0.0f;
        g_icm_ins.lin_az_ms2 = 0.0f;
        return;
    }

    /* Rotate body -> nav frame, remove gravity (Z-up ENU) */
    icm_attitude_get_quaternion(&q0, &q1, &q2, &q3);

    nx = (q0*q0+q1*q1-q2*q2-q3*q3)*acc_x_g
       + 2.0f*(q1*q2-q0*q3)*acc_y_g
       + 2.0f*(q1*q3+q0*q2)*acc_z_g;
    ny = 2.0f*(q1*q2+q0*q3)*acc_x_g
       + (q0*q0-q1*q1+q2*q2-q3*q3)*acc_y_g
       + 2.0f*(q2*q3-q0*q1)*acc_z_g;
    nz = 2.0f*(q1*q3-q0*q2)*acc_x_g
       + 2.0f*(q2*q3+q0*q1)*acc_y_g
       + (q0*q0-q1*q1-q2*q2+q3*q3)*acc_z_g - 1.0f;

    g_icm_ins.lin_ax_ms2 = nx * INS_GRAVITY_MS2;
    g_icm_ins.lin_ay_ms2 = ny * INS_GRAVITY_MS2;
    g_icm_ins.lin_az_ms2 = nz * INS_GRAVITY_MS2;

    /* Velocity integration */
    g_icm_ins.vel_x_ms += g_icm_ins.lin_ax_ms2 * dt_s;
    g_icm_ins.vel_y_ms += g_icm_ins.lin_ay_ms2 * dt_s;

    /* Position integration (horizontal only) */
    g_icm_ins.pos_x_m += g_icm_ins.vel_x_ms * dt_s;
    g_icm_ins.pos_y_m += g_icm_ins.vel_y_ms * dt_s;
}

void icm_ins_get_velocity(float *vx_ms, float *vy_ms, float *vz_ms)
{
    if (NULL != vx_ms) *vx_ms = g_icm_ins.vel_x_ms;
    if (NULL != vy_ms) *vy_ms = g_icm_ins.vel_y_ms;
    if (NULL != vz_ms) *vz_ms = g_icm_ins.vel_z_ms;
}

void icm_ins_get_position(float *px_m, float *py_m)
{
    if (NULL != px_m) *px_m = g_icm_ins.pos_x_m;
    if (NULL != py_m) *py_m = g_icm_ins.pos_y_m;
}

void icm_ins_get_linear_acc(float *ax_ms2, float *ay_ms2, float *az_ms2)
{
    if (NULL != ax_ms2) *ax_ms2 = g_icm_ins.lin_ax_ms2;
    if (NULL != ay_ms2) *ay_ms2 = g_icm_ins.lin_ay_ms2;
    if (NULL != az_ms2) *az_ms2 = g_icm_ins.lin_az_ms2;
}

float icm_ins_get_speed_ms(void)
{
    float vx = g_icm_ins.vel_x_ms;
    float vy = g_icm_ins.vel_y_ms;
    return sqrtf(vx*vx + vy*vy);
}

uint8 icm_ins_is_stationary(void)
{
    return g_icm_ins.is_stationary;
}
