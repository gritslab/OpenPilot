#include "inc/positioncontrol.h"
#include <openpilot.h>
#include <taskinfo.h>
#include <pios_com.h>
#include <mathmisc.h>

#define STACK_SIZE_BYTES 724
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    20

#define GRAVITY 9.80665f
#define MASS 1.0f
#define MAX_FORCE 15.0f // [N] max force from all 4 motors
#define MAX_BANK_ANG 0.2618f // [radians] (15 degrees)
#define MAX_Z_ACC 2.0f // [m/s^2]
#define EPSILON 1.0e-3f // size of zero ball

// Default PID values
#define XY_P 0.5f
#define XY_D 0.55f
#define Z_P 1.0f
#define Z_I 0.0f
#define Z_D 0.3f




static xTaskHandle taskHandle;
static void PositionControlTask(void *parameters);

int32_t PositionControlInitialize(void)
{
    ManualControlCommandInitialize();
    PositionStateInitialize();
    PositionDesiredInitialize();
    StabilizationDesiredInitialize();
    PositionPidInitialize();

    return 0;
}

int32_t PositionControlStart(void)
{
    xTaskCreate(PositionControlTask,
                "PositionControl",
                STACK_SIZE_BYTES / 4,
                NULL,
                TASK_PRIORITY,
                &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_POSITIONCONTROL, taskHandle);
    return 0;
}

MODULE_INITCALL(PositionControlInitialize, PositionControlStart);

static void PositionControlTask(__attribute__((unused)) void *parameters)
{
    PositionPidData posPidValues;
    posPidValues.xy_p = XY_P;
    posPidValues.xy_d = XY_D;
    posPidValues.z_p = Z_P;
    posPidValues.z_i = Z_I;
    posPidValues.z_d = Z_D;
    PositionPidSet(&posPidValues);

    struct pid posPid[3];

    pid_zero(&posPid[0]);
    pid_zero(&posPid[1]);
    pid_zero(&posPid[2]);

    // pid_configure(&posPid[0], 0.75f, 0.0f, 1.0f, 0);
    // pid_configure(&posPid[1], 0.75f, 0.0f, 1.0f, 0);
    // pid_configure(&posPid[2], 2.0f, 1.0f, 0.75f, 0);

    ManualControlCommandData cmd;

    AttitudeStateData att;
    StabilizationDesiredData attDesired;
    attDesired.StabilizationMode.Roll = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
    attDesired.StabilizationMode.Pitch = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
    attDesired.StabilizationMode.Yaw = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
    attDesired.StabilizationMode.Thrust = STABILIZATIONDESIRED_STABILIZATIONMODE_MANUAL;

    float up_thrust = 0.0f;

    PositionStateData pos;
    PositionDesiredData posDesired;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        ManualControlCommandGet(&cmd);
        PositionStateGet(&pos);

        if (cmd.FlightModeSwitchPosition == 1) {

            PositionDesiredGet(&posDesired);

            PositionPidGet(&posPidValues);

            pid_configure(&posPid[0], posPidValues.xy_p, 0.0f, posPidValues.xy_d, 0);
            pid_configure(&posPid[1], posPidValues.xy_p, 0.0f, posPidValues.xy_d, 0);
            pid_configure(&posPid[2], posPidValues.z_p, posPidValues.z_i, posPidValues.z_d, 2*posPidValues.z_i);

            performControl(posPid, &pos, &posDesired, &up_thrust, &attDesired);
            StabilizationDesiredSet(&attDesired);

        } else {
            AttitudeStateGet(&att);
            up_thrust = cmd.Thrust;
            attDesired.Yaw = att.Yaw;
            posDesired.North = pos.North;
            posDesired.East = pos.East;
            posDesired.Down = pos.Down;

            PositionDesiredSet(&posDesired);
        }

        // Delay
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }

}

void performControl(struct pid *posPid,
                    PositionStateData* pos,
                    PositionDesiredData* posDesired,
                    float* up_thrust,
                    StabilizationDesiredData* att)
{
    // Calculate force vector from PID gains and error vector
    float dt = UPDATE_PERIOD_MS / 1000.0f;
    float f[3];
    f[0] = pid_apply(&(posPid[0]), posDesired->North - pos->North, dt) * MASS;
    f[1] = pid_apply(&(posPid[1]), posDesired->East - pos->East, dt) * MASS;
    f[2] = pid_apply(&(posPid[2]), posDesired->Down - pos->Down, dt) * MASS;

    // Constrain z force to acceleration constrains
    if (fabsf(f[2] / MASS) > MAX_Z_ACC) {
        f[2] = f[2] / fabsf(f[2] / MASS) * MAX_Z_ACC;
    }

    // Add in necessary z force to counteract gravity
    // f[2] -= GRAVITY*MASS;
    f[2] -= (*up_thrust > 0 ? *up_thrust : 0) * MAX_FORCE;

    // Constrain magnitude of force in the xy direction to maximum bank angle
    float fxy = 1 / fast_invsqrtf(f[0]*f[0] + f[1]*f[1]);
    if (atan2f(fxy,-f[2]) > MAX_BANK_ANG) {
        f[0] = -f[2]*tanf(MAX_BANK_ANG)/fxy*f[0];
        f[1] = -f[2]*tanf(MAX_BANK_ANG)/fxy*f[1];
        fxy = 1 / fast_invsqrtf(f[0]*f[0] + f[1]*f[1]);
    }

    // Constrain norm of force in the max force
    float f_norm = vector_lengthf(f, 3);
    if (f_norm > MAX_FORCE) {
        float fxy_scale = 1 / (fxy*fast_invsqrtf(MAX_FORCE*MAX_FORCE - f[2]*f[2]));
        f[0] = fxy_scale * f[0];
        f[1] = fxy_scale * f[1];
        f_norm = MAX_FORCE;
    }

    // Case 1: Force is straight up
    if (f_norm < EPSILON) {
        att->Roll = 0.0f;
        att->Pitch = 0.0f;
        att->Thrust = forceNorm2thrustPercentage(f_norm);
        return;
    }

    // Case 2: Force not straight up
    float yaw = deg2rad(att->Yaw);
    float X[3];
    float Y[3];
    float Z[3] = {-f[0], -f[1], -f[2]};

    // Solve for Z axis of rotation matrix
    vector_normalizef(Z, 3);

    // Solve for X axis of rotation matrix
    float a = Z[0]*cosf(yaw) + Z[1]*sinf(yaw);
    if (fabsf(a) < EPSILON) {
        X[2] = 0.0f;
        X[0] = cosf(yaw);
        X[1] = sinf(yaw);
    } else {
        X[2] = fast_invsqrtf( (Z[2]*Z[2])/(a*a) + 1.0f );
        float d = 1 / fast_invsqrtf(1.0f-X[2]*X[2]);
        X[0] = d*cosf(yaw);
        X[1] = d*sinf(yaw);
        vector_normalizef(X, 3);
        float b = X[0]*Z[0] + X[1]*Z[1] + X[2]*Z[2];
        if (fabsf(b) > EPSILON) {
            X[2] = -X[2];
        }
    }

    // Solve for Y axis of rotation matrix
    Y[0] = Z[1]*X[2] - Z[2]*X[1];
    Y[1] = Z[2]*X[0] - Z[0]*X[2];
    Y[2] = Z[0]*X[1] - Z[1]*X[0];

    // Construct rotation matrix
    float R[3][3];
    R[0][0] = X[0]; R[0][1] = Y[0]; R[0][2] = Z[0];
    R[1][0] = X[1]; R[1][1] = Y[1]; R[1][2] = Z[1];
    R[2][0] = X[2]; R[2][1] = Y[2]; R[2][2] = Z[2];

    toEuler(R, &(att->Roll), &(att->Pitch), &(att->Yaw));
    att->Roll = rad2deg(att->Roll);
    att->Pitch = rad2deg(att->Pitch);
    att->Yaw = rad2deg(att->Yaw);
    att->Thrust = forceNorm2thrustPercentage(f_norm);
}

float forceNorm2thrustPercentage(float f)
{
    f = boundf(f, 0.0f, MAX_FORCE);
    return f / MAX_FORCE;
}
