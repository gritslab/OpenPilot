#include "inc/positioncontrol.h"
#include <openpilot.h>
#include <taskinfo.h>
#include <pios_com.h>
#include <mathmisc.h>

#define STACK_SIZE_BYTES 612
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    100

#define GRAVITY 9.80665f
#define MASS 1.0f
#define FORCE_MAX 15.0f

// #define DEBUG

static xTaskHandle taskHandle;
static void PositionControlTask(void *parameters);


#ifdef DEBUG
static uint32_t comPortFlex;

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

int int2str(int x, char str[], int d)
{
    int i = 0;
    bool isNeg = false;
    if (x < 0) {
        isNeg = true;
        x = -1*x;
    }
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    if (isNeg) {
        str[i++] = '-';
    }
    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void float2str(float x, char *str, int afterpoint)
{
    bool isNeg = false;
    if (x < 0) {
        isNeg = true;
        x = -1*x;
    }

    // Extract integer part
    int ipart = (int)x;

    // Extract floating part
    float fpart = x - (float)ipart;

    // convert integer part to string
    int i = int2str(ipart, str, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        if (isNeg){
            str[i++] = '-';
        }
        str[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * powf(10, afterpoint);

        int2str((int)fpart, str + i + 1, afterpoint);
    }
}

void print(char* buffer)
{
    PIOS_COM_SendBuffer(comPortFlex,
                                   (uint8_t*)buffer,
                                   strlen(buffer));
}

void println(char* buffer)
{
    strcat(buffer, "\n\r");
    print(buffer);
}

void printVec(float vec[3])
{
    char buffer[16];

    for (int i=0; i<3; ++i) {
        float2str(vec[i], buffer, 2);
        strcat(buffer, ", ");
        print(buffer);
    }
    strcpy(buffer," ");
    println(buffer);
}

void printMat(float mat[3][3])
{
    char buffer[16];

    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            float2str(mat[i][j], buffer, 2);
            strcat(buffer, ", ");
            print(buffer);
        }
        strcpy(buffer," ");
        println(buffer);
    }
    strcpy(buffer," ");
    println(buffer);
}

#endif


int32_t PositionControlInitialize(void)
{
    ManualControlCommandInitialize();
    PositionStateInitialize();
    PositionDesiredInitialize();
    StabilizationDesiredInitialize();

#ifdef DEBUG
    comPortFlex = PIOS_COM_TELEM_RF;
    PIOS_COM_ChangeBaud(comPortFlex, 115200);
#endif

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
    struct pid posPid[3];

    pid_zero(&posPid[0]);
    pid_zero(&posPid[1]);
    pid_zero(&posPid[2]);

    pid_configure(&posPid[0], 0.0f, 0.0f, 0.0f, 0);
    pid_configure(&posPid[1], 0.0f, 0.0f, 0.0f, 0);
    pid_configure(&posPid[2], 0.0f, 0.0f, 0.0f, 0);

    ManualControlCommandData cmd;

    AttitudeStateData att;
    StabilizationDesiredData attDesired;
    float prevYaw = 0;
    float prevThrust = 0;

    PositionStateData pos;
    PositionDesiredData posDesired;

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        ManualControlCommandGet(&cmd);
        PositionStateGet(&pos);
        // PositionDesiredGet(&posDesired);

        if (cmd.FlightModeSwitchPosition == 1) {
            attDesired.Yaw = prevYaw;
            attDesired.Thrust = prevThrust;
            performControl(posPid, &attDesired, &pos, &posDesired);
        } else {
            AttitudeStateGet(&att);
            prevYaw = att.Yaw;
            prevThrust = cmd.Thrust;
            posDesired.North = pos.North;
            posDesired.East = pos.East;
            posDesired.Down = pos.Down;
        }
        PositionDesiredSet(&posDesired);

        // Delay
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }

}

void performControl(struct pid *posPid,
                    StabilizationDesiredData* attDesired,
                    PositionStateData* pos,
                    PositionDesiredData* posDesired)
{
    float dt = UPDATE_PERIOD_MS / 1000.0f;

    float f[3];
    f[0] = pid_apply(&(posPid[0]), posDesired->North - pos->North, dt) * MASS;
    f[1] = pid_apply(&(posPid[1]), posDesired->East - pos->East, dt) * MASS;
    f[2] = pid_apply(&(posPid[2]), posDesired->Down - pos->Down, dt) * MASS;
    // f[2] -= GRAVITY*MASS;
    float thrust = attDesired->Thrust * FORCE_MAX;
    thrust = thrust > 0 ? thrust : 0;
    f[2] -= thrust;

    calcAttFromForceVec(f, attDesired->Yaw, attDesired);
    StabilizationDesiredSet(attDesired);

// #ifdef DEBUG
//     char buffer[16];

//     for (int i=0; i<3; ++i) {
//         float2str(f[i], buffer, 2);
//         strcat(buffer, ", ");
//         print(buffer);
//     }
//     strcpy(buffer," ");
//     println(buffer);
// #endif

// #ifdef DEBUG
    // char buffer[16];
    // float2str(att.Roll, buffer, 2);
    // strcat(buffer, ", ");
    // print(buffer);
    // float2str(att.Pitch, buffer, 2);
    // strcat(buffer, ", ");
    // print(buffer);
    // float2str(att.Yaw, buffer, 2);
    // strcat(buffer, ", ");
    // print(buffer);
    // float2str(att.Thrust, buffer, 2);
    // println(buffer);
// #endif


}

void calcAttFromForceVec(float f[3], float yaw, StabilizationDesiredData* att)
{
    float eps = 1.0e-3f; // Size of closeness ball

    float fNorm = vector_lengthf(f, 3);

    if (fNorm < eps) {
        att->Roll = rad2deg(0);
        att->Pitch = rad2deg(0);
        att->Yaw = yaw;
        att->Thrust = forceNorm2thrustPercentage(fNorm);
        return;
    }

    float X[3];
    float Y[3];
    float Z[3] = {-f[0], -f[1], -f[2]};
    vector_normalizef(Z, 3);

    // Solve for Y axis
    float a = -Z[0]*sinf(yaw) + Z[1]*cosf(yaw);

    float d;
    if (a < eps && a > -eps) {
        Y[2] = 0.0f;
        d = sqrtf(1 - powf(Y[2], 2));
        Y[0] = -d*sinf(yaw);
        Y[1] = d*cosf(yaw);
    } else {
        Y[2] = sqrtf(1 / ( powf(Z[2]/a,2) + 1) );
        d = sqrtf(1 - powf(Y[2], 2));
        Y[0] = -d*sinf(yaw);
        Y[1] = d*cosf(yaw);
        float b = Y[0]*Z[0] + Y[1]*Z[1] + Y[2]*Z[2];
        if (b > eps || b < -eps) {
            Y[2] = -sqrtf(1 / ( powf(Z[2]/a,2) + 1) );
            d = sqrtf(1 - powf(Y[2], 2));
            Y[0] = -d*sinf(yaw);
            Y[1] = d*cosf(yaw);
        }
    }
    vector_normalizef(Y, 3);

    // Solve for X axis = Y cross Z
    X[0] = Y[1]*Z[2] - Y[2]*Z[1];
    X[1] = Y[2]*Z[0] - Y[0]*Z[2];
    X[2] = Y[0]*Z[1] - Y[1]*Z[0];
    vector_normalizef(X,3);

    // Construct rotation matrix
    float R[3][3];
    R[0][0] = X[0]; R[0][1] = Y[0]; R[0][2] = Z[0];
    R[1][0] = X[1]; R[1][1] = Y[1]; R[1][2] = Z[1];
    R[2][0] = X[2]; R[2][1] = Y[2]; R[2][2] = Z[2];

#ifdef DEBUG
    printMat(R);
#endif

    toEuler(R, &(att->Roll), &(att->Pitch), &(att->Yaw));
    att->Roll = rad2deg(att->Roll);
    att->Pitch = rad2deg(att->Pitch);
    att->Yaw = rad2deg(att->Yaw);
    att->Thrust = forceNorm2thrustPercentage(fNorm);

    return;
}

float forceNorm2thrustPercentage(float f)
{
    f = boundf(f, 0.0f, FORCE_MAX);
    return f / FORCE_MAX;
}
