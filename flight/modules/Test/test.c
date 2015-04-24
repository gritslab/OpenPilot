#include "inc/test.h"
#include <openpilot.h>
#include "taskinfo.h"
#include <pios_com.h>

#include <manualcontrolcommand.h>
#include "positionstate.h"
#include "positiondesired.h"


#define STACK_SIZE_BYTES 520
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    200

static xTaskHandle taskHandle;
static void TestTask(void *parameters);

static uint32_t comPortFlex;

int32_t TestInitialize(void)
{
    comPortFlex = PIOS_COM_TELEM_RF;
    PIOS_COM_ChangeBaud(comPortFlex, 115200);

    ManualControlCommandInitialize();
    PositionStateInitialize();
    PositionDesiredInitialize();

    return 0;
}

int32_t TestStart(void)
{
    xTaskCreate(TestTask,
                "Test",
                STACK_SIZE_BYTES / 4,
                NULL,
                TASK_PRIORITY,
                &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_TEST, taskHandle);
    return 0;
}

MODULE_INITCALL(TestInitialize, TestStart);

static void TestTask(__attribute__((unused)) void *parameters)
{
    char writeBuffer[32];
    char tempBuffer[4];


    ManualControlCommandData cmd;
    ManualControlCommandGet(&cmd);
    uint8_t mode = cmd.FlightModeSwitchPosition;

    PositionStateData pos;
    PositionStateGet(&pos);

    PositionDesiredData posDesired;
    PositionDesiredGet(&posDesired);

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {

        // Get control mode
        ManualControlCommandGet(&cmd);
        mode = cmd.FlightModeSwitchPosition;
        intToStr(mode, tempBuffer, 2);

        strcpy(writeBuffer,"Mode: ");
        strcat(writeBuffer, tempBuffer);
        strcat(writeBuffer, "\n\r");
        PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));

        // Get position
        PositionStateGet(&pos);

        // strcpy(writeBuffer,"Pos: ");

        ftoa(pos.North, writeBuffer, 2);
        // strncat(writeBuffer, tempBuffer, 5);
        // strcat(writeBuffer, ", ");

        // ftoa(pos.East, tempBuffer, 2);
        // strncat(writeBuffer, tempBuffer, 5);
        // strcat(writeBuffer, ", ");

        // ftoa(pos.Down, tempBuffer, 2);
        // strncat(writeBuffer, tempBuffer, 5);
        // strcat(writeBuffer, ", ");

        PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));


        // Delay
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }

}

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

// Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    if (x < 0) {
        str[i++] = '-';
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

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * powf(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
