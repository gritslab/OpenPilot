#include "inc/motioncapture.h"

#include <openpilot.h>
#include "taskinfo.h"
#include "positionstate.h"
#include "homelocation.h"

#include <pios_com.h>

#define STACK_SIZE_BYTES 512
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    10

static xTaskHandle taskHandle;
static void motionCaptureTask(void *parameters);

static uint32_t comPortMain;
static uint32_t comPortFlex;

// struct PosData {
//     char x;
//     char y;
//     char z;
// };

int32_t MotiontCaptureStart(void)
{
    xTaskCreate(motionCaptureTask,
                "MotiontCapture",
                STACK_SIZE_BYTES / 4,
                NULL,
                TASK_PRIORITY,
                &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_MOTIONCAPTURE, taskHandle);
    return 0;
}

int32_t MotionCaptureInitialize(void)
{
    PositionStateInitialize();

    // Set home location
    HomeLocationInitialize();
    HomeLocationData home;
    HomeLocationGet(&home);
    home.Latitude = 0;
    home.Longitude = 0;
    home.Altitude = 0;
    home.Set = HOMELOCATION_SET_TRUE;
    HomeLocationSet(&home);

    // Set port
    comPortMain = PIOS_COM_GPS;
    comPortFlex = PIOS_COM_TELEM_RF;

    return 0;
}

MODULE_INITCALL(MotionCaptureInitialize, MotiontCaptureStart);

static void motionCaptureTask(__attribute__((unused)) void *parameters)
{
    //This data struct is contained in the automatically generated UAVObject code
    PositionStateData data;

    //Populate the data struct with the UAVObject's current values
    PositionStateGet(&data);

    portTickType readDelay = 100 / portTICK_RATE_MS;
    uint8_t BUFFER_SIZE = 128;
    char readBuffer[BUFFER_SIZE];
    char writeBuffer[BUFFER_SIZE];
    uint16_t cnt;

    // PosData pos;

    TickType_t lastWakeTime  = xTaskGetTickCount();
    while(1) {
        while ((cnt = PIOS_COM_ReceiveBuffer(comPortMain, (uint8_t *)readBuffer, BUFFER_SIZE, readDelay)) > 0) {
            strncpy(writeBuffer, readBuffer, cnt);
            PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, cnt);
        }

        //Get a new reading from the thermocouple
        data.East = 1;
        data.North = 2;
        data.Down = 3;

        //Update the UAVObject data. The updated values can be viewed in the GCS.
        PositionStateSet(&data);

        // Delay until it is time to read the next sample
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }
}
