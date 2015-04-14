#include "inc/motioncapture.h"

#include <openpilot.h>
#include "taskinfo.h"
#include "positionstate.h"

#define STACK_SIZE_BYTES 1024
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD    50

static xTaskHandle taskHandle;
static void motionCaptureTask(void *parameters);

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
    return 0;
}

MODULE_INITCALL(MotionCaptureInitialize, MotiontCaptureStart);

static void motionCaptureTask(__attribute__((unused)) void *parameters)
{
    static portTickType lastSysTime;

    lastSysTime = xTaskGetTickCount();
    //This data struct is contained in the automatically generated UAVObject code
    PositionStateData data;

    //Populate the data struct with the UAVObject's current values
    PositionStateGet(&data);

    while(1) {
        //Get a new reading from the thermocouple
        data.East = 1;
        data.North = 2;
        data.Down = 3;

        //Update the UAVObject data. The updated values can be viewed in the GCS.
        PositionStateSet(&data);

        // Delay until it is time to read the next sample
        vTaskDelayUntil(&lastSysTime, UPDATE_PERIOD / portTICK_RATE_MS);
    }
}
