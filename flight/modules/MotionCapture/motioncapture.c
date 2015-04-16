#include "inc/motioncapture.h"

#include <openpilot.h>
#include "taskinfo.h"
#include "positionstate.h"
#include "homelocation.h"

#include <pios_com.h>

#define STACK_SIZE_BYTES 512
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    5

static xTaskHandle taskHandle;
static void motionCaptureTask(void *parameters);

static uint32_t comPortMain;
static uint32_t comPortFlex;

// size 3*4 = 13 bytes;
struct PosData {
    float x;
    float y;
    float z;
};

union Data {
    struct PosData pos;
    uint8_t c;
};

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

    PIOS_COM_ChangeBaud(comPortMain, 115200);

    return 0;
}

MODULE_INITCALL(MotionCaptureInitialize, MotiontCaptureStart);

static void motionCaptureTask(__attribute__((unused)) void *parameters)
{
    //This data struct is contained in the automatically generated UAVObject code
    PositionStateData posState;

    //Populate the data struct with the UAVObject's current values
    PositionStateGet(&posState);

    portTickType readDelay = 10 / portTICK_RATE_MS;
    uint8_t BUFFER_SIZE = 32;
    char readBuffer[BUFFER_SIZE];
    uint16_t cnt;

    union Data data;
    uint8_t dataSize = sizeof(data);
    memset(&data, 0, dataSize);
    uint8_t posDataInd = 0;
    bool posDataValid = false;
    uint8_t* dataIndPointer;
    dataIndPointer = &data.c;

    char writeBuffer[BUFFER_SIZE];
    // int loopCnt = 0;

    TickType_t lastWakeTime  = xTaskGetTickCount();
    while(1) {
        while ((cnt = PIOS_COM_ReceiveBuffer(comPortMain, (uint8_t *)readBuffer, BUFFER_SIZE, readDelay)) > 0) {
            for (uint8_t i=0; i<cnt; ++i) {
                if (posDataValid) {
                    dataIndPointer[posDataInd++] = readBuffer[i];

                    // intToStr(posDataInd, writeBuffer, 4);
                    // PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));
                    // PIOS_COM_SendCharNonBlocking(comPortFlex, '\t');

                    // intToStr(dataSize, writeBuffer, 4);
                    // PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));
                    // PIOS_COM_SendCharNonBlocking(comPortFlex, '\t');

                    if (posDataInd == dataSize) {


                        posDataInd = 0;
                        posDataValid = false;

                        // memcpy(&pos, readBuffer, dataSize);

                        // Get a new reading
                        posState.North = data.pos.x;
                        posState.East = data.pos.y;
                        posState.Down = data.pos.z;

                        // Update the UAVObject data. The updated values can be viewed in the GCS.
                        PositionStateSet(&posState);

                        // intToStr(9999, writeBuffer, 4);
                        // PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));
                        // PIOS_COM_SendCharNonBlocking(comPortFlex, '\t');
                    }
                }

                if (readBuffer[i] == '$') {
                    posDataInd = 0;
                    posDataValid = true;
                }
            }
            // if (cnt == dataSize && readBuffer[0] == '$') {

            // } else {
            //     // Output
            //     loopCnt++;
            //     if (loopCnt == 100) {
            //     // ftoa(data.North, writeBuffer, 2);
            //     // PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));
            //     // PIOS_COM_SendCharNonBlocking(comPortFlex, '\n');

            //         intToStr(cnt, writeBuffer, 4);
            //         PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));
            //         PIOS_COM_SendCharNonBlocking(comPortFlex, '\t');

            //         loopCnt = 0;
            //     }
            // }
        }

        // Delay until it is time to read the next sample
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }
}
