#include "inc/motioncapture.h"

#include <openpilot.h>
#include "taskinfo.h"
#include "positionstate.h"
#include "pathdesired.h"
#include "homelocation.h"

#include <pios_com.h>

#define STACK_SIZE_BYTES 512
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    5

static xTaskHandle taskHandle;
static void motionCaptureTask(void *parameters);

static uint32_t comPortMain;
static uint32_t comPortFlex;

enum MsgType {
    POSITION = 1,
    POSITION_DESIRED = 2
};

// size = 4
struct MsgHeader {
    uint8_t marker;
    uint8_t parity;
    uint8_t target;
    uint8_t type;
};
static uint8_t PARITY_BYTE_IND = 1;

// size 4+4*4 = 20
struct MsgPosition {
    struct MsgHeader header;
    float x;
    float y;
    float z;
    float yaw;
};

// size 4+4*4 = 20
struct MsgPositionDesired {
    struct MsgHeader header;
    float x;
    float y;
    float z;
    float yaw;
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

bool parityCheck(uint8_t* msg, uint8_t msgSize)
{
    uint8_t msgByte = 0;
    uint8_t parityByte = msg[PARITY_BYTE_IND];
    for (uint8_t i=PARITY_BYTE_IND+1; i<msgSize; ++i) {
        msgByte ^= msg[i];
    }
    return (msgByte == parityByte);
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
    PathDesiredInitialize();

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
    PIOS_COM_ChangeBaud(comPortMain, 115200);

    comPortFlex = PIOS_COM_TELEM_RF;
    PIOS_COM_ChangeBaud(comPortFlex, 115200);

    return 0;
}

MODULE_INITCALL(MotionCaptureInitialize, MotiontCaptureStart);

static void motionCaptureTask(__attribute__((unused)) void *parameters)
{
    // UAVObjects
    PositionStateData posState;
    PositionStateGet(&posState);

    PathDesiredData pathDesired;
    PathDesiredGet(&pathDesired);


    portTickType readDelay = 10 / portTICK_RATE_MS;
    uint8_t BUFFER_SIZE = 128;
    char readBuffer[BUFFER_SIZE];
    uint16_t readSize;

    // int loopCnt = 0;
    // char writeBuffer[BUFFER_SIZE];
    // writeBuffer[0] = 'H';
    // writeBuffer[1] = 'E';
    // writeBuffer[2] = 'L';
    // writeBuffer[3] = 'L';
    // writeBuffer[4] = 'O';
    // writeBuffer[5] = '\0';
    // PIOS_COM_SendBufferNonBlocking(comPortFlex, (uint8_t *)writeBuffer, strlen(writeBuffer));

    union Msg {
        struct MsgHeader header;
        struct MsgPosition pos;
        struct MsgPositionDesired posDesired;
    } msg;
    memset(&msg, 0, sizeof(msg));

    bool msgMarkerFlag = false;
    uint8_t msgByteInd = 0;
    uint8_t* msgBytePointer = (uint8_t*)&msg;

    TickType_t lastWakeTime  = xTaskGetTickCount();
    while(1) {
        while ((readSize = PIOS_COM_ReceiveBuffer(comPortMain, (uint8_t *)readBuffer, BUFFER_SIZE, readDelay)) > 0) {
            for (uint8_t i=0; i<readSize; ++i) {
                if (msgMarkerFlag == false) {
                    if (readBuffer[i] == 0xFF) {
                        msgMarkerFlag = true;
                        msgByteInd = 0;
                    }
                }

                if (msgMarkerFlag == true) {
                    msgBytePointer[msgByteInd++] = readBuffer[i];
                    if (msgByteInd >= sizeof(msg.header)) {
                        if (msg.header.type == POSITION) {
                            if (msgByteInd >= sizeof(msg.pos)) {
                                msgMarkerFlag = false;
                                if (parityCheck((uint8_t *)&msg, sizeof(msg))) {
                                    posState.North = msg.pos.x;
                                    posState.East = msg.pos.y;
                                    posState.Down = msg.pos.z;

                                    PositionStateSet(&posState);
                                }
                            }
                        } else if (msg.header.type == POSITION_DESIRED) {
                            if (msgByteInd >= sizeof(msg.posDesired)) {
                                msgMarkerFlag = false;
                                if (parityCheck((uint8_t *)&msg, sizeof(msg))) {
                                    pathDesired.End.North = msg.posDesired.x;
                                    pathDesired.End.East = msg.posDesired.y;
                                    pathDesired.End.Down = msg.posDesired.z;

                                    PathDesiredSet(&pathDesired);
                                }
                            }
                        } else {
                            msgMarkerFlag = false;
                        }
                    }
                }
            }
        }

        // Delay until it is time to read the next sample
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }
}
