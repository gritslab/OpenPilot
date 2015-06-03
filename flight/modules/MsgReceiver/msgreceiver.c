#include "inc/msgreceiver.h"

#include <openpilot.h>
#include <taskinfo.h>
#include <pios_com.h>
#include <CoordinateConversions.h>

#include <motioncapture.h>
#include <positionstate.h>
#include <positiondesired.h>
#include <manualcontrolcommand.h>


#define STACK_SIZE_BYTES 380
#define TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define UPDATE_PERIOD_MS    20

static xTaskHandle taskHandle;
static void MsgReceiverTask(void *parameters);

static uint32_t comPortMain;

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

bool parityCheck(uint8_t* msg, uint8_t msgSize)
{
    uint8_t msgByte = 0;
    uint8_t parityByte = msg[PARITY_BYTE_IND];
    for (uint8_t i=PARITY_BYTE_IND+1; i<msgSize; ++i) {
        msgByte ^= msg[i];
    }
    return (msgByte == parityByte);
}

int32_t MsgReceiverInitialize(void)
{
    MotionCaptureInitialize();
    PositionStateInitialize();
    PositionDesiredInitialize();
    ManualControlCommandInitialize();

    // Set port
    comPortMain = PIOS_COM_GPS;
    PIOS_COM_ChangeBaud(comPortMain, 57600);

    return 0;
}

int32_t MsgReceiverStart(void)
{
    xTaskCreate(MsgReceiverTask,
                "MsgReceiver",
                STACK_SIZE_BYTES / 4,
                NULL,
                TASK_PRIORITY,
                &taskHandle);
    PIOS_TASK_MONITOR_RegisterTask(TASKINFO_RUNNING_MSGRECEIVER, taskHandle);
    return 0;
}

MODULE_INITCALL(MsgReceiverInitialize, MsgReceiverStart);

static void MsgReceiverTask(__attribute__((unused)) void *parameters)
{
    // UAVObjects
    MotionCaptureData motion;

    PositionStateData posState;
    PositionStateGet(&posState);

    PositionDesiredData posDesired;
    PositionDesiredGet(&posDesired);

    portTickType readDelay = 10;
    uint8_t BUFFER_SIZE = 32;
    char readBuffer[BUFFER_SIZE];
    uint16_t readSize;

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

        readSize = PIOS_COM_ReceiveBuffer(comPortMain, (uint8_t *)readBuffer, BUFFER_SIZE, readDelay);

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

                                motion.North = msg.pos.x;
                                motion.East = msg.pos.y;
                                motion.Down = msg.pos.z;
                                motion.Yaw = msg.pos.yaw;

                                MotionCaptureSet(&motion);

                            }
                        }
                    } else if (msg.header.type == POSITION_DESIRED) {
                        if (msgByteInd >= sizeof(msg.posDesired)) {
                            msgMarkerFlag = false;
                            if (parityCheck((uint8_t *)&msg, sizeof(msg))) {
                                posDesired.North = msg.posDesired.x;
                                posDesired.East = msg.posDesired.y;
                                posDesired.Down = msg.posDesired.z;

                                ManualControlCommandData cmd;
                                ManualControlCommandGet(&cmd);

                                if (cmd.FlightModeSwitchPosition == 1) {
                                    PositionDesiredSet(&posDesired);
                                }
                            }
                        }
                    } else {
                        msgMarkerFlag = false;
                    }
                }
            }
        }


        // Delay until it is time to read the next sample
        vTaskDelayUntil(&lastWakeTime, UPDATE_PERIOD_MS  / portTICK_RATE_MS);
    }
}