#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include <openpilot.h>
#include <pid.h>

#include <manualcontrolcommand.h>
#include <positionstate.h>
#include <positiondesired.h>
#include <stabilizationdesired.h>
#include <attitudestate.h>

int32_t PositionControlStart(void);
int32_t PositionControlInitialize(void);

void performControl(struct pid *posPid,
               PositionStateData* pos,
               PositionDesiredData* posDesired,
               float* up_thrust,
               StabilizationDesiredData* att);
float forceNorm2thrustPercentage(float f);

#endif // POSITIONCONTROL_H
