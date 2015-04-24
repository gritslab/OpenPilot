#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include <openpilot.h>
#include <pid.h>

#include <manualcontrolcommand.h>
#include <positionstate.h>
#include <positiondesired.h>
#include <stabilizationdesired.h>

int32_t PositionControlStart(void);
int32_t PositionControlInitialize(void);

void performControl(struct pid *posPid);
void calcAttFromForceVec(float f[3], float yawDesired, StabilizationDesiredData* att);
float forceNorm2thrustPercentage(float f);

#endif // POSITIONCONTROL_H
