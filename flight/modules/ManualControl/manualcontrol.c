/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup ManualControlModule Manual Control Module
 * @brief Provide manual control or allow it alter flight mode.
 * @{
 *
 * Reads in the ManualControlCommand FlightMode setting from receiver then either
 * pass the settings straght to ActuatorDesired object (manual mode) or to
 * AttitudeDesired object (stabilized mode)
 *
 * @file       manualcontrol.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2014.
 * @brief      ManualControl module. Handles safety R/C link and flight mode.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "inc/manualcontrol.h"
#include <sanitycheck.h>
#include <manualcontrolsettings.h>
#include <manualcontrolcommand.h>
#include <vtolselftuningstats.h>
#include <flightmodesettings.h>
#include <flightstatus.h>
#include <systemsettings.h>
#include <stabilizationdesired.h>
#include <callbackinfo.h>
#include <stabilizationsettings.h>

// Private constants
#if defined(PIOS_MANUAL_STACK_SIZE)
#define STACK_SIZE_BYTES                               PIOS_MANUAL_STACK_SIZE
#else
#define STACK_SIZE_BYTES                               1152
#endif

#define CALLBACK_PRIORITY                              CALLBACK_PRIORITY_REGULAR
#define CBTASK_PRIORITY                                CALLBACK_TASK_FLIGHTCONTROL

#define ASSISTEDCONTROL_NEUTRALTHROTTLERANGE_FACTOR    0.2f
#define ASSISTEDCONTROL_BRAKETHRUST_DEADBAND_FACTOR_LO 0.92f
#define ASSISTEDCONTROL_BRAKETHRUST_DEADBAND_FACTOR_HI 1.08f


// defined handlers

static const controlHandler handler_MANUAL = {
    .controlChain      = {
        .Stabilization = false,
        .PathFollower  = false,
        .PathPlanner   = false,
    },
    .handler           = &manualHandler,
};
static const controlHandler handler_STABILIZED = {
    .controlChain      = {
        .Stabilization = true,
        .PathFollower  = false,
        .PathPlanner   = false,
    },
    .handler           = &stabilizedHandler,
};

// Private variables
static DelayedCallbackInfo *callbackHandle;

// Private functions
static void configurationUpdatedCb(UAVObjEvent *ev);
static void commandUpdatedCb(UAVObjEvent *ev);
static void manualControlTask(void);

#define assumptions (assumptions1 && assumptions2 && assumptions3 && assumptions4 && assumptions5 && assumptions6 && assumptions7 && assumptions_flightmode)

/**
 * Module starting
 */
int32_t ManualControlStart()
{
    // Run this initially to make sure the configuration is checked
    configuration_check();

    // Whenever the configuration changes, make sure it is safe to fly
    SystemSettingsConnectCallback(configurationUpdatedCb);
    ManualControlSettingsConnectCallback(configurationUpdatedCb);
    ManualControlCommandConnectCallback(commandUpdatedCb);

    // clear alarms
    AlarmsClear(SYSTEMALARMS_ALARM_MANUALCONTROL);

    // Make sure unarmed on power up
    armHandler(true);

    // Start main task
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);

    return 0;
}

/**
 * Module initialization
 */
int32_t ManualControlInitialize()
{
    /* Check the assumptions about uavobject enum's are correct */
    PIOS_STATIC_ASSERT(assumptions);

    ManualControlCommandInitialize();
    FlightStatusInitialize();
    ManualControlSettingsInitialize();
    FlightModeSettingsInitialize();
    SystemSettingsInitialize();
    StabilizationSettingsInitialize();

    callbackHandle = PIOS_CALLBACKSCHEDULER_Create(&manualControlTask, CALLBACK_PRIORITY, CBTASK_PRIORITY, CALLBACKINFO_RUNNING_MANUALCONTROL, STACK_SIZE_BYTES);

    return 0;
}
MODULE_INITCALL(ManualControlInitialize, ManualControlStart);

/**
 * Module task
 */
static void manualControlTask(void)
{
    // Process Arming
    armHandler(false);

    // Process flight mode
    FlightStatusData flightStatus;

    FlightStatusGet(&flightStatus);
    ManualControlCommandData cmd;
    ManualControlCommandGet(&cmd);

    FlightModeSettingsData modeSettings;
    FlightModeSettingsGet(&modeSettings);

    uint8_t position = cmd.FlightModeSwitchPosition;
    uint8_t newMode  = flightStatus.FlightMode;
    uint8_t newFlightModeAssist      = flightStatus.FlightModeAssist;
    uint8_t newAssistedControlState  = flightStatus.AssistedControlState;
    uint8_t newAssistedThrottleState = flightStatus.AssistedThrottleState;
    if (position < FLIGHTMODESETTINGS_FLIGHTMODEPOSITION_NUMELEM) {
        newMode = modeSettings.FlightModePosition[position];
    }

    // if a mode change occurs we default the assist mode and states here
    // to avoid having to add it to all of the below modes that are
    // otherwise unrelated
    if (newMode != flightStatus.FlightMode) {
        // set assist mode to none to avoid an assisted flight mode position
        // carrying over and impacting a newly selected non-assisted flight mode pos
        newFlightModeAssist      = FLIGHTSTATUS_FLIGHTMODEASSIST_NONE;
        // The following are eqivalent to none effectively. Code should always
        // check the flightmodeassist state.
        newAssistedControlState  = FLIGHTSTATUS_ASSISTEDCONTROLSTATE_PRIMARY;
        newAssistedThrottleState = FLIGHTSTATUS_ASSISTEDTHROTTLESTATE_MANUAL;
    }

    // Depending on the mode update the Stabilization or Actuator objects
    const controlHandler *handler = &handler_MANUAL;
    switch (newMode) {
    case FLIGHTSTATUS_FLIGHTMODE_MANUAL:
        handler = &handler_MANUAL;
        break;
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED1:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED2:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED3:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED4:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED5:
    case FLIGHTSTATUS_FLIGHTMODE_STABILIZED6:
        handler = &handler_STABILIZED;

        break;
        // There is no default, so if a flightmode is forgotten the compiler can throw a warning!
    }

    bool newinit = false;

    // FlightMode needs to be set correctly on first run (otherwise ControlChain is invalid)
    static bool firstRun = true;

    if (flightStatus.FlightMode != newMode || firstRun ||
        newFlightModeAssist != flightStatus.FlightModeAssist ||
        newAssistedControlState != flightStatus.AssistedControlState ||
        flightStatus.AssistedThrottleState != newAssistedThrottleState) {
        firstRun = false;
        flightStatus.ControlChain          = handler->controlChain;
        flightStatus.FlightMode            = newMode;
        flightStatus.FlightModeAssist      = newFlightModeAssist;
        flightStatus.AssistedControlState  = newAssistedControlState;
        flightStatus.AssistedThrottleState = newAssistedThrottleState;
        FlightStatusSet(&flightStatus);
        newinit = true;
    }
    if (handler->handler) {
        handler->handler(newinit);
    }
}

/**
 * Called whenever a critical configuration component changes
 */
static void configurationUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    configuration_check();
}

/**
 * Called whenever a critical configuration component changes
 */
static void commandUpdatedCb(__attribute__((unused)) UAVObjEvent *ev)
{
    PIOS_CALLBACKSCHEDULER_Dispatch(callbackHandle);
}
