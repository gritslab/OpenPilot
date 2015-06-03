Rowland's OpenPilot Notes
=========================

## Links
* [OpenPilot Documention](https://wiki.openpilot.org)

## Get the code

    >> git clone git://git.openpilot.org/OpenPilot.git OpenPilot

## Compiling

All compiling is done from the OpenPilot root folder.

From the beginning...

    >> make arm_sdk_install
    >> make all

Updating flight code...

    >> make fw_coptercontrol_clean
    >> make fw_coptercontrol

Generate flight code for firmware flashing through GCS

    >> make opfw_resource

Flashing firmware

* Disconnect board
* Open OpenPilot GCS GUI
* Click on Firmware button at the bottom
* Click on Rescue button at the top
* Plug in board before time runs out
* Click on Open button
* Open OpenPilot/build/fw_coptercontrol/fw_coptercontrol.opfw
* Click on Flash button
* After flash click Boot button

## Compiling and Running GCS

* To compile GCS

    >> make gcs

* Run

    >> cd OpenPilot/build/openpilotgcs_release/bin
    >> sudo ./openpilotgcs

## Adding new UAVObject

* Create new UAVObject xml definition file in `OpenPilot/shared/uavobjectdefinition`
* Add uavobject to make file at `OpenPilot/flight/targets/boards/coptercontrol/firmware/Makefile`
* Add .h and .cpp file to `OpenPilot/ground/openpilotgcs/src/plugins/uavobjects/uavobjects.pro`
* Recompile GCS

    >> make uavobjects
    >> make gcs


## Creating new module

* Add module to "madatory" (i.e. `MODULES += NewModule`) or "optional" (i.e. `OPTMODULES += NewModule`) modules list in the Makefile at

    `OpenPilot/flight/targets/boards/coptercontrol/firmware/Makefile`

The Telemetry module should be last in the list.

* Add the module to all the fields (i.e. `<elementname>NewModule</elementname>`) in the taskinfo.xml at

    `OpenPilot/shared/uavobjectdefinition/taskinfo.xml`

  * This will require a recompile of the uavobjects (i.e. `>> make uavobjects`)

* If a compile error complains about a UAVObject make sure the object is added to the make file at

    `OpenPilot/flight/targets/boards/coptercontrol/firmware/Makefile

* Create the module folder (i.e. `NewModule`) at

    `OpenPilot/flight/modules'

* At the header and source files to the module folder.

## Creating new UAVObject

* UAVObject definitions are in `OpenPilot/shared/uavobjectdefinition`

## CPU and Memory Usage

* Attitude, Telemetry
    * 31% 64 degrees CPU, 7990 bytes free
* Attitude, Telemetry, FirmwareIAP
    * 31% 64 degrees CPU, 7730 bytes free
* Attitude, Telemetry, FirmwareIAP, Stabilization
    * 49% 64 degrees CPU, 4930 bytes free
* Attitude, Telemetry, FirmwareIAP, Stabilization, Receiver
    * 55% 64 degrees CPU, 3670 bytes free
* Attitude, Telemetry, FirmwareIAP, Stabilization, Receiver, Actuator
    * 55% 64 degrees CPU, 2020 bytes free
* Attitude, Telemetry, FirmwareIAP, Stabilization, Receiver, Actuator, ManualControl
    * 84% CPU, 1410 bytes free
* Attitude, Telemetry, FirmwareIAP, Stabilization, Receiver, Actuator, ManualControl, MsgReceiver (5 ms)
    * 92% CPU, 712 bytes free
* Attitude, Telemetry, FirmwareIAP, Stabilization, Receiver, Actuator, ManualControl, MsgReceiver (5 ms), PositionControl (20 ms)
    * 99% CPU, 12 bytes free

Summary:
Attitude, Telemetry     = 31%
FirmwareIAP             = <1%,  260 bytes
Stabilization           = 18%, 2800 bytes
Receiver                =  6%, 1270 bytes
Actuator                = <1%, 1650 bytes
ManualControl           = 29%,  610 bytes
MsgReceiver (5 ms)      =  8%,  698 bytes
PositionControl (20 ms) =  7%,  700 bytes
