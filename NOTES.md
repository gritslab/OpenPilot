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

## Creating new module

* Add module to "madatory" (i.e. `MODULES += NewModule`) or "optional" (i.e. `OPTMODULES += NewModule`) modules list in the Makefile at

    `OpenPilot/flight/targets/boards/coptercontrol/firmware/Makefile`

* Add the module to all the fields (i.e. `<elementname>NewModule</elementname>`) in the taskinfo.xml at

    `OpenPilot/shared/uavobjectdefinition/taskinfo.xml`

  * This will require a recompile of the uavobjects (i.e. `>> make uavobjects`)

* Create the module folder (i.e. `NewModule`) at

    `OpenPilot/flight/modules'

* At the header and source files to the module folder.

