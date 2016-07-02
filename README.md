jUAV Modified Paparazzi
This is a modified version of the Paparazzi UAV project.

The modifications to this code were made to facilitate development of a Java Autopilot that will be
 compatible with the existing Paparazzi tools (Ground Station, NPS Simulator, ect.)

The modifications to this code include:
 1) Modification to the build run in the Paparazzi tool while compiling Airframe specific code.
    These modification create a shared library file which will be found in the Airframes nps folder
    ex. var/aircrafts/Quad_LisaM_2/nps/libpapa.so
 2) Various extra functions allowing for entry from the jUAV code base. These extra functions are
    called over JNI allowing for jUAV to fully function even though it is still in the process of
    being ported.

The current implementation of the jUAV code base has only been tested using the Quad_LisaM_2 airframe.
jUAV can be found at https://github.com/adamczer/pappa

Instructions on how to build and run Paparazzi are seen in the original Paparazzi README seen below:
***********
MAIN README

Paparazzi UAS
=============

[![Build Status](https://travis-ci.org/paparazzi/paparazzi.png?branch=master)](https://travis-ci.org/paparazzi/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)

Paparazzi is an attempt to develop a free software Unmanned (Air) Vehicle System.
 As of today the system is being used successfuly by a number of hobbyists, universities and companies all over the world, on vehicle of various size ( 100g to 25Kg ) and of various nature ( fixed wing, rotorcrafts, boats and surface vehicles).

Up to date information is available in the wiki http://wiki.paparazziuav.org

and from the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi)
and the IRC channel (freenode, #paparazzi).


Required Software
-----------------

Installation is described in the wiki (http://wiki.paparazziuav.org/wiki/Installation).

For Ubuntu users, required packages are available in the [paparazzi-uav PPA] (https://launchpad.net/~paparazzi-uav/+archive/ppa),
Debian users can use the [OpenSUSE Build Service repository] (http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/)

Debian/Ubuntu packages:
- **paparazzi-dev** is the meta-package that depends on everything needed to compile and run the ground segment and the simulator.
- **paparazzi-jsbsim** is needed for using JSBSim as flight dynamic model for the simulator.

Recommended cross compiling toolchain: https://launchpad.net/gcc-arm-embedded


Directories quick and dirty description:
----------------------------------------

_conf_: the configuration directory (airframe, radio, ... descriptions).

_data_: where to put read-only data (e.g. maps, terrain elevation files, icons)

_doc_: documentation (diagrams, manual source files, ...)

_sw_: software (onboard, ground station, simulation, ...)

_var_: products of compilation, cache for the map tiles, ...


Compilation and demo simulation
-------------------------------

1. type "make" in the top directory to compile all the libraries and tools.

2. "./paparazzi" to run the Paparazzi Center

3. Select the "Microjet" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" from
  the upper-right session combo box and click "Execute".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading of the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same than in simulation !
