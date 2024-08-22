Overview
========
The project contains a software implementation of EVSE-SIG-BRD features in EV mode. 
 - Mesure frequency and duty cycle J1772 Control Pilot PWM signal
 - Measure Proximity pilot level
 - Serial communication through host connector
Toolchain supported
===================
- MCUXpresso  11.8.0

Hardware requirements
=====================
- Host controller board e.g., S32G-VNP-RDB2/3 etc.
- Personal Computer

Board settings
==============
* Jumper setting of the board for Power supply 
 - Place J2 to 1,2 and J3 to 1,2 positions if powered by external 5V DC adapter connected to J1
 - Keep default J2 to 2,3 and J3 to 2,3 positions if powered by above mentioned host controller boards NFP connector

Prepare to start
================
Power on the board either by an external 5V DC adapter connected to J1 or by the above mentioned host controller boards MFP connector.

Running the software
====================
When the software runs successfully, the LEDS D18, D19 blink at the rate of 2Hz.

