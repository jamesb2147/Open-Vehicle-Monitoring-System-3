=========
Fiat 500e 
=========

Vehicle Type: **FT5E**

This vehicle type supports the 2013-2019 Fiat 500e.

----------------
Support Overview
----------------

=========================== ==============
Function                    Support Status
=========================== ==============
Hardware                    OVMS v3
Vehicle Cable               Works with a Kia Soul OVMS cable
GSM Antenna                 Works with any OVMS v3 supported antenna
GPS Antenna                 Works with any OVMS v3 supported antenna
SOC Display                 Yes
Range Display               Yes
GPS Location                Yes (from GPS module)
Speed Display               tba
Temperature Display         Yes
BMS v+t Display             Yes
TPMS Display                tba
Charge Status Display       Yes
Charge Interruption Alerts  Yes
Charge Control              Yes
Cabin Pre-heat/cool Control Yes (Remote Climate Control, server v2 command 26)
Lock/Unlock Vehicle         Yes
Valet Mode Control          No (see note below)
Others                      12V battery, Homelink 1/2 are used to start and end vehicle alarm
=========================== ==============

-------------------------
Cabin preconditioning
-------------------------

Preconditioning is exposed as **Remote Climate Control** (server v2 command 26),
and its state is reported in ``v.e.hvac``.

Because the framework's scheduled-preconditioning support
(``vehicle``/``climate.precondition``) drives ``CommandClimateControl`` and
gates on ``v.e.hvac``, that feature works on this vehicle as well.

For backward compatibility the **valet mode commands (21/23) remain available as
permanent aliases** for climate control on/off, since preconditioning was
originally exposed that way. Existing app buttons, scripts and home-automation
integrations bound to valet mode continue to work unchanged.

Two consequences worth knowing:

* ``v.e.valet`` is no longer set by preconditioning. Activating preconditioning
  via the valet command will precondition the car, but the app will not show
  valet mode as active. This is deliberate -- reporting preconditioning as valet
  mode caused a "Valet mode enabled" notification on every precondition, and
  armed the hood/trunk intrusion alerts for as long as it ran.
* The valet aliases ignore the PIN argument, as they always have on this
  vehicle. Requiring a PIN now would break existing setups that never configured
  one.

This vehicle does not implement real valet mode.
