# rotorctrl
Rotorcontrol for Arduino Uno/Nano.
For:
. DC motors with end switch and parallel diode
. stepper motors, using accel library, with end detections

One or 2 rotors can be specified. Names are AX and EY: Azimut or X and Elevation or Y

For pinning see rotor_spec.h.

Files:

calibrate.ino	calibration function to run in setup
command.ino	read commands from USB
rotorfuncs.ino	rotor drive related functions
rotorctrl.ino	top-level; contains setup(), loop() and interrupt functions
misc.ino	some misc functions, for display, feedback etc.
pins.ino	definitions of pins
rotorctrl.h	definitions of special types, including rotor records
		Includes rotor_spec.h
rotor_spec.h	contains all info of rotors, Arduino-pinning etc.


NOTE: rotor_spec.h is the only file to customize for rotor specs:
. pinning of Arduino for both rotors
. enable 1 or 2 rotors
. motor type: DC or stepper
. rotor type: 
  . X/Y
  . elevation/azimut:
    . 90/360
    . 180/180 degrees with east/west pass info

. motor run parameters: 
.   PWM frequency
.   min. / max. speed, accelleration (stepper)
.   offset and pulses/steps -to- degrees factors
.   error degrees threshold from where speed wil increase with increasing error
.   error degrees threshold from where speed is max.

Calibration:
. may be done in 2 steps, first fast, second slow and accurate.
. SPD_ENDSW1 and SPD_ENDSW2 are the speeds in percentage of max. speed defined:
.   DC-motor: max. speed defined by hardware/voltage
.   stepper motor: defined by xx_MotorSpeed
. If one of 2 speeds is defined 0 then a single-calibration is done.

For calibration progress detection two 3-colour-led's may be connected, for each rotor one. See rotor_spec.h.
Colour meaning:
- Blue: start calibration, move forward from calibration point
- Purple (blue+red): move back to endswitch
- White (blue+green+red): move forward to reference point
- Green: calibration succeeded
- Red: calibration failed 
