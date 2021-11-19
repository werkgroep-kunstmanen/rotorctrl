/*******************************************************************
 * RCSId: $Id: rotor_spec.h,v 1.1 2021/07/26 12:34:39 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: header:
 *   def. of:
 *     motor/rotor characteristics
 *     pins
 *     pwm frequency
 *
 * History: 
 * $Log: rotor_spec.h,v $
 * Revision 1.1  2021/07/26 12:34:39  ralblas
 * Initial revision
 *
 *
 *******************************************************************/
/*******************************************************************
 * Copyright (C) 2020 R. Alblas. 
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License 
 * as published by the Free Software Foundation.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
 * 02111-1307, USA.
 ********************************************************************/
// Included by rotorctrl.h
#define RELEASE "2021.10a"

#define ROT_ID "Rot1"

#ifndef ROTORSPEC_HDR
#define ROTORSPEC_HDR

// Enable rotors used
#define ROTOR_AX true
#define ROTOR_EY true

// Set rotortype
#ifndef ROTORTYPE
#define ROTORTYPE ROTORTYPE_AE
#endif

#ifndef MOTORTYPE
#define MOTORTYPE MOT_DC_PWM
#endif


/**************************************************
 * Start definitions; adapt
 **************************************************/
// serial connection
#define SERIAL_SPEED 19200

// Needed interrupt inputs:
// 2	PD2
// 3	PD3

// Allowed PWM outputs:
// pin	name	timer
// 9	PB1	1A
// 10	PB2	1B
// 11	PB3	2A

//==================== Define pins ====================
// If not used: set pinnumber to -1
#define PIN_ROTPLS_AX 2            // PD2: input pulses (interrupt)
#define PIN_ROTDIR_AX 4            // PD4: output direction
#define PIN_ROTDIN_AX 5            // PD5: inverted output direction
#define PIN_LOWSPD_AX -1 // 11     // PD0: output speed (low/high)
#define PIN_ROTPWM_AX 10           // PB2: output speed (pwm)

#define PIN_ROTPLS_EY 3            // PD3: input pulses (interrupt)
#define PIN_ROTDIR_EY 7            // PD7: output direction
#define PIN_ROTDIN_EY 8            // PB0: inverted output direction
#define PIN_LOWSPD_EY -1 // 12     // PD1: output speed (low/high)
#define PIN_ROTPWM_EY 9            // PB1: output speed (pwm)

#define PIN_AXEYEnable -1 // 6     // Enable rotors
#define PIN_EYEndsw1   -1 // 5     // EY Rotor end stops
#define PIN_EYEndsw2   -1 // 4
#define PIN_AXEndsw1   -1 // 3     // AX Rotor end stops
#define PIN_AXEndsw2   -1 // 2

//==================== Extra pins for LED indication ====================
#define PIN_Rax 14                 // PC0: output red 1
#define PIN_Gax 15                 // PC1: output green 1
#define PIN_Bax 16                 // PC2: output blue 1
#define PIN_Rey 17                 // PC3: output red 2
#define PIN_Gey 18                 // PC4: output green 2
#define PIN_Bey 19                 // PC5: output blue 2

//==================== Define rest ====================

#if MOTOR_SIM
  // Simulation: can't use lowspeed, reconnect to GND if lowspeed used! 
  #undef PIN_LOWSPD_AX
  #define PIN_LOWSPD_AX -1
  #undef PIN_LOWSPD_EY
  #define PIN_LOWSPD_EY -1

  #define PIN_SIMPLS_AX 11         // PB3: for motorsim: generated pulse
  #define PIN_SIMPLS_EY 12         // PB4: for motorsim: generated pulse 
#endif

// Rotor characteristics
// _POffset: steps needed to go from end-stop to 0 degrees. 
//   > 0: end-stop is at -x degrees
//   < 0: end-stop is at +x degrees
//
// _REFPOS:  position where calibration ends. (E.g. for elevation: 90 = zenit)
#define AX_POffset 10              // Nr. pulses from End Switch To ref. pos = 0
#define EY_POffset 10              // Nr. pulses from End Switch To ref. pos = 0

#define AX_REFPOS 0.               // Reference position (degrees)
#define EY_REFPOS 0.               // Reference position (degrees)

#define AX_STEPS_DEGR 3L*360L      // Nr. pulses per 360 degrees
#define EY_STEPS_DEGR 3L*360L      // Nr. pulses per 360 degrees

// Speeds
#if MOTORTYPE == MOT_STEPPER       // 
  #define AX_MotorSpeed 100        // max. motorspeed
  #define AX_MotorAccel 50         // max. acceleration
  #define EY_MotorSpeed 100        // max. motorspeed
  #define EY_MotorAccel 50         // max. acceleration
#endif

// Names
#if ROTORTYPE==ROTORTYPE_XY        // X/Y
  #define AX_NAME "X"              // name of rotor 2
  #define EY_NAME "Y"              // name of rotor 1
#elif ROTORTYPE==ROTORTYPE_AE
  #define AX_NAME "azi"            // name of rotor 2
  #define EY_NAME "ele"            // name of rotor 1

  #define USE_EASTWEST true        // use east/west pass info (set false if X/Y rotor)
  #define FULLRANGE_AZIM false     // azimut rotor has limited range 0...180

  #if FULLRANGE_AZIM && USE_EASTWEST
    #error Errored configuration: FULLRANGE_AZIM and USE_EASTWEST!
  #endif

  // Azimut rotor has range of ROT_AZIM_MIN...ROT_AZIM_MAX (via 0)
  // Area ROT_AZIM_MAX...ROT_AZIM_MIN for azimut is forbidden.
  // East pass: azimut= -20...  0...200                    , elev =   0...90
  // West pass: azimut= 160...360...380 ==> -20...180...200, elev = 180...90
  #define SET_AZIM_MIN 340         // min. degrees from tracker   (ROT_AZIM_MIN+360)
  #define SET_AZIM_MAX 200         // max. degrees from tracker

#endif

// Define motor speed settings
#if MOTORTYPE == MOT_DC_FIX        // low speed done in hardware
#define SPEED_LOW 20               // below this speed PIN_LOWSPD_.. becomes active 
#endif

#if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX))
  #define MINSPEED 50                // min. speed (% of max. voltage)
  #define MAXSPEED 100               // max. speed (% of max. voltage)
  #define L_DEGR_MAXSPEED 10.        // >= diff-degrees where rotorspeed is max.
  #define H_DEGR_MINSPEED 2.         // <= diff-degrees where rotorspeed is min.
  #define D_DEGR_STOP 0.2            // <= diff-degrees to stop rotor
#endif

#define SWAP_DIR false             // flip directions

// Calibration motor speed:
//   if one is 0, single calibration
//   Other: 2-step calibration, one fast, one slow for accurate calibration
#define SPD_ENDSW1 100             // speed1: % of **_MotorSpeed, cal. to endswitch
#define SPD_ENDSW2 0               // speed2: same, for second cal. (0: single-calibration)

#if MOTORTYPE == MOT_STEPPER       // stepper motor
  #include <AccelStepper.h>        // http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
  AccelStepper stepperAX(1, PIN_ROTPWM_AX, PIN_ROTDIR_AX);
  AccelStepper stepperEY(1, PIN_ROTPWM_EY, PIN_ROTDIR_EY);

  #define CMD(n,r) ((AccelStepper *)(n.stepper))->r
  #define CMDP(n,r) ((AccelStepper *)(n->stepper))->r
#endif

#if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX)) // DC motor
  #include <PWM.h>
// PWM frequency and max. PWM (some controllers MUST have pulses, so not 100%!)
  #define PWMFreq 10000              // PWM frequency
  #define MAX_PWM 255                // 255=DC 5V
#endif

/**************************************************
 * End definitions
 **************************************************/
#endif
