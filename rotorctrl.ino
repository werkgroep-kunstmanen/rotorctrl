 /*******************************************************************
 * RCSId: $Id: rotorctrl.ino,v 1.1 2021/07/29 08:18:30 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   main program, with setup() and loop()
 *
 * functions:
 *   void setup(void)
 *   void loop(void)
 *
 * History: 
 *   
 * $Log: rotorctrl.ino,v $
 * Revision 1.1  2021/07/29 08:18:30  ralblas
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
#include <stdio.h>
#include <string.h>
#include "rotorctrl.h"

#if USE_DISPLAY
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

ROTOR gAX_rot,gEY_rot;
ROTOR *SAX_rot,*SEY_rot;
boolean do_feedback;
static GOTO_VAL gotoval;

// Pulse counter
static void pos_handler(ROTOR *rot)
{
  if (!rot) return;
  if (rot->dir)
  {
    rot->rotated++;
  }
  else 
  {
    rot->rotated--;
  }
}

// Pulse counter rotor Azimut/X
static void AX_pos_handler(void)
{
  pos_handler(&gAX_rot);
}

// Pulse counter rotor elevation/Y
static void EY_pos_handler(void)
{ 
  pos_handler(&gEY_rot);
}

// setup and calibrate
static void setup_ax(ROTOR *rot)
{
  if (!rot) return;
  strcpy(rot->name,AX_NAME);
  rot->id=AX_ID;
  rot->round=0;
  rot->rotated=0;
  rot->cal_status=cal_notdone;
  AX_set_pins(rot);
  rot->steps_degr=AX_STEPS_DEGR;
  #if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX)) 
    rot->minspeed=AX_MINSPEED;
    rot->maxspeed=AX_MAXSPEED;
  #endif
  #if MOTORTYPE == MOT_STEPPER
    rot->stepper=&stepperAX;
    CMDP(rot,setMaxSpeed(AX_MotorSpeed));     // Set the X rotor-motor maximum speed
    CMDP(rot,setAcceleration(AX_MotorAccel)); // Set the X rotor-motor acceleration speed
  #endif
}

static void setup_ey(ROTOR *rot)
{
  if (!rot) return;
  strcpy(rot->name,EY_NAME);
  rot->id=EY_ID;
  rot->round=0;
  rot->rotated=0;
  rot->cal_status=cal_notdone;
  EY_set_pins(rot);
  rot->steps_degr=EY_STEPS_DEGR;
  #if ((MOTORTYPE == MOT_DC_PWM) || (MOTORTYPE == MOT_DC_FIX)) 
    rot->minspeed=EY_MINSPEED;
    rot->maxspeed=EY_MAXSPEED;
  #endif
  #if MOTORTYPE == MOT_STEPPER
    rot->stepper=&stepperEY;
    CMDP(rot,setMaxSpeed(EY_MotorSpeed));     // Set the X rotor-motor maximum speed
    CMDP(rot,setAcceleration(EY_MotorAccel)); // Set the Y rotor-motor acceleration speed
  #endif
}

void setup(void)
{
  int err=0;
  SAX_rot=NULL;
  SEY_rot=NULL;

  #if ROTOR_AX
    SAX_rot=&gAX_rot;
  #endif
  #if ROTOR_EY
    SEY_rot=&gEY_rot;
  #endif

  setup_ax(SAX_rot);
  setup_ey(SEY_rot);

  #if MOTORTYPE == MOT_STEPPER
    #ifdef PIN_AXEYEnable
      if (PIN_AXEYEnable >=0)
        digitalWrite(PIN_AXEYEnable, HIGH);             // Enable use of M415C controllers
    #endif
  #endif

  pinMode(LED_BUILTIN, OUTPUT);     // calibration indication
  do_feedback=false;

  #if MOTOR_SIM
    pinMode(PIN_SIMPLS_AX , OUTPUT);   // 
    pinMode(PIN_SIMPLS_EY , OUTPUT);   // 
  #endif
  // define interrupts for backpulsecounters
  #if PIN_ROTPLS_AX && PIN_ROTPLS_AX >=0
    attachInterrupt(digitalPinToInterrupt(PIN_ROTPLS_AX) , AX_pos_handler , RISING);
  #endif
  #if PIN_ROTPLS_EY && PIN_ROTPLS_EY >=0
    attachInterrupt(digitalPinToInterrupt(PIN_ROTPLS_EY) , EY_pos_handler , RISING);
  #endif

  #if USE_DISPLAY
    // define LCD display
    lcd.begin(20,4);
    lcd.clear();
  #endif

  // define serial input
  Serial.begin(SERIAL_SPEED);       // Start Serial Communication Interface
  Serial.println(ROT_ID);

  digitalWrite(LED_BUILTIN, LOW);   // LED off; start calibration
  delay(1000);

  err=calibrate(SAX_rot,SEY_rot);
  if (SAX_rot) gotoval.ax=SAX_rot->degr;
  if (SEY_rot) gotoval.ey=SEY_rot->degr;

  if (err)
  {
    run_motor(SAX_rot,0); // stop motors (just in case, should already be stopped)
    run_motor(SEY_rot,0);
    xprint(0,0,(char *)"Calibration error!");
    blink(20,100);        // Note: causes pin 13 to pulse!
    return;
  }

  xprint(0,0,(char *)"Calibration done");
  digitalWrite(LED_BUILTIN, HIGH);   // LED on; calibration done
}

// endless loop: catch position from serial interface and run motors
void loop(void)
{
  static boolean contrun,nieuw;
  int ok=0;
  int busy;
  int eval;

  if (Serial.available())
  {
    ok=readCommand(Serial.read(),&gotoval,&eval); // If There's serial data go read and check for valid data

    #if defined(USE_EASTWEST) && USE_EASTWEST
      if ((ok==1) && ((gotoval.eastwest_pass_info))) convert_eastwest(&gotoval);
    #endif
    if (ok=='f')         // set frequency (testing only, normally never changes)
    {
    #if MOTORTYPE == MOT_DC_PWM       // stepper motor
      SetPinFrequencySafe(PIN_ROTPWM_AX,eval);
      SetPinFrequencySafe(PIN_ROTPWM_EY,eval);
      xprint(0,0,(char *)"Freq set");
    #endif
    }
    else if (ok=='a')    // run motor a continuous (testing only)
    {
      contrun=true;
      run_motor(SAX_rot,eval);
      xprint(0,0,(char *)"MES: Start rotor A/X");

    }
    else if (ok=='b')    // run motor b continuous (testing only)
    {
      contrun=true;
      run_motor(SEY_rot,eval);
      xprint(0,0,(char *)"MES: Start rotor E/Y");
    }
    else if (ok=='g')    
    {
      send_specs(SAX_rot,SEY_rot);
    }
    else if (ok=='s')    
    {
      send_stat(SAX_rot,SEY_rot);
    }
    else if (ok=='c')  
    {
      if (calibrate(SAX_rot,SEY_rot))
      {
        xprint(0,0,(char *)"Calibration error!");
      }
    }  
    else if (ok=='m')  
    {
      do_feedback=(eval? true : false);
    }
    else if (ok==1)      // get positions
    {

      if (contrun)
      {
        xprint(0,0,(char *)"stop contrun");
        run_motor(SAX_rot,0);
        run_motor(SEY_rot,0);
      }
      contrun=false;
    }
    if (ok>0) nieuw=true;

  }
  else if (!contrun)
  {
    busy=rotor_goto(SAX_rot,gotoval.ax);
    busy=busy | rotor_goto(SEY_rot,gotoval.ey);
    if (nieuw) send_pos(SAX_rot,SEY_rot);
    nieuw=false;
  }
  #if MOTORTYPE == MOT_STEPPER
  else
  {
    run_safe(SAX_rot,true);
    run_safe(SEY_rot,true);
  }
  #endif
}
