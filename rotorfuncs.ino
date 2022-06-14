/*******************************************************************
 * RCSId: $Id: rotorfuncs.ino,v 1.2 2021/08/03 09:14:34 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   several motor/rotor functions
 *
 * public functions:
 *   float to_degr(ROTOR *rot)
 *   long from_degr(ROTOR *rot)
 *   void convert_eastwest(GOTO_VAL *gv)
 *   int run_safe(ROTOR *rot,boolean cont)
 *   void run_motor(ROTOR *rot,int speed)
 *   int rotor_goto(ROTOR *rot,float val)
 *   void reset_to_pos(ROTOR *rot,long pos)
 *   void run_to_pos(ROTOR *AX_rot, ROTOR *EY_rot,float ax_pos,float ey_pos,boolean relative)
 *   void run_to_endswitch(ROTOR *AX_rot, ROTOR *EY_rot,int speed)
 *
 * History: 
 * $Log: rotorfuncs.ino,v $
 * Revision 1.1  2021/07/29 08:25:38  ralblas
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
#include <stdlib.h>
#include <stdio.h>
#include "rotorctrl.h"

static float step2degr(ROTOR *rot,long step)
{
  float degr;
  degr=(float)step*360./(float)rot->steps_degr;
  return degr;
}

float to_degr(ROTOR *rot)
{
  if (!rot) return 0.;
  return step2degr(rot,rot->rotated);
}

static long degr2step(ROTOR *rot,float degr)
{
  long step=(long)((degr*rot->steps_degr)/360);
  #if SWAP_DIR
  step*=-1;
  #endif
  return step;
}

long from_degr(ROTOR *rot)
{
  if (!rot) return 0;
  return degr2step(rot,rot->degr);
}

#if defined(USE_EASTWEST) && USE_EASTWEST
/*********************************************************************
 * For elevation/azimut, where azimut rotor cannot rotate 360 (400) degrees:
 *   use elevation rotor having 0...180 range; 
 *   determine flip-of-elevation rotor
 *
 * Input azimut: 0=north, 90=east, 180=south, 270=west
 * Input elevation 0...90
 * east pass:   0...180, azimut ranges between 340 (N-bound) and 200 (S-bound)
 * west pass: 180...360, azimut ranges between 160 (N-bound) and  20 (S-bound)
 * 
 *********************************************************************/
void convert_eastwest(GOTO_VAL *gv)
{
  // force to range 0...360
  if (!gv->east_pass)
  {                           // it's a west-pass
    gv->ax=gv->ax-180.;       // and substract 180 of 'goto' azimuth
    gv->ey=180.-gv->ey;       // so flip 'goto' elevation
  }
  if (gv->ax > 360.) gv->ax-=360.;
  if (gv->ax <   0.) gv->ax+=360.;

  // Check if azimut rotor is out-of-range, shouldn't happen with correct east/west pass info
  if ((gv->ax > SET_AZIM_MAX) && (gv->ax < SET_AZIM_MIN))
  {                            // correct
    gv->ax=gv->ax-180.;        // azimuth
    gv->ey=180.-gv->ey;        // elevation
  }


  // force to range 0...360
  if (gv->ax > 360.) gv->ax-=360.;
  if (gv->ax <   0.) gv->ax+=360.;
}
#endif

#if MOTORTYPE == MOT_STEPPER
static void moveto(ROTOR *rot)
{
  long step;
  if (!rot) return;

  step=degr2step(rot,rot->req_degr);
  CMDP(rot,moveTo(step)); // do requested
}

// run if not at end-stop with speed in wrong drection
int run_safe(ROTOR *rot,boolean cont)
{
  float aspeed;
  int pspeed;
  int running=1;
  int end1=0,end2=0;
  if (!rot) return 0;

  aspeed=CMDP(rot,speed());                               // actual speed
  aspeed=(aspeed*100.) / CMDP(rot,maxSpeed());            // speed in %
  pspeed=(int)aspeed;                                     // now to integer

  if ((aspeed) && (!pspeed)) pspeed=(aspeed<0? -1 : +1); // set to 1% if 0<s<1
  if (pspeed>100) pspeed=100;
  if (pspeed<-100) pspeed=-100;
  rot->speed=pspeed;

  end1=digitalRead(rot->pin_end1);
  end2=digitalRead(rot->pin_end2);

  if (((end1) && (pspeed>0)) || ((end2) && (pspeed<0)))
  {
    CMDP(rot,setSpeed(0));          // force internally saved speed to 0
    CMDP(rot,stop());               // force stop
    running=0;                      //
  }
  else
  {
    if (cont)
      CMDP(rot,runSpeed());
    else
      CMDP(rot,run());
    rot->rotated=CMDP(rot,currentPosition());
    if (!pspeed) running=0;
  }
  return running;
}

#else

/*********************************************************************
 * define speed from degrees between current and target position
 * input: deg = difference current and requested angle in degrees
 * return: speed in percents (100=max. speed)
 *********************************************************************/
static int rotor_speed(ROTOR *rot,float deg)
{
  int speed;
  float adg=abs(deg);

  if (adg<=D_DEGR_STOP)
  {
    speed=0;
  }
  else
  {
    float tmp;
    tmp=(float)(adg-H_DEGR_MINSPEED)*(MAXSPEED-MINSPEED)/(float)(L_DEGR_MAXSPEED-H_DEGR_MINSPEED);
    tmp=tmp + MINSPEED;
    speed=(int)tmp;
    speed=MIN(speed,MAXSPEED);
    speed=MAX(speed,MINSPEED);

    if (deg<0) speed*=-1;
    #if SWAP_DIR
      speed*=-1;
    #endif
  }
  #ifdef SLOW_START
  if (rot)
  {
    if (abs(speed) > abs(rot->speed))
    {
      speed=(rot->speed+speed)/2;
    }
  }
  #endif

  return speed;
}

static void set_speed(ROTOR *rot,int speed)
{
  if (speed>100) speed=100;
  #if MOTORTYPE == MOT_DC_PWM
  {
    rot->pwm=(MAX_PWM*speed)/100;
    pwmWrite(rot->pin_pwm,rot->pwm);
  }
  #elif MOTORTYPE==MOT_DC_FIX
  {
    rot->pwm=MAX_PWM;
    if (speed < SPEED_LOW)
    {
      digitalWrite(rot->pin_lsp, HIGH);
    }
    else
    {
      digitalWrite(rot->pin_lsp, LOW);
    }
    pwmWrite(rot->pin_pwm,rot->pwm);
  }
  #endif
}

static void set_dir(ROTOR *rot,boolean dir)
{
  boolean din;
  if (dir!=rot->dir)
  {
    set_speed(rot,0);
    delay(10);
  }
  rot->dir=dir;

  digitalWrite(rot->pin_dir, rot->dir);
  if (rot->pin_din>=0)
  {
    digitalWrite(rot->pin_din, (rot->dir? LOW : HIGH));
  }
}

#endif



/*********************************************************************
 * Start (change speed)  motor with speed 'speed' 
 * speed: 100=max, 0=stop, negative=opposite direction
 *********************************************************************/
void run_motor(ROTOR *rot,int speed)
{
  boolean din;
  float nspeed;
  if (!rot) return;

  #if MOTOR_SIM
    motorpuls(rot);
  #endif

// set speed and run
#if MOTORTYPE == MOT_STEPPER
  nspeed=CMDP(rot,maxSpeed())*(float)speed/100.;
  CMDP(rot,setSpeed(nspeed));
  CMDP(rot,runSpeed());
#else
  if (speed)
    set_dir(rot,speed >= 0? HIGH : LOW);
  set_speed(rot,abs(speed));
#endif
  if ((rot->calibrated) && (speed!=rot->speed))
  {
    if (speed) set_led(rot,6,false); else set_led(rot,2,false);
  }

  rot->speed = speed;
}

/*********************************************************************
 * Rotor to angle 'val'.
 * Must be used in loop; each time this func. is executed 
 *   speed is determined and used.
 * return: current speed; 0=stop=rotator is at requested position.
 *********************************************************************/
int rotor_goto(ROTOR *rot,float val)
{
  float rot_degr;      // current pos. rotor in decdegrees
  float req_degr;      // requested pos. rotor in decdegrees
  float diff_degr;
  int speed;
  if (!rot) return 0;

  #if ROTORTYPE==ROTORTYPE_AE
    #if FULLRANGE_AZIM == false
      if (val>270) val-=360;
    #endif
  #endif

  rot->req_degr=val;                      // requested degrees
  rot->degr=to_degr(rot);

  req_degr=rot->req_degr;
  rot_degr=rot->degr;                  // actual degrees

  #if MOTORTYPE == MOT_STEPPER
    moveto(rot);
    diff_degr=req_degr - rot_degr;
    rot->err_degr=diff_degr;

    if (!run_safe(rot,false)) return 0;         // Do it
    speed=CMDP(rot,distanceToGo());       // to detect if ready, not actual speed...

  #else
    #if ROTORTYPE==ROTORTYPE_AE
      #if FULLRANGE_AZIM
        if ((rot_degr > 270) && (req_degr+rot->round*360 < 90)) rot->round++;
        if ((req_degr > 270) && (rot_degr+rot->round*360 < 90)) rot->round--;
        req_degr+=rot->round*360;
      #endif
    #endif
    diff_degr=req_degr - rot_degr;
    rot->err_degr=diff_degr;

    speed=rotor_speed(NULL,diff_degr);
    run_motor(rot,speed);
  #endif

  return speed;
}


/*********************************************************************
 * calibration funcs
 *********************************************************************/

void reset_to_pos(ROTOR *rot,long pos)
{
  if (!rot) return;
  #if SWAP_DIR
  pos*=-1;
  #endif
  rot->rotated=pos;
  #if MOTORTYPE == MOT_STEPPER
    CMDP(rot,setCurrentPosition(pos));
  #endif
}

static void set_status(ROTOR *rot,CAL_STATUS status)
{
  if (!rot) return;
  if (status==rot->cal_status+1) rot->cal_status=status;
}

#define THR_ERRORDETECT 30000

static int run_one_rotor(ROTOR *rot,float pos,long *run_err,float *err)
{
  int busy;
  if (!rot) return 0;
  busy=rotor_goto(rot,pos);

#if MOTORTYPE == MOT_STEPPER
  rot->rotated=CMDP(rot,currentPosition());
#else
  if (rot->err_degr==*err) (*run_err)++; else (*run_err)=0;
  *err=rot->err_degr;
#endif
  return busy;
}


// blocks until pos. reached, or no pulses reached for some time (=error)
// AX_rot=NULL: only use EY_rot; EY_rot=NULL: only use AX_rot
void run_to_pos(ROTOR *AX_rot, ROTOR *EY_rot,float ax_pos,float ey_pos,boolean relative)
{
  int xbusy=0;
  int ybusy=0;
  long ax_run_err,ey_run_err;
  float ax_err,ey_err;
  ax_run_err=0; ey_run_err=0;
  ax_err=0; ey_err=0;

  if (relative)
  {
    ax_pos+=to_degr(AX_rot);
    ey_pos+=to_degr(EY_rot);
  }
  do
  {
    xbusy=run_one_rotor(AX_rot,ax_pos,&ax_run_err,&ax_err);
    ybusy=run_one_rotor(EY_rot,ey_pos,&ey_run_err,&ey_err);
    if (ax_run_err > THR_ERRORDETECT) break;
    if (ey_run_err > THR_ERRORDETECT) break;

  } while ((xbusy) || (ybusy));
  run_motor(AX_rot,0);
  run_motor(EY_rot,0);
  if (!xbusy) set_status(AX_rot,cal_ready);
  if (!ybusy) set_status(EY_rot,cal_ready);
}


#if MOTORTYPE == MOT_STEPPER
#define RUN_ENDSW_MAX -365.
void run_to_endswitch(ROTOR *AX_rot, ROTOR *EY_rot,int speed)
{
  int busy;
  long n=0;
  float ax_maxspeed=0.,ey_maxspeed=0.;
  float new_speed;

  if (AX_rot)
  {
    ax_maxspeed=CMDP(AX_rot,maxSpeed());              // save current maxspeed
    new_speed=ax_maxspeed*(float)speed/100.;
    CMDP(AX_rot,setMaxSpeed(new_speed));              // set speed to new according to 'speed'

    CMDP(AX_rot,setCurrentPosition(0));               // reset current position
    AX_rot->req_degr=RUN_ENDSW_MAX;                   // max. angle to rotate before give-up
    AX_rot->at_end1=false;
    AX_rot->at_end2=false;
  }
  if (EY_rot)
  {
    ey_maxspeed=CMDP(EY_rot,maxSpeed());              // save current maxspeed
    new_speed=(long)ey_maxspeed*(float)speed/100.;
    CMDP(EY_rot,setMaxSpeed(new_speed));              // set speed to new according to 'speed'

    CMDP(EY_rot,setCurrentPosition(0));               // reset current position
    EY_rot->req_degr=RUN_ENDSW_MAX;                   // max. angle to rotate before give-up
    EY_rot->at_end1=false;
    EY_rot->at_end2=false;
  }
  // Now move rotors to their endswitches
  do
  {
    moveto(AX_rot);
    moveto(EY_rot);

    busy=run_safe(EY_rot,false);                      // run if not at ensswitch
    busy|=run_safe(AX_rot,false);
    n++;
  } while (busy);

  // both rotors at their endswitch, stop
  run_motor(AX_rot,0);
  run_motor(EY_rot,0);

  // Restore org. speed; detect if rotor did move anyway
  if (AX_rot)
  {
    CMDP(AX_rot,setMaxSpeed(ax_maxspeed));

    if (abs(CMDP(AX_rot,currentPosition())) >= abs(degr2step(AX_rot,RUN_ENDSW_MAX))-2) AX_rot->cal_status=cal_got_pulses;
    else  set_status(AX_rot,cal_end_stop);
  }
  if (EY_rot)
  {
    CMDP(EY_rot,setMaxSpeed(ey_maxspeed));

    if (abs(CMDP(EY_rot,currentPosition())) >= abs(degr2step(EY_rot,RUN_ENDSW_MAX))-2) EY_rot->cal_status=cal_got_pulses;
    else set_status(EY_rot,cal_end_stop);
  }
}


#else
// DC/PWM motor

#define pulse_update_rate 500
void run_to_endswitch(ROTOR *AX_rot, ROTOR *EY_rot,int speed)
{
  int AX_pre_cnt,EY_pre_cnt;
  int i;

  if (AX_rot) AX_rot->rotated=0;
  if (EY_rot) EY_rot->rotated=0;

  run_motor(AX_rot,speed);
  run_motor(EY_rot,speed);

  // Oeps, sudden stop at endswitch...
  // wait until both rotors are at their end-switch (no pulses during pulse_update_rate msecs)
  do
  {
    if (AX_rot) AX_pre_cnt=AX_rot->rotated;
    if (EY_rot) EY_pre_cnt=EY_rot->rotated;
    delay(pulse_update_rate);
  } while (((AX_rot) && (AX_pre_cnt!=AX_rot->rotated)) || ((EY_rot) && (EY_pre_cnt!=EY_rot->rotated)));

  if (AX_rot) if (AX_rot->rotated) AX_rot->cal_status=cal_got_pulses;
  if (EY_rot) if (EY_rot->rotated) EY_rot->cal_status=cal_got_pulses;

  if (AX_rot) if (AX_pre_cnt==AX_rot->rotated) set_status(AX_rot,cal_end_stop);
  if (EY_rot) if (EY_pre_cnt==EY_rot->rotated) set_status(EY_rot,cal_end_stop);
}


#endif
