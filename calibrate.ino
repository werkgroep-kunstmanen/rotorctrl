/*******************************************************************
 * RCSId: $Id$
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   calibration of one or two motors
 *
 * public functions:
 *   void calibrate(ROTOR *AX_rot, ROTOR *EY_rot)
 *
 * History: 
 *   
 * $Log$
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
#include "rotorctrl.h"
extern ROTOR AX_rot,EY_rot;


/*********************************************************************
 * Calibrate rotors
 *   run for some time both motors from start position
 *   run both motors back to start position = end switch
 *   run both motors 
 * return: 0 if OK, 1 if error
 * AX_rot/EY_rot have calibration status:
 *   0=not done yet
 *   1=start
 *   2=run forward
 *   3=stop forward
 *   4=run backwards to end-stop
 *   5=stop, run to offset  
 *   6=ready, set calibrating=false
 * If calibrating stays true: error; status shows which rotor gives error.
 * AX_rot or EY_rot may be NULL to calibrate a single rotor
 *********************************************************************/

static int run_to_cal_pos(ROTOR *AX_rot, ROTOR *EY_rot)
{
  int err=0;
  // Set pulse count to end-position = pulses needed to go to zero-position
  reset_to_pos(AX_rot,-1*AX_POffset);
  reset_to_pos(EY_rot,-1*EY_POffset);

  // Run to (0.,0.), so run for (AX_POffset,EY_POffset) steps.
  //   This corresponds with pos. (AX_REFPOS,EY_REFPOS)
  run_to_pos(AX_rot, EY_rot,0.,0.,false);
    
  if ((AX_rot) && (AX_rot->cal_status!=cal_ready)) err|=1;
  if ((EY_rot) && (EY_rot->cal_status!=cal_ready)) err|=2;
  run_motor(AX_rot,0);
  run_motor(EY_rot,0);
  if (err) return err;


  // Set pulse count to reference position
  if (AX_rot) AX_rot->degr=AX_REFPOS;
  if (AX_rot) AX_rot->rotated=from_degr(AX_rot);
  if (EY_rot) EY_rot->degr=EY_REFPOS;
  if (EY_rot) EY_rot->rotated=from_degr(EY_rot);
  return 0;
}

void reset_for_cal(ROTOR *rot)
{
  boolean led_ena=true;
  if (!rot) return;
  set_led(rot,1,led_ena);              // set B: start cal.
  rot->calibrated=false;
  rot->cal_status=cal_started;
  reset_to_pos(rot,0);
}

void check_run(ROTOR *rot)
{
  boolean led_ena=true;
  if (!rot) return;
  if (rot->rotated)
  {
    rot->cal_status=cal_got_pulses;    // got pulses
  }
  else
  {
    set_led(rot,4,led_ena);            // set R: not running
    xprint(0,1,(char *)"MES: rotor not running!");
  }
}

int calibrate1(ROTOR *AX_rot,ROTOR *EY_rot,int spd_end1,int spd_end2)
{
  int i;
  int n=0;
  int err=0;
  int speed[2];
  boolean led_ena=true;

  float degr_forward=10.; // must be enough to move rotors from their end-switch!
  #if SWAP_DIR == false
    spd_end1*=-1;
    spd_end2*=-1;
  #endif

  speed[0]=spd_end1;
  speed[1]=spd_end2;

  for (i=0; i<2; i++)
  {
    if (!speed[i]) continue;
    reset_for_cal(AX_rot);
    reset_for_cal(EY_rot);

    //---------- Run both rotors forward, from end-point, for some time.
    xprint(0,1,(char *)"MES: Move from end-switch ");
    run_to_pos(AX_rot, EY_rot,degr_forward,degr_forward,true); // some degrees forward

    // Do some checks: did rotors move?
    check_run(AX_rot);
    check_run(EY_rot);

    delay(1000);

    //---------- Run both rotors backward, until endswitch
    set_led(AX_rot,5,led_ena);               // set RB: to endswitch
    set_led(EY_rot,5,led_ena);               // set RB: to endswitch
    xprint(0,1,(char *)"MES: Move to end-switch");
    run_to_endswitch(AX_rot,EY_rot,speed[i]);

    set_led(AX_rot,7,led_ena);               // set RGB: start going to cal. pos
    set_led(EY_rot,7,led_ena);               // set RGB: start going to cal. pos
    delay(1000);
  }

  //---------- Run both rotors to reference
  xprint(0,1,(char *)"MES: Goto reference      ");
  err=run_to_cal_pos(AX_rot,EY_rot);
  set_led(AX_rot,(err&1? 4 : 2),led_ena);  // set R or G
  set_led(EY_rot,(err&2? 4 : 2),led_ena);  // set R or G
  if (AX_rot) AX_rot->calibrated=(err&1? false : true);
  if (EY_rot) EY_rot->calibrated=(err&2? false : true);

  if (err)
  {
    xprint(0,1,(char *)"MES: Motors don't run!");
    return 3;
  }
  else
  {
    xprint(0,1,(char *)"MES: Calibration ready!");
  }

  return 0;
}

int calibrate(ROTOR *AX_rot,ROTOR *EY_rot)
{
  return calibrate1(AX_rot,EY_rot,SPD_ENDSW1,SPD_ENDSW2);
}
