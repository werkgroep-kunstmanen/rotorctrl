/*******************************************************************
 * RCSId: $Id: misc.ino,v 1.2 2021/08/03 09:12:38 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   some debug functions
 *
 * History: 
 * $Log: misc.ino,v $
 * Revision 1.2  2021/08/03 09:12:38  ralblas
 * _
 *
 * Revision 1.1  2021/07/29 08:15:44  ralblas
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
#include "rotorctrl.h"
#include <stdarg.h>

// serial print without blocking: only print if enough buffer!
// (Otherwise big delays will occur if sent bits are not received by xtrack)
// hm, if not received at PC: filling up buffer in PC, if full: TX and RX cont. on.
// So if TX blocks also RX blocks??? 
int swrite(char *str)
{
#if USE_MON_UART
  if (!do_feedback) return 0;
  int bufspace=Serial.availableForWrite();
  if (strlen(str) < bufspace-2)
  {
    Serial.println(str);
    return 0;
  }
  return 1;
#else
  return 0;
#endif
}

// print string to serial and/or display
void xprint(int x,int y,char *s)
{
#if USE_MON_UART
  swrite(s);
#endif

#if USE_DISPLAY
  if ((x>=0) && (y>=0))
  {
    lcd.setCursor(x, y);
    lcd.print(s);
  }
#endif
}

#define STRLEN 60

// print format to serial and/or display
void xdispprintf(int x,int y,char *frmt,...)
{
  char str[STRLEN];
  va_list ap;
  va_start(ap,frmt);
  vsnprintf(str,STRLEN,frmt,ap);
  xprint(x,y,str);
}


// print format to serial only
void xprintf(char *frmt,...)
{
  char str[STRLEN];
  va_list ap;
  va_start(ap,frmt);
  vsnprintf(str,STRLEN,frmt,ap);
  xprint(-1,-1,str);
}


// delay between 2 write actions, to prevent buffer overflow
#define SERDEL 10
void send_specs(ROTOR *AX_rot,ROTOR *EY_rot)
{
  char tmp[10];
  xprintf("SPEC: Release %s",RELEASE);                              delay(SERDEL);
  xprintf("SPEC: AX_POffset     =%ld",(long)AX_POffset);            delay(SERDEL);
  xprintf("SPEC: EY_POffset     =%ld",(long)EY_POffset);            delay(SERDEL);

  xprintf("SPEC: AX_REFPOS      =%s",dtostrf(AX_REFPOS,5,1,tmp));   delay(SERDEL);
  xprintf("SPEC: EY_REFPOS      =%s",dtostrf(EY_REFPOS,5,1,tmp));   delay(SERDEL);

  xprintf("SPEC: AX_STEPS_DEGR  =%ld",(long)AX_STEPS_DEGR);         delay(SERDEL);
  xprintf("SPEC: EY_STEPS_DEGR  =%ld",(long)EY_STEPS_DEGR);         delay(SERDEL);

 #if MOTORTYPE == MOT_STEPPER
  xprintf("SPEC: AX_MotorSpeed  =%d",(int)AX_MotorSpeed);           delay(SERDEL);
  xprintf("SPEC: AX_MotorAccel  =%d",(int)AX_MotorAccel);           delay(SERDEL);
  xprintf("SPEC: EY_MotorSpeed  =%d",(int)EY_MotorSpeed);           delay(SERDEL);
  xprintf("SPEC: EY_MotorAccel  =%d",(int)EY_MotorAccel);           delay(SERDEL);
 #else
  xprintf("SPEC: MINSPEED       =%d",(int)MINSPEED);                delay(SERDEL);
  xprintf("SPEC: MAXSPEED       =%d",(int)MAXSPEED);                delay(SERDEL);
  xprintf("SPEC: L_DEGR_MAXSPEED=%d",(int)L_DEGR_MAXSPEED);         delay(SERDEL);
  xprintf("SPEC: H_DEGR_MINSPEED=%d",(int)H_DEGR_MINSPEED);         delay(SERDEL);
  xprintf("SPEC: D_DEGR_STOP    =%s",dtostrf(D_DEGR_STOP,5,1,tmp)); delay(SERDEL);
  xprintf("SPEC: PWMFreq        =%d",(int)PWMFreq);                 delay(SERDEL);
  xprintf("SPEC: MAX_PWM        =%d",(int)MAX_PWM);                 delay(SERDEL);
 #endif
}

void send_stat(ROTOR *AX_rot,ROTOR *EY_rot)
{
  int stat_ax=-1,stat_ey=-1;
  if (AX_rot) stat_ax=AX_rot->cal_status;
  if (EY_rot) stat_ey=EY_rot->cal_status;
  xprintf("STAT: ax=%d  ey=%d",stat_ax,stat_ey);
}

// max. stringlen=23+4*6+2*3=53
void send_pos(ROTOR *AX_rot,ROTOR *EY_rot)
{
  char sdig[4][10];
  float axpos_degr=0.,axreq_degr=0.,eypos_degr=0.,eyreq_degr=0.;
  int ax_speed=0,ey_speed=0;
  int swap=(SWAP_DIR? -1 : 1);
  if (AX_rot) 
  {
    axpos_degr=AX_rot->degr*swap;
    axreq_degr=AX_rot->req_degr;
    ax_speed  =AX_rot->speed;
  }
  if (EY_rot) 
  {
    eypos_degr=EY_rot->degr*swap;
    eyreq_degr=EY_rot->req_degr;
    ey_speed  =EY_rot->speed;
  }
  // floating: -123.4 so min. len buf=6 + 1 (for '0')
  dtostrf(axpos_degr,6, 1, sdig[0]);
  dtostrf(eypos_degr,6, 1, sdig[1]);
  dtostrf(axreq_degr,6, 1, sdig[2]);
  dtostrf(eyreq_degr,6, 1, sdig[3]);
  // max. len=23+4*6+2*3=53
  xprintf("pos=[%s,%s] req=[%s,%s] spd=[%d,%d]",sdig[0],sdig[1],sdig[2],sdig[3],ax_speed,ey_speed);
}

void rec2displ(int ep,float ax,float ey)
{
  static int nr;
  char str1[10],str2[10];

  dtostrf(ax,6, 1, str1);
  dtostrf(ey,6, 1, str2);
  xdispprintf(0,0,"cmd: %d  %6s  %6s",ep,str1,str2);
  xdispprintf(0,1,"nr=%d",++nr);
}

void blink(int n,int d)
{
  for (; n>0; n--)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // LED on; command received
    delay(d);
    digitalWrite(LED_BUILTIN, LOW);    // LED off; command not (yet) received
    delay(d);
  }
}

void set_led(ROTOR *rot,int rgb,boolean enable)
{
  if (!rot) return;
  if (!enable) return;
  if (rot->id==EY_ID)
  {
    if (PIN_Rey>=0) digitalWrite(PIN_Rey , (rgb&4? HIGH : LOW));
    if (PIN_Gey>=0) digitalWrite(PIN_Gey , (rgb&2? HIGH : LOW));
    if (PIN_Bey>=0) digitalWrite(PIN_Bey , (rgb&1? HIGH : LOW));
  }

  if (rot->id==AX_ID)
  {
    if (PIN_Rax>=0) digitalWrite(PIN_Rax , (rgb&4? HIGH : LOW));
    if (PIN_Gax>=0) digitalWrite(PIN_Gax , (rgb&2? HIGH : LOW));
    if (PIN_Bax>=0) digitalWrite(PIN_Bax , (rgb&1? HIGH : LOW));
  }
}

#if MOTOR_SIM
#define PULS_WIDTH 1
#define SIM_AZIMUT_MIN  -20
#define SIM_AZIMUT_MAX  200
void motorpuls(ROTOR *rot)
{
  if (!rot) return;
  if (abs(rot->speed) > 0)
  {
    if (rot->id == AX_ID)
    {
      if (((rot->speed > 0) && (rot->degr < (SIM_AZIMUT_MAX))) ||
          ((rot->speed < 0) && (rot->degr > (SIM_AZIMUT_MIN))))
      {
        digitalWrite(PIN_SIMPLS_AX, HIGH);
        delay(PULS_WIDTH);
        digitalWrite(PIN_SIMPLS_AX, LOW);
      }
    }

    if (rot->id == EY_ID)
    {
      if (((rot->speed > 0) && (rot->degr < 180)) ||
          ((rot->speed < 0) && (rot->degr >   0)))
      {
        digitalWrite(PIN_SIMPLS_EY, HIGH);
        delay(PULS_WIDTH);
        digitalWrite(PIN_SIMPLS_EY, LOW);
      }
    }
  }
}
#endif
