/*******************************************************************
 * RCSId: $Id: command.ino,v 1.1 2021/07/29 08:27:25 ralblas Exp $
 *
 * Project: rotordrive
 * Author: R. Alblas
 *
 * content: 
 *   parser for commands entered via USB
 *
 * public functions:
 *   int readCommand(char ch,GOTO_VAL *gotoval)
 *
 * History: 
 *   
 * $Log: command.ino,v $
 * Revision 1.1  2021/07/29 08:27:25  ralblas
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
/*
 * Project: rotor
 * content: 
 *   readCommand(char ch,const char *frmt,GOTO_VAL *gotoval):
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rotorctrl.h"

/*********************************************************************
 *     get character from serial interface; 
 *     collect requested rotor position from serial interface
 *     commands:
 *       [ew],<ey>,<ax>            east/west flag optional
 *       f<val>                    PWM frequency in Hz
 *       a<val>                    run rotor a (val=-255...255)
 *       b<val>                    run rotor b (val=-255...255)
 *       g                         get rotor info + status
 *     store in:
 *       gotoval->ey                 (elevation or X)
 *       gotoval->ax                 (azimut or Y)
 *       gotoval->east_pass          (info east or west pass)
 *       gotoval->eastwest_pass_info (info if east/westinfo is available
 *  return:  0 if not ready yet
 *           1 if ready
 *          -1 if error
 * Max. length of command: BUFLEN
 ********************************************************************/
#define BUFLEN 20
int readCommand(char ch,GOTO_VAL *gotoval,int *eval)
{
  static char buf[BUFLEN],*p;
  static int pos;
  static int nrcmd;
  char *sv[3];
  float vals[3];
  int ret=0;
  int nrvals=0;

  if ((ch == '\n') || (ch == '\r'))
  { // received command ready
    if ((sv[0]=strtok(buf,",\n\r")))  nrvals++;
    if ((sv[1]=strtok(NULL,",\n\r"))) nrvals++;
    if ((sv[2]=strtok(NULL,",\n\r"))) nrvals++;

    for (int i=0; i<nrvals; i++) if (sv[i]) vals[i]=atof(sv[i]);

    ret=1;
    switch(nrvals)
    {
      case 3: 
        gotoval->ax=vals[1]; 
        gotoval->ey=vals[2]; 
        gotoval->east_pass=(vals[0]? true : false);
        gotoval->eastwest_pass_info=true;
      break;
      case 2:
        gotoval->ax=vals[0]; 
        gotoval->ey=vals[1]; 
        gotoval->east_pass=true;
        gotoval->eastwest_pass_info=false;
      break;
      case 1:
        if (sv[0][0]=='f')
        {
          *eval=atoi(sv[0]+1);
          ret='f';
        }
        else if (sv[0][0]=='a')
        {
          *eval=atoi(sv[0]+1);
          ret='a';
        }
        else if (sv[0][0]=='b')
        {
          *eval=atoi(sv[0]+1);
          ret='b';
        }
        else if (!strncmp(sv[0],"gc",5))
        {
          *eval=0;
          ret='g';
        }
        else if (!strncmp(sv[0],"gs",5))
        {
          *eval=0;
          ret='s';
        }

        else if (sv[0][0]=='g')
        {
          *eval=0;
          ret='g';
        }
        else if (sv[0][0]=='m')
        {
          *eval=atoi(sv[0]+1);
          ret='m';
        }
        else if (!strncmp(sv[0],"calibrate",3))
        {
          *eval=0;
          ret='c';
        }
        else
        {
          ret=-1;
        }
      break;
      default:
        ret=-1;
      break;
    }
    rec2displ(gotoval->east_pass,gotoval->ax,gotoval->ey);
    pos=0;
  }
  else if (ch)
  { // still collecting line
    if (pos < BUFLEN-1)
    {
      buf[pos++]=ch; buf[pos]=0;
    }
    else
    {
      ret=-1;
    }
  }
  return ret;
}

#ifdef TEST
main()
{
  GOTO_VAL gotoval;
  int val;
  int n;
  char *frmt="%f,%f,%f";
  int i;
  char str[100];
  strcpy(str,"5,1.2,3.4\n");
  i=0;
  while ((n=readCommand(str[i],&gotoval,&val)) <=0) i++;
  printf("%d  %f  %f\n",gotoval.east_pass,gotoval.ax,gotoval.ey);
  i=0;
  strcpy(str,"0,7.8,9.0\n");
  while ((n=readCommand(str[i],&gotoval,&val)) <=0) i++;
  printf("%d  %f  %f\n",gotoval.east_pass,gotoval.ax,gotoval.ey);

  i=0;
  strcpy(str,"f1234\n");
  while ((n=readCommand(str[i],&gotoval,&val)) <=0) i++;
  printf("%c: %d\n",n,val);

  i=0;
  strcpy(str,"a244\n");
  while ((n=readCommand(str[i],&gotoval,&val)) <=0) i++;
  printf("%c: %d\n",n,val);

  i=0;
  strcpy(str,"b366\n");
  while ((n=readCommand(str[i],&gotoval,&val)) <=0) i++;
  printf("%c: %d\n",n,val);
}
#endif
