/*
 * utils.cpp
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */
#include <cstring>
#include <cstdint>
#include "xprintf.h"
#include "configuration.h"


/* Stolen from stackoverflow.com by user411313 Aug. 5 2010
 * I had a hard time using the built in strtok always segfault-ing*/
char *xstrtok(char **m,char *s,char c)
{
  char *p=s?s:*m;
  if( !*p )
    return 0;
  *m=strchr(p,c);
  if( *m )
    *(*m)++=0;
  else
    *m=p+strlen(p);
  return p;
}


/* Check if a character is an alpha from A to Z */
bool isLetter(char c) {
  bool ret = false;
  if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
    ret = true;
  }
  return ret;
}


/* Check if character is a number 0 to 9*/
bool isNumber(char n) {
  bool ret = false;
  if (n >= '0' && n <= '9') {
    ret = true;
  }
  return ret;
}





