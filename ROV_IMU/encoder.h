/*********************************************************************
Encoder Header file

Functions to get encoder data using interrupts
**********************************************************************
Copyright(C)  <December 2015>  <Chandra P. Mangipudi>

This program is free software : you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#ifndef enc_H
#define enc_H
#include "constants.h"

typedef struct
{
	int pinA;
	int pinB;
	int pos;
	int del;
} Encoder;

extern volatile Encoder Enc[3];
extern int drA, drB;

void get_encoder();
void init_encoder();
void debounce(int del);
void isrPinAEn0();
void isrPinBEn0();
void isrPinAEn1();
void isrPinBEn1();
void isrPinAEn2();
void isrPinBEn2();
void print_encoder();

extern int drA, drB;

#endif


