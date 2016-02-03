/*********************************************************************
YEI IMU Header file

Functions to read YEI IMU for reference
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

#ifndef yei_h
#define yei_h

#include "Arduino.h"
#include "string.h"
#include "Ethernet.h"
#include "EthernetUdp.h"
#include "SPI.h" 
#include "constants.h"

extern float q_raw[4];
void isrIMU();
void IMU_Rd_Channel(union SPIdata *var, int num);
void IMU_Rd_command(byte cmd, int num, boolean byte_type, union SPIdata *var);
void IMU_Rd_Bytes(byte cmd, int num, byte* var);
void IMU_Send_Bytes(byte cmd, int num, byte* data);
void IMU_Send_command(byte cmd, int num, boolean byte_type, union SPIdata *data);
void IMUsetColor(float red, float green, float blue);
void IMUreadColor();
void tare();
void IMUstop_streaming();
void IMUstart_streaming();
void init_YEI();
void update_YEI();

#endif