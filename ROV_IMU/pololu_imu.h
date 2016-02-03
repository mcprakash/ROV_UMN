/*********************************************************************
Pololu IMU Header file

Functions to read and calibrate IMU
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

#ifndef pololu_imu_h
#define pololu_imu_h

#include <L3G.h>
#include <LSM303.h>
#include <Wire.h>

extern LSM303 acc_mag;
extern L3G gyro;
extern char ch;

void init_pololu_imu();
void get_pololu_imu();




#endif