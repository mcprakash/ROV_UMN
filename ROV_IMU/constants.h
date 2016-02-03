/*********************************************************************
Constants Header file

Definition of constants
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

#ifndef constants_h
#define constants_h

#define grav 9.81
#define MAX_STORAGE 100
#define DELTA_T 0.01f
#define epsilon_quat 0.01

#define pol_mag_biasx 0.129
#define pol_mag_biasy 0.081
#define pol_mag_biasz  0.243
#define BIAS_COUNT_MAX 1000

#define PI 3.141592653589793
#define pol_gyro_sens 8.75/1000.0*PI / 180.0 
//#define pol_gyro_sens 2*7.6296/1000.0*PI / 180.0 

#define PROTO_SELECT=1


#define yei_mag_biasx -0.016
#define yei_mag_biasy -0.0049
#define yei_mag_biasz -0.122

#endif

