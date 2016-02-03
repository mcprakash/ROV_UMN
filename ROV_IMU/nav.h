/*********************************************************************
Navigation Header file

Functions to optimally estimate the ROV sensor pose using fusion
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

#ifndef nav_h

#include <Eigen313.h>
#include <LU>
#include "constants.h"

using namespace Eigen;

// Order of feedforward and feedback filter 
extern int index_imu, index_forfusion, index_imu_prev;	// n= Order of Filter and index_imu = circular array current data index

// Storage of previous input and output acc data of 3 axes 
extern MatrixXd ip_acc_storage, op_acc_storage;

// Conversion between IMU Co-ordinates and Camera Co-ordinates
extern MatrixXf R_yei_cam, R_pol_cam;

void init_nav();

void filter();

void sensor_fusion();

void pos_update();

void orient_update();

VectorXf quatmultiply(VectorXf q, VectorXf r);

MatrixXf quat2dcm(VectorXf q);

VectorXf quatconj(VectorXf q);

VectorXf dcm2quat(MatrixXf R);

VectorXf get_current_quat();

void print_quatstorage();

#endif