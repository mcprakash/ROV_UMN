/*********************************************************************
SO3-PnP

Header file - Algorithm to estimate Position and Orientation from 
image data using SO3-PnP
**********************************************************************
Copyright(C)  <December 2015>  <Chandra P. Mangipudi, Perry Y. Li>

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

#ifndef so3_lib
#define so3_lib

// <------------------------------------------------ Standard Libraries ---------------------------------------------->
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#define epsilon 0.01

using namespace std;
using namespace Eigen;
using namespace cv;

// Union for UDP sending data

union float_char
{
	struct temp
	{
		float q[4];
		float pos[3];
	}cam_pose;
	
	char buf[sizeof(cam_pose)];
}extern cam_data;
	

// SO3 Variables


extern pthread_mutex_t cam_data_mutex;
extern MatrixXf R, pt_img;
extern Mat cameraMatrix, distCoeffs;
extern VectorXf r_cam;

extern int i, j;

MatrixXf quatrot(VectorXf Vec);
void err_calc(MatrixXf& R, int flag, VectorXf& err, VectorXf& Vec, VectorXf& rgm);
void findPosSO3(MatrixXf& pt_img, MatrixXf& R_guess);
void init_SO3();
void reproject();
VectorXf dcm2quat(MatrixXf R);

#endif
