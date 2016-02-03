/*********************************************************************
SO3-PnP

Algorithm to estimate Position and Orientation from image data
using SO3-PnP
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

// <------------------------------------------------ Standard Libraries ---------------------------------------------->
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "graphics.h"
#include "so3.h"

using namespace std;
using namespace cv;
using namespace Eigen;

// Global Constants
int W_img=MAIN_TEXTURE_WIDTH, H_img=MAIN_TEXTURE_HEIGHT;
int n = 4;
Mat cameraMatrix, distCoeffs;
const static IOFormat CSVFormat(FullPrecision,DontAlignCols,",","\n");

// Landmark points and Image points
MatrixXf cam_mat(2,2);
VectorXf cen(2);
MatrixXf landmark(n, 3), pt_img(n, 2), pt_reproject(n,2);
VectorXf pt_temp(3);
float pts[4][2];

// Variables for SO3 algorithm
MatrixXf R(3, 3);
VectorXf rgm(3);
VectorXf gam(2*n);
MatrixXf step(3, 3);
MatrixXf N(3, 3);
MatrixXf beta1(2 * n, 3), Pr1(3, 2 * n), Pr(2 * n, 2 * n), Ra(3, 3), Rc(3, 3);
VectorXf Veca(3), Vecc(3), err(2 * n), dummyrgm(3);
MatrixXf R_cam(3,3);
VectorXf r_cam(3);
float error_a, error_c, stepsize=M_PI/72;

// Variables for SO3 Iteration
int k3 = 0, k1 = 0, k2 = 0;
int k=0;

// Variables for SO3 Simulation
float angx = 0, angy = M_PI / 8, angz = 0;


// Variables for Function quatrot
VectorXf ax;
MatrixXf R_ret(3, 3);	
float ang,c_ang,s_ang;

// Variables for Reordering points
float ltriangle[4],ltr;

// Log files for storing data 

ofstream f_rgm, f_err, f_rcam, f_Rcam, f_pix;


//<----------------------Function to Swap Vertices ------------------------->//
void swapfcn(int v1,int v2)
{
  pt_img(v1,0)=pts[v2][0];
  pt_img(v1,1)=pts[v2][1]; 
  pt_img(v2,0)=pts[v1][0];  
  pt_img(v2,1)=pts[v1][1];

  pts[v2][0]=pt_img(v2,0);
  pts[v2][1]=pt_img(v2,1); 
  pts[v1][0]=pt_img(v1,0);  
  pts[v1][1]=pt_img(v1,1);
 
  ltr=ltriangle[v2]; 
  ltriangle[v2]=ltriangle[v1];
  ltriangle[v1]=ltr;
}

//<---------------------- Function quatrot to get rotation matrix------------------------->//
MatrixXf quatrot(VectorXf Vec)
{
	ax=Vec;
	ang = Vec.norm();
	ax = ax / ang;
	c_ang = cos(ang), s_ang = sin(ang);

	R_ret << c_ang + ax(0)*ax(0)*(1 - c_ang), ax(0)*ax(1)*(1 - c_ang) - ax(2)*s_ang, ax(0)*ax(2)*(1 - c_ang)+ax(1)*s_ang,
	         ax(0)*ax(1)*(1 - c_ang) + ax(2)*s_ang, c_ang + ax(1)*ax(1)*(1 - c_ang), ax(1)*ax(2)*(1 - c_ang)-ax(0)*s_ang,
		 ax(0)*ax(2)*(1 - c_ang) - ax(1)*s_ang, ax(1)*ax(2)*(1 - c_ang) + ax(0)*s_ang, c_ang + ax(2) *ax(2) * (1 - c_ang);

	return R_ret;
	
}


//<---------------------- Function to calcualte rotation error------------------------->//
void err_calc(MatrixXf& R, int flag, VectorXf& err, VectorXf& Vec, VectorXf& rgm)
{
	gam=VectorXf::Zero(2*n);
	
	N = MatrixXf::Zero(3,3);

	for (k = 0; k < n; k++)
	{
		gam.segment(2 * k, 2) = beta1.block(2 * k, 0, 2, 3)*R*(landmark.row(k)).transpose();
	}
	if (flag)
		rgm = Pr1*(- gam);
	
	err = Pr*(- gam);

	for (k = 0; k < n; k++)
	{
		N = N + (landmark.row(k)).transpose()*(err.block(2 * k, 0, 2, 1)).transpose()*beta1.block(2 * k, 0, 2, 3);
	}

	step = R.transpose()*N.transpose() - N*R;

	Vec << step(2, 1), step(0, 2), step(1, 0);
	
	Vec /= Vec.norm();

}

//<---------------------- Function to calcualte optimal rotation matrix------------------------->//
void findPosSO3(MatrixXf& pt_img, MatrixXf& R_guess)
{
	R = R_guess;

	for (int k = 0; k < n; k++)
	{
		beta1(2*k,2)=pt_img(k,0)-W_img/2;
		beta1(2*k+1,2)=pt_img(k,1)-H_img/2;
		//beta1(2*k,2)=pt_img(k,0)-cen(0);
		//beta1(2*k+1,2)=pt_img(k,1)-cen(1);
	
	}

	Pr1 = (beta1.transpose()*beta1).inverse()*beta1.transpose();

	Pr = MatrixXf::Identity(2 * n, 2 * n) - beta1*Pr1;
	
	Ra = R;

	err_calc(Ra, 0, err, Veca, dummyrgm);

	error_a = err.norm();

	error_c = error_a;
	Rc = Ra;
	Vecc = Veca;

	stepsize=M_PI/72;

	while (stepsize > M_PI / 1440)
	{
		k2++;
		
		while (Veca.transpose()*Vecc>0)
		{
			error_a = error_c;
			Ra = Rc;
			Veca = Vecc;
			Rc = Ra*quatrot(stepsize*Veca);
			err_calc(Rc, 0, err, Vecc, dummyrgm);
			error_c = err.norm();
			k3++;
		}

		while (Veca.transpose()*Vecc < 0)
		{
			stepsize *= 0.98 / (1 - Veca.transpose()*Vecc);
			Rc = Ra*quatrot(stepsize*Veca);
			err_calc(Rc, 0, err, Vecc, dummyrgm);
			error_c = err.norm();
			f_err<<err.norm()<<",";
			k1++;
		}
	}

	R = Rc;
	err_calc(R, 1, err, dummyrgm, rgm);
	
	R_cam=R.transpose();
	
	r_cam=-R_cam*rgm;
	
	//<--------------------- Copy Data to wifi send module for Vision debugging -------------------------->
	
	VectorXf q= dcm2quat(R_cam);
	
	//pthread_mutex_lock(&cam_data_mutex);
	
	for(i=0;i<4;i++)
		cam_data.cam_pose.q[i]=q(i);
	
	for(i=0;i<3;i++)
		cam_data.cam_pose.pos[i]=r_cam(i);
	
	//pthread_mutex_unlock(&cam_data_mutex);
	
	//<--------------------- Display and Log Data -------------------------->
	cout<<"R_Cam=\n"<<R_cam<<endl<<endl;
	cout<<"r_cam=\n"<<r_cam<<endl<<endl;
	//cout<<"r_rgm="<<rgm<<endl<<endl;
	
	f_Rcam<<(q.transpose()).format(CSVFormat)<<endl<<endl;
	f_rcam<<(r_cam.transpose()).format(CSVFormat)<<endl;
	f_rgm<<(rgm.transpose()).format(CSVFormat)<<endl;
	for(int i=0;i<4;i++)
	  for(int j=0;j<2;j++)
			f_pix<<pt_img(i,j)<<",";
	f_pix<<endl;
	f_err<<endl;
	

}


//<---------------------- Function to initialize Matrices------------------------->//
void init_SO3()
{
	cout<<"Initialize SO3 Matrices \r\n";
	
	// Initialize Matrices
	R = MatrixXf::Identity(3, 3);
	Ra = R;
	Rc = R;
	R_cam=R;

	//landmark << 0, -14.2 ,0, 0, 0, 0, 19, 0, 0, 4.62, -4.62, 10;
	//pt_img << 395.2982, 381.2661, 399.9229, 454.0621, 495.9957, 447.6127, 417.5306, 421.1901;

    // First Hole which was replaced by the second one as it was loose
	landmark << 0, -15 ,0, 0, 0, 0, 20, 0, 0, 7.5, -4, -7.1;
	
	// Hole currently being used
	landmark << 0, -15 ,0, 0, 0, 0, 20, 0, 0, 10, -8, -10;
	
	Veca << 0, 0, 0;
	dummyrgm << 0, 0, 0;
	r_cam=dummyrgm;

	rgm<<0,0,200;


	// Proof of Concept Camera 
	// FileStorage fs("cam_calib/picam_calib_init_old.yml", FileStorage::READ);
	
	// Prototype Camera 
	// FileStorage fs("cam_calib/picam_calib_proto_air.yml", FileStorage::READ);
	
	// Prototype Camera Underwater 
	 FileStorage fs("cam_calib/picam_calib_proto_water.yml", FileStorage::READ);
	
	fs["cameraMatrix"] >> cameraMatrix;
	//fs["distCoeffs"] >> distCoeffs;

	//cout << cameraMatrix << endl;
	//cout << distCoeffs << endl;

	cam_mat << cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(0,1), cameraMatrix.at<double>(1,0), cameraMatrix.at<double>(1,1);
	cen<<cameraMatrix.at<double>(0,2),cameraMatrix.at<double>(1,2);
	cout<<"cam_mat=\n"<<cam_mat<<endl;

	//cam_mat << -1000.0, 0, 0, -1000; 

	for(k=0;k<n;k++)
		beta1.block(2*k, 0, 2, 2) = -cam_mat;
	
	for(i=0;i<4;i++)
		cam_data.cam_pose.q[i]=0;
	
	for(i=0;i<3;i++)
		cam_data.cam_pose.pos[i]=0;
	
	cout<<"Completed Initializing SO3 Matrices"<<endl<<endl;
	
	cout<<"Initializing Log Files"<<endl<<endl;
	
	f_rgm.open("log_files/rgm.txt");
	f_err.open("log_files/err.txt");
	f_rcam.open("log_files/poscam.txt");
	f_Rcam.open("log_files/Rcam.txt");
	f_pix.open("log_files/pix.txt");
	
	cout<<"Initialized Log Files"<<endl<<endl;
	
}

//<---------------------- Function to Reproject calculated position------------------------->//
void reproject()
{
	
	for(k=0;k<n;k++)
	{
		pt_temp=rgm+R*(landmark.row(k)).transpose();
		pt_reproject(k,0)=cam_mat(0,0)*pt_temp(0)/pt_temp(2)+W_img/2;
		pt_reproject(k,1)=cam_mat(1,1)*pt_temp(1)/pt_temp(2)+H_img/2;
	}
}

//<---------------------- Function to convert from DCM to Quaternion------------------------->//
VectorXf dcm2quat(MatrixXf R)
{
	VectorXf q(4);
	q(0) = sqrt(abs((1 + R(0, 0) + R(1, 1) + R(2, 2)) / 4));

	if (abs(q(0))>epsilon)
	{
		q(1) = (R(2, 1) - R(1, 2)) / 4 / q(0);
		q(2) = (R(0, 2) - R(2, 0)) / 4 / q(0);
		q(3) = (R(1, 0) - R(0, 1)) / 4 / q(0);
	}
	else
	{
		if (R(0, 0) >= R(1, 1) && R(0, 0) >= R(2, 2))
		{
			q(1) = sqrt((1 + R(0, 0)) / 2);
			q(2) = R(0, 1) / sqrt(2 * (1 + R(0, 0)));
			q(3) = R(0, 2) / sqrt(2 * (1 + R(0, 0)));
		}
		else if (R(1, 1) >= R(0, 0) && R(1, 1) >= R(2, 2))
		{
			q(2) = sqrt((1 + R(1, 1)) / 2);
			q(1) = R(1, 0) / sqrt(2 * (1 + R(1, 1)));
			q(3) = R(1, 2) / sqrt(2 * (1 + R(1, 1)));
		}
		else if (R(2, 2) >= R(0, 0) && R(2, 2) >= R(0, 0))
		{
			q(3) = sqrt((1 + R(2, 2)) / 2);
			q(1) = R(2, 0) / sqrt(2 * (1 + R(2, 2)));
			q(2) = R(2, 1) / sqrt(2 * (1 + R(2, 2)));
		}
	}

	return q;
}

//<---------------------- Main Function for Debugging ------------------------->//
/*
int main()
{
	init_SO3();
	
	reorder_pts();	
	
	findPosSO3(pt_img, R, 1);
	
	cout << "errror=\n" << err << endl;

	cout << "R=\n" << R << endl;

	cout << "rgm=\n" <<rgm <<endl;

	
}
*/
