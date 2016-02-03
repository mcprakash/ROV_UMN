/*********************************************************************
Navigation source code

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

#include "nav.h"
#include "encoder.h"
#include "yei.h"
#include "base_fcns.h"
#include "pololu_imu.h"

using namespace Eigen;

// Relative orientation of yei and pol from camera (Obtained by calibration)
MatrixXf R_yei_cam(3, 3), R_pol_cam(3, 3);

// Order of feedforward and feedback filter 
// n= Order of Filter 
// index_imu = circular array current data index_imu
// index_forfusion = array index to be fused 
int n_filt = 5, index_imu = 0, index_forfusion = 0, index_imu_prev = MAX_STORAGE - 1;

// Co-efficients of feedforward and feedback filters
VectorXd a(n_filt), b(n_filt);

// Storage of previous input and output acc data of 3 axes 
MatrixXd ip_acc_storage(MAX_STORAGE, 3), op_acc_storage(MAX_STORAGE, 3);

// Vectors for storing the previous data to be filtered
MatrixXd z_filtdata(n_filt - 1, 3);

// Variables for Sensor Fusion

MatrixXf P_pos(6, 6), P_old_pos(6, 6), Q_pos(6, 6), R_pos(3, 3), K_pos(6, 3), H_pos(3, 6), eye66(6, 6), F_pos(6, 6), Omega(4, 4);
VectorXf x(6), y(3), q_fused(4), q_cam(4), q_err(4), q1(4), q2(4), dif_pos(3), dif_vel(3);
MatrixXf PHI_TRANSITION_POS(6, 6);
MatrixXf Q_ang(4, 4), P_ang(4, 4), R_ang(4, 4);

// Storing Q_posuaternions and Positions of Previous 100 time steps for going back in time and coming forward in time

MatrixXf quat_storage(MAX_STORAGE, 4);
MatrixXf pos_storage(MAX_STORAGE, 3);
MatrixXf vel_storage(MAX_STORAGE, 3);

void init_nav()
{
	/*
	//Coefficients from Matlab filter ------------- [b,a]=butter(5,30/500);
	//a << 1.000000000000000, -4.390276194260847, 7.742869540801032, -6.854349350895913, 3.044685309180256, -0.542751374933465;
	//b << 0.0000055603090957021, 0.0000278015454785105, 0.0000556030909570210, 0.0000556030909570210, 0.0000278015454785105, 0.0000055603090957021;

	// Coefficients from Matlab filter ------------- [b,a]=butter(3,[0.002, 0.5],'bandpass');
	//a << 1.000000000000000, -2.992656447290368, 3.326011754492057, -2.006256804262511, 1.006256969630407, -0.334402370391459, 0.001047228557668;
	//b << 0.165100251677446, 0, -0.495300755032338, 0, 0.495300755032338, 0, -0.165100251677446;
 
	//b << 1.000000000000000, -1.999999875041659, 0.999999875041659;
	//a << 1.000000000000000, -1.999000249958339, 0.999000499833375;

	// Coefficients from Matlab filter ------------- [b, a] = butter(1, 0.002, 'high');
	//b << 0.996868235770807 ,- 0.996868235770807;
	//a << 1.000000000000000, -0.993736471541615;

	// Coefficients from Matlab filter ------------- [b, a] = butter(1, 0.0015, 'high');
	//b << 0.997649339772466, -0.99764933977246;
	//a << 1.000000000000000, -0.995298679544933;

	// 2nd order high pass filter ------------------ [b, a] = butter(3, 0.002, 'high');
	//b << 0.993736502353988, -2.981209507061963, 2.981209507061963, -0.993736502353988;
	//a << 1.000000000000000, -2.987433650055722, 2.974946132665443, -0.987512236110736;
	*/

	b << 0.001*0.227569494707103, 0, -0.001*0.455138989414207, 0, 0.001*0.227569494707103;
	a << 1.000000000000000, -3.939609765991646, 5.819789650353378, -3.820748535029082, 0.940568651258018;

	z_filtdata.setZero(n_filt - 1, 3);

	ip_acc_storage.setZero(MAX_STORAGE, 3);
	op_acc_storage.setZero(MAX_STORAGE, 3);
	quat_storage.setZero(MAX_STORAGE, 4);
	pos_storage.setZero(MAX_STORAGE, 3);
	vel_storage.setZero(MAX_STORAGE, 3);

	for (int i = 0; i < MAX_STORAGE; i++)
	{
		quat_storage(i, 0) = nav.data.q_fused[0];
		quat_storage(i, 1) = nav.data.q_fused[1];
		quat_storage(i, 2) = nav.data.q_fused[2];
		quat_storage(i, 3) = nav.data.q_fused[3];

		pos_storage(i, 0) = nav.data.r_fused[0];
		pos_storage(i, 1) = nav.data.r_fused[1];
		pos_storage(i, 2) = nav.data.r_fused[2];


		/*VectorXf qtemp = quat_storage.row(i);
		Serial.println(i);
		print_vectorxf("qtemp=", qtemp, 4);*/
	}

	/* Obsolete Values for POC (Proof of Concept Model)
	R_pol_cam << 0.999651759547957, -0.012790268416985, 0.023081782134304,
		0.010002397026848, 0.993070532411838, 0.117093422996284,
		-0.024419493983114, -0.116821773180735, 0.992852638423507;*/

	/* Values for Broken Prototype
	R_pol_cam << 0.999756888095383, -0.021095073253415, -0.006415807842412,
		0.021674814247325, 0.993648085678438, 0.110425016436136,
		0.004045631369964, -0.110537232243304, 0.993863749794110;*/

	R_pol_cam << 0.999076204511510, -0.016770727096228, -0.039566150830433,
				 0.018762222191940, 0.998547253181831, 0.050511010497099,
				 0.038661564858306, -0.051206697566643, 0.997939455842400;

	R_yei_cam << 0.9998, 0.0171, -0.0130,
		-0.0154, 0.9928, 0.1187,
		0.0150, -0.1185, 0.9928;

	// Initialize Velocity as 0 and Positon as camera position
	nav.data.v_pol[0] = 0; nav.data.v_pol[1] = 0; nav.data.v_pol[2] = 0;
	nav.data.r_fused[0] = cam.pose.pos[0]; nav.data.r_fused[1] = cam.pose.pos[1]; nav.data.r_fused[2] = cam.pose.pos[2];

	// Co-variance Initialization
	P_pos = 10 * MatrixXf::Identity(6, 6);
	P_pos(3, 3) = 1;
	P_pos(4, 4) = 1;
	P_pos(5, 5) = 1;

	P_old_pos = P_pos;
	Q_pos = 0.05*MatrixXf::Identity(6, 6); 

	Q_pos(3, 3) = 10;
	Q_pos(4, 4) = 10;
	Q_pos(5, 5) = 10;

	R_pos = 6*MatrixXf::Identity(3, 3);

	// Transition Matrices and Coeffients Initialization
	H_pos << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
	eye66 = MatrixXf::Identity(6, 6);

	PHI_TRANSITION_POS = MatrixXf::Identity(6, 6);
	
	// F= I +A *dt
	F_pos = eye66; F_pos(0, 3) = dt; F_pos(1, 4) = dt; F_pos(2, 5) = dt;

	dif_pos.setZero();
	dif_vel.setZero();

	/*print_vectorxf("dif_pos_init", dif_pos, 3);
	print_vectorxf("dif_vel_init", dif_vel, 3);*/
}

// Form 2 Implementation of Matlab function filter()
void filter()
{
	
	//IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
	op_acc_storage.row(index_imu) = b(0)*ip_acc_storage.row(index_imu) + z_filtdata.row(0);

	for (int i = 1; i < n_filt; i++)
	{
		if (i != n_filt - 1)
		{
			z_filtdata.row(i - 1) = b(i)*ip_acc_storage.row(index_imu) + z_filtdata.row(i) - a(i)*op_acc_storage.row(index_imu);
		}
		else
		{
			z_filtdata.row(i - 1) = b(i)*ip_acc_storage.row(index_imu) - a(i)*op_acc_storage.row(index_imu);
		}
	}
	
	//cout << "z_filtdata=" << endl << z_filtdata << endl << endl;
	//cout << "op_acc_storage=" << endl << (op_acc_storage.row(index_imu)).format(HeavyFmt) << endl << endl;
	
}

void pos_update()
{
	// Give 20s of no motion of Wait Time for initial Boundary Settlement of HPF 
	
	// Update Velocity (cm/s) and Position (cm)

	// ---- x(t)=F*x(t-)+u ------- //

	vel_storage.row(index_imu) = vel_storage.row(index_imu_prev) + (op_acc_storage.row(index_imu)).cast<float>() * DELTA_T * 100;

	pos_storage.row(index_imu) = pos_storage.row(index_imu_prev) + vel_storage.row(index_imu)*DELTA_T;


	nav.data.v_pol[0] = vel_storage(index_imu, 0);
	nav.data.v_pol[1] = vel_storage(index_imu, 1);
	nav.data.v_pol[2] = vel_storage(index_imu, 2);

	nav.data.r_fused[0] = pos_storage(index_imu, 0);
	nav.data.r_fused[1] = pos_storage(index_imu, 1);
	nav.data.r_fused[2] = pos_storage(index_imu, 2);
	
	P_pos = P_old_pos;

	// ---- P=F*P*F'+Q ------- //
	/*P_old_pos(1, 4) = P_pos(1, 4) + DELTA_T*P_pos(1, 1); P_old_pos(1, 5) = P_pos(1, 5) + DELTA_T*P_pos(1, 2); P_old_pos(1, 6) = DELTA_T*P_pos(1, 3);
	P_old_pos(2, 4) = P_pos(2, 4) + DELTA_T*P_pos(2, 1); P_old_pos(2, 5) = P_pos(2, 5) + DELTA_T*P_pos(2, 2); P_old_pos(1, 6) = DELTA_T*P_pos(2, 3);
	P_old_pos(3, 4) = P_pos(3, 4) + DELTA_T*P_pos(3, 1); P_old_pos(3, 5) = P_pos(3, 5) + DELTA_T*P_pos(3, 2); P_old_pos(3, 6) = DELTA_T*P_pos(3, 3);

	P_old_pos(4, 1) = P_pos(4, 1) + DELTA_T*P_pos(1, 1); P_old_pos(4, 2) = P_pos(4, 2) + DELTA_T*P_pos(1, 2); P_old_pos(4, 3) = P_old_pos(4, 3) + DELTA_T*P_pos(1, 3);
	P_old_pos(4, 4) = P_pos(4, 4) + DELTA_T*(P_pos(1, 4) + P_pos(4, 1) + DELTA_T*P_pos(1, 1));
	P_old_pos(4, 5) = P_pos(4, 5) + DELTA_T*(P_pos(1, 5) + P_pos(4, 2) + DELTA_T*P_pos(1, 2));
	P_old_pos(4, 6) = P_pos(4, 4) + DELTA_T*(P_pos(1, 4) + P_pos(4, 1) + DELTA_T*P_pos(1, 1));*/


	P_old_pos = F_pos*P_pos*F_pos.transpose() + Q_pos;
}

void orient_update()
{
	// <-------------------- Get Sensor Data ----------------------->

	Omega << 0, -imu.imu_data.omega_pol[0], -imu.imu_data.omega_pol[1], -imu.imu_data.omega_pol[2],
		imu.imu_data.omega_pol[0], 0, imu.imu_data.omega_pol[2], -imu.imu_data.omega_pol[1],
		imu.imu_data.omega_pol[1], -imu.imu_data.omega_pol[2], 0, imu.imu_data.omega_pol[0],
		imu.imu_data.omega_pol[2], imu.imu_data.omega_pol[1], -imu.imu_data.omega_pol[0], 0;

	// <-------------------- Compute the Quaternion for Pololu Gyro (IMU) alone----------------------->
	//<-------------------- This section can be commented later on----------------------->
	/*
	Using Raw Floating point multiplicaitons; Better use Matrix library for cleaner codes
	float q_temp[4] = { nav.data.q_pol[0], nav.data.q_pol[1], nav.data.q_pol[2], nav.data.q_pol[3] };

	nav.data.q_pol[0] += 0.5*(-imu.imu_data.omega_pol[0] * q_temp[1] - imu.imu_data.omega_pol[1] * q_temp[2] - imu.imu_data.omega_pol[2] * q_temp[3])*DELTA_T;
	nav.data.q_pol[1] += 0.5*(imu.imu_data.omega_pol[0] * q_temp[0] + imu.imu_data.omega_pol[2] * q_temp[2] - imu.imu_data.omega_pol[1] * q_temp[3])*DELTA_T;
	nav.data.q_pol[2] += 0.5*(imu.imu_data.omega_pol[1] * q_temp[0] - imu.imu_data.omega_pol[2] * q_temp[1] + imu.imu_data.omega_pol[0] * q_temp[3])*DELTA_T;
	nav.data.q_pol[3] += 0.5*(imu.imu_data.omega_pol[2] * q_temp[0] + imu.imu_data.omega_pol[1] * q_temp[1] - imu.imu_data.omega_pol[0] * q_temp[2])*DELTA_T;
	

	float q_norm = sqrt(nav.data.q_pol[0] * nav.data.q_pol[0] + nav.data.q_pol[1] * nav.data.q_pol[1] + nav.data.q_pol[2] * nav.data.q_pol[2] + nav.data.q_pol[3] * nav.data.q_pol[3]);

	nav.data.q_pol[0] /= q_norm;
	nav.data.q_pol[1] /= q_norm;
	nav.data.q_pol[2] /= q_norm;
	nav.data.q_pol[3] /= q_norm;
	

	*/

	
	q1 << nav.data.q_pol[0], nav.data.q_pol[1], nav.data.q_pol[2], nav.data.q_pol[3];

	q2 = 0.5 * Omega*q1 * DELTA_T;

	q1 = q1 + q2;

	q1 /= q1.norm();

	nav.data.q_pol[0] = q1(0);
	nav.data.q_pol[1] = q1(1);
	nav.data.q_pol[2] = q1(2);
	nav.data.q_pol[3] = q1(3);

	// <-------------------- Compute the Quaternion for Sensor Fused Gyro Time Update----------------------->

	/*
	q_temp[0] = nav.data.q_fused[0];
	q_temp[1] = nav.data.q_fused[1];
	q_temp[2] = nav.data.q_fused[2];
	q_temp[3] = nav.data.q_fused[3];
	
	nav.data.q_fused[0] += 0.5*(-imu.imu_data.omega_pol[0] * q_temp[1] - imu.imu_data.omega_pol[1] * q_temp[2] - imu.imu_data.omega_pol[2] * q_temp[3])*DELTA_T;
	nav.data.q_fused[1] += 0.5*(imu.imu_data.omega_pol[0] * q_temp[0] + imu.imu_data.omega_pol[2] * q_temp[2] - imu.imu_data.omega_pol[1] * q_temp[3])*DELTA_T;
	nav.data.q_fused[2] += 0.5*(imu.imu_data.omega_pol[1] * q_temp[0] - imu.imu_data.omega_pol[2] * q_temp[1] + imu.imu_data.omega_pol[0] * q_temp[3])*DELTA_T;
	nav.data.q_fused[3] += 0.5*(imu.imu_data.omega_pol[2] * q_temp[0] + imu.imu_data.omega_pol[1] * q_temp[1] - imu.imu_data.omega_pol[0] * q_temp[2])*DELTA_T;

	q_norm = sqrt(nav.data.q_fused[0] * nav.data.q_fused[0] + nav.data.q_fused[1] * nav.data.q_fused[1] + nav.data.q_fused[2] * nav.data.q_fused[2] + nav.data.q_fused[3] * nav.data.q_fused[3]);

	nav.data.q_fused[0] /= q_norm;
	nav.data.q_fused[1] /= q_norm;
	nav.data.q_fused[2] /= q_norm;
	nav.data.q_fused[3] /= q_norm;

	quat_storage(index_imu,0)=nav.data.q_fused[0];
	quat_storage(index_imu,1)=nav.data.q_fused[1];
	quat_storage(index_imu,2)=nav.data.q_fused[2];
	quat_storage(index_imu,3)=nav.data.q_fused[3];

	*/

	
	q1 = quat_storage.row(index_imu_prev);

	q2 = 0.5 * Omega*q1 * DELTA_T;

	q1 = q1 + q2;

	q1 /= q1.norm();

	// <-------------------- Store the Time Update values----------------------->

	nav.data.q_fused[0] = q1(0);
	nav.data.q_fused[1] = q1(1);
	nav.data.q_fused[2] = q1(2);
	nav.data.q_fused[3] = q1(3);

	quat_storage.row(index_imu) = q1;

	//print_vectorxf("q1", q1, 4);
	
}

void sensor_fusion()
{
	
	// <-------------------- Orientation Fusion----------------------->
	
	q_cam << cam.pose.q[0], cam.pose.q[1], cam.pose.q[2], cam.pose.q[3];
	q1 = quat_storage.row(index_forfusion);
	q2 = quat_storage.row(index_imu_prev);
	
	// Check for ambiguity of quaternions since q and -q are the same representation
	if (q1(0)*q_cam(0) < 0)
		q1 = -q1;

	q_fused = q1+0.5*(q_cam-q1);
	q_fused = q_fused / q_fused.norm();
	q_err = quatmultiply(quatconj(q1), q_fused);
	q1 = quatmultiply(q2, q_err);

	//print_vectorxf("qcam", q_cam, 4);
	//print_vectorxf("q1", q1, 4);
	//print_vectorxf("q2", q2, 4);
	//print_vectorxf("q_temp", q_temp, 4);
	//print_vectorxf("qerr", q_err, 4);

	
	quat_storage.row(index_imu_prev) = q1;

	
	nav.data.q_fused[0] = q1(0);
	nav.data.q_fused[1] = q1(1);
	nav.data.q_fused[2] = q1(2);
	nav.data.q_fused[3] = q1(3);
	
	// <-------------------- Position Fusion----------------------->

	PHI_TRANSITION_POS(0, 3) = ndt;
	PHI_TRANSITION_POS(1, 4) = ndt;
	PHI_TRANSITION_POS(2, 5) = ndt;

	// Calculate the difference in position due to time lag and compensate for rotation
	dif_pos = (pos_storage.row(index_imu_prev) - pos_storage.row(index_forfusion));
	dif_vel = (vel_storage.row(index_imu_prev) - vel_storage.row(index_forfusion));

	
	// Do fusion when the image was captured
	y << cam.pose.pos[0], cam.pose.pos[1], cam.pose.pos[2];

	x.segment<3>(0) = pos_storage.row(index_forfusion);
	x.segment<3>(3) = vel_storage.row(index_forfusion);

	K_pos = P_old_pos*H_pos.transpose()*(H_pos*P_old_pos*H_pos.transpose() + R_pos).inverse();

	x += K_pos*(y - H_pos*x);

	P_pos = (eye66 - K_pos*H_pos)*P_old_pos;

	P_old_pos = P_pos;

	// Come forward in time and add the corrected and rotated dif_pos to the current time step
	pos_storage.row(index_imu_prev) = x.segment<3>(0) + quat2dcm(q1)*dif_pos;
	vel_storage.row(index_imu_prev) = x.segment<3>(3) + quat2dcm(q1)*dif_vel;


	// Update the states 
	/*nav.data.r_fused[0] = pos_storage(index_imu, 0);
	nav.data.r_fused[1] = pos_storage(index_imu, 1);
	nav.data.r_fused[2] = pos_storage(index_imu, 2);
	
	nav.data.v_pol[0] = vel_storage(index_imu, 0);
	nav.data.v_pol[1] = vel_storage(index_imu, 1);
	nav.data.v_pol[2] = vel_storage(index_imu, 2);
	*/

	//print_matrixxf("k", K_pos, 6, 3);
	//print_vectorxf("x", x, 6);
	
	/*
	print_vectorxf("dif_pos", dif_pos, 3);
	print_vectorxf("dif_vel", dif_vel, 3);
	*/
	
	
}

//<---------------------- Function to convert from DCM to Quaternion------------------------->//
VectorXf dcm2quat(MatrixXf R)
{
	VectorXf q(4);
	q(0) = sqrt(abs((1 + R(0, 0) + R(1, 1) + R(2, 2)) / 4));

	if (abs(q(0))>epsilon_quat)
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

VectorXf quatmultiply(VectorXf q, VectorXf r)
{
	VectorXf t(4);
	t(0) = r(0)*q(0) - r(1)*q(1) - r(2)*q(2) - r(3)*q(3);
	t(1) = r(0)*q(1) + r(1)*q(0) - r(2)*q(3) + r(3)*q(2);
	t(2) = r(0)*q(2) + r(1)*q(3) + r(2)*q(0) - r(3)*q(1);
	t(3) = r(0)*q(3) - r(1)*q(2) + r(2)*q(1) + r(3)*q(0);

	t /= t.norm();
	return t;
}

MatrixXf quat2dcm(VectorXf q)
{
	MatrixXf dcm(3, 3);

	dcm << 1 - 2 * q(2)*q(2) - 2 * q(3)*q(3), 2 * (q(1)*q(2) - q(3)*q(0)), 2 * (q(1)*q(3) + q(2)*q(0)),
		2 * (q(1)*q(2) + q(3)*q(0)), 1 - 2 * q(1)*q(1) - 2 * q(3)*q(3), 2 * (q(2)*q(3) - q(1)*q(0)),
		2 * (q(1)*q(3) - q(2)*q(0)), 2 * (q(2)*q(3) + q(1)*q(0)), 1 - 2 * q(1)*q(1) - 2 * q(2)*q(2);

	return dcm;

}

VectorXf quatconj(VectorXf q)
{
	VectorXf q_conj = q;
	q_conj(1) = -q(1);
	q_conj(2) = -q(2);
	q_conj(3) = -q(3);

	return q_conj;
}

VectorXf get_current_quat()
{
	VectorXf quat(4);
	quat << nav.data.q_fused[0], nav.data.q_fused[1], nav.data.q_fused[2], nav.data.q_fused[3];
	return quat;
}

void print_quatstorage()
{
	print_matrixxf("quat_storage", quat_storage, 100, 4);
	print_matrixxf("omega=", Omega, 4, 4);
}