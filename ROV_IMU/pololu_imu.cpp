/*********************************************************************
Pololu IMU Source code

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

#include <L3G.h>
#include <LSM303.h>
#include <Wire.h>
#include "pololu_imu.h"
#include "encoder.h"
#include "yei.h"
#include "base_fcns.h"
#include "nav.h"

using namespace Eigen;

LSM303 acc_mag;
L3G gyro;

float norm_acc;

VectorXf pol_acc(3), pol_acc_temp(3), grav_global(3);
MatrixXf dcm(3, 3);
MatrixXf K_acc(3, 3);
VectorXf b_acc(3);

void init_pololu_imu()
{
	pinMode(23, OUTPUT);
	digitalWrite(23, LOW);

	// Initialize Scale factor and Constant bias of accelerometer

	K_acc << 1.018738699621334, -0.001334222724689, 0.002981899340733,
			-0.001334222724689,  0.979454316088211,-0.000834151778617,
			 0.002981899340733, -0.000834151778617, 0.977600094815186;

	b_acc << -0.046014399701080, 0.128313608409085, -0.424804618233981;

	// Set all initial sensor values to zero
	pol_acc.setZero(3);
	pol_acc_temp.setZero(3);
	dcm = MatrixXf::Identity(3,3);
	grav_global.setZero(3);

	acc_mag.init();
	acc_mag.enableDefault();

	/*Serial.print("pololu imu timeout= ");
	Serial.println(acc_mag.getTimeout());*/

	if (!gyro.init())
	{
		Serial.println("Failed to autodetect gyro type!");
		//while (1);
	}

	gyro.enableDefault();

	// Do Gyroscope Bias Estimation and Accelerometer global frame estimation
	int bias_count = 0;
	while (bias_count < BIAS_COUNT_MAX)
	{
		acc_mag.read();
		gyro.read();
		imu.imu_data.omega_pol[0] = (float)(gyro.g.x)*pol_gyro_sens;
		imu.imu_data.omega_pol[1] = (float)(gyro.g.y)*pol_gyro_sens;
		imu.imu_data.omega_pol[2] = (float)(gyro.g.z)*pol_gyro_sens;
		imu.imu_data.gyrobias_pol[0] = (imu.imu_data.gyrobias_pol[0] * bias_count + imu.imu_data.omega_pol[0]) / (bias_count + 1);
		imu.imu_data.gyrobias_pol[1] = (imu.imu_data.gyrobias_pol[1] * bias_count + imu.imu_data.omega_pol[1]) / (bias_count + 1);
		imu.imu_data.gyrobias_pol[2] = (imu.imu_data.gyrobias_pol[2] * bias_count + imu.imu_data.omega_pol[2]) / (bias_count + 1);

		pol_acc_temp << (float)(acc_mag.a.x >> 4) * 1 / 1000 * grav, (float)(acc_mag.a.y >> 4) * 1 / 1000 * grav, (float)(acc_mag.a.z >> 4) * 1 / 1000 * grav;

		dcm = quat2dcm(get_current_quat());
		pol_acc = dcm*R_pol_cam*(K_acc*pol_acc_temp+b_acc);

		grav_global = (grav_global*bias_count + pol_acc) / (bias_count + 1);

		bias_count++;
		delay(5);
	}

	Serial.print("Pololu Gyro Bias = ");
	Serial.print(imu.imu_data.gyrobias_pol[0], 4); Serial.print(",");
	Serial.print(imu.imu_data.gyrobias_pol[1], 4); Serial.print(",");
	Serial.println(imu.imu_data.gyrobias_pol[2], 4);

	print_vectorxf("grav_global", grav_global, 3);

}

void get_pololu_imu()
{
	// <-------------------- Get Raw Data ----------------------->
	acc_mag.read();
	gyro.read();

	imu.imu_data.mag_pol[0] = (float)(acc_mag.m.x)*0.16 / 1000 - pol_mag_biasx;
	imu.imu_data.mag_pol[1] = (float)(acc_mag.m.y)*0.16 / 1000 - pol_mag_biasy;
	imu.imu_data.mag_pol[2] = (float)(acc_mag.m.z)*0.16 / 1000 - pol_mag_biasz;

	imu.imu_data.omega_pol[0] = (float)(gyro.g.x)*pol_gyro_sens - imu.imu_data.gyrobias_pol[0];
	imu.imu_data.omega_pol[1] = (float)(gyro.g.y)*pol_gyro_sens - imu.imu_data.gyrobias_pol[1];
	imu.imu_data.omega_pol[2] = (float)(gyro.g.z)*pol_gyro_sens - imu.imu_data.gyrobias_pol[2];

	pol_acc_temp<< (float)(acc_mag.a.x >> 4) * 1 / 1000 * grav, (float)(acc_mag.a.y >> 4) * 1 / 1000 * grav, (float)(acc_mag.a.z >> 4) * 1 / 1000 * grav;

	dcm = quat2dcm(get_current_quat());
	pol_acc = dcm*R_pol_cam*(K_acc*pol_acc_temp + b_acc) -grav_global;

	/*if (Serial.available() > 0)
	{
		print_vectorxf("pol_acc_temp", pol_acc_temp, 3);
		print_vectorxf("grav_global", grav_global, 3);
		print_vectorxf("pol_acc", pol_acc, 3);
	}*/

	imu.imu_data.acc_pol[0] = (float)pol_acc(0) ;
	imu.imu_data.acc_pol[1] = (float)pol_acc(1) ;
	imu.imu_data.acc_pol[2] = (float)pol_acc(2) ;


	ip_acc_storage(index_imu, 0) = (double)pol_acc(0) ;
	ip_acc_storage(index_imu, 1) = (double)pol_acc(1) ;
	ip_acc_storage(index_imu, 2) = (double)pol_acc(2) ;

	// <-------------------- Filter the Accelerometer ----------------------->
	
	filter();

	imu.imu_data.acc_pol_filt[0] = (float)op_acc_storage(index_imu, 0);
	imu.imu_data.acc_pol_filt[1] = (float)op_acc_storage(index_imu, 1);
	imu.imu_data.acc_pol_filt[2] = (float)op_acc_storage(index_imu, 2);

	nav.data.acc_pol_filt[0] = (float)op_acc_storage(index_imu, 0);
	nav.data.acc_pol_filt[1] = (float)op_acc_storage(index_imu, 1);
	nav.data.acc_pol_filt[2] = (float)op_acc_storage(index_imu, 2);

}