/*********************************************************************
Helping functions Header file 

Functions to help the main function
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

#include <Eigen313.h>
#include <LU>

#include <SPI.h>
#include <EthernetUdp.h>
#include <Ethernet.h>
#include "constants.h"

using namespace Eigen;
extern char ch;
extern bool updateimu_flag, tcap_changed;

extern EthernetUDP Udp;

extern float tcap, tproc, tcap_prev, dt, ndt;

extern int fusion_cnt;

void wait_for_start();
void check_for_stop();

union float_char
{
	struct temp
	{									// Matlab index
		float t;						// 1 
		float q_yei[4];					// 2-5
		
		float q_pol[4];					// 6-9
		float v_pol[3];					// 10-12
		float r_pol[3];					// 13-15

		float q_fused[4];				// 16-19
		float r_fused[3];				// 20-22

		float enc[3];					// 23-25

		float acc_pol_filt[3];			// 26-28
	}data;

	char ch[sizeof(data)];
}extern nav;

union float_char1
{
	struct temp
	{
		float q[4];						// 29-32
		float pos[3];					// 33-35
		float t_cap;					// 36
	}pose;

	char ch[sizeof(pose)];
}extern cam;

union float_char2
{
	struct temp
	{
		float t;						// 37
		float omega_pol[3];				// 38-40
		float gyrobias_pol[3];			// 41-43
		float acc_pol[3];				// 44-46	
		float acc_pol_filt[3];			// 47-49
		float mag_pol[3];				// 50-52

		float omega_yei[3];				// 53-55
		float acc_yei[3];				// 56-58
		float mag_yei[3];				// 59-61
		float q_err[4];					// 62-65
		//float q1[4];					// 65-68
		//float q2[4];					// 69-72
	}imu_data;

	char ch[sizeof(imu_data)];
}extern imu;

extern float tcam, tproc;

void send_data();

void print_data();

void custom_delay();

void init_UDP();

void init_interrupts();

//void UDP_Send_Data(IPAddress ip, unsigned int port, int num_bytes, char* data);
void UDP_Send_Data();
void UDP_Recv_Data();

void print_vectorxf(char name[], VectorXf& vec, int n);

void print_matrixxf(char name[], MatrixXf& mat, int m, int n);
