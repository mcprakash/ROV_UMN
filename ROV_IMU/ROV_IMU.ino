/*********************************************************************
Arduino

Main Function 

Masters Project - Vision Aided Inertial Localization for ROV
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

#include <Wire.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <Ethernet.h>
#include <Eigen313.h>
#include <LU>
#include <Dns.h>
#include <Dhcp.h>
#include <L3G.h>
#include <LSM303.h>

#include "encoder.h"
#include "yei.h"
#include "base_fcns.h"
#include "pololu_imu.h"
#include "nav.h"
#include "constants.h"

// --------------------------------------------------- Extern Structures for holding data ------------------------------------------------------------

// Navigation Data Structure 
float_char nav;

// Camera Data Structure
float_char1 cam;

// IMU Data Structure (Contains Raw sensor data) 
// Not transmitted to ground station(Laptop)
float_char2 imu;

// fusion_cnt - The count of new camera data ; fusion_div - The new cam data to be skipped 1=0 data skipped; 2=1 data skipped and so on
int fusion_cnt=0, fusion_div=10;

void setup()
{
	//Begin Serial
	Serial.begin(115200);
	Wire.begin();

	delay(1000);
	Serial.println("Starting Program");

	wait_for_start();

	// Initialize UDP on Arduino
	init_UDP();

	Serial.println("Waiting for UDP Connection from RPI");
	// Wait for Camera Data to start the Program
	while (1)
	{
		if (Udp.parsePacket())
		{
			UDP_Recv_Data();
			cam.pose.t_cap = tcap;
			for (int i = 0; i < 4; i++)
			{
				nav.data.q_pol[i] = cam.pose.q[i];
				nav.data.q_fused[i] = cam.pose.q[i];
				if (i != 3)
				{
					nav.data.r_fused[i] = cam.pose.pos[i];
					nav.data.r_pol[i] = cam.pose.pos[i];
				}
			}

			Serial.println("Received UDP packet from RPI Cam");

			break;
		}
		check_for_stop();
	}

	Serial.println("Initialiing All Modules");

	// Initialize Navigation Module
	init_nav();
	delay(1000);

	//Initialize Pololu IMU
	init_pololu_imu();
	delay(1000);

	//Initialize Encoder
	init_encoder();
	//delay(1000);

	////Wait for User to Press 'a' to start 
	//Serial.println("Enter a to start");
	//wait_for_start();

	// Initialize YEI IMU
	//init_YEI();
	
	// Initialize interrupts for t_cap
	init_interrupts();

	Serial.println("Starting Navigation");
}

void loop()
{
	while (1)
	{

		//nav.data.t = millis();
		
		// Update YEI encoder
		//update_YEI();

		
		//if (updateimu_flag == true)
		if (millis()-nav.data.t>=(DELTA_T-0.001)*1000.0)
		{
			dt = ((float)micros() / 1000.0 - nav.data.t)/1000.0f;
			nav.data.t = (float)micros()/1000.0;
			get_encoder();
			//print_encoder();

			get_pololu_imu();

			// Time Update the UROV state
			orient_update();
			pos_update();

			UDP_Send_Data();
			
			//Serial.println(nav.data.t, 4);

			index_imu++;

			index_imu_prev = index_imu - 1;

			if (index_imu == 0)
				index_imu_prev = MAX_STORAGE - 1;

			if (index_imu > MAX_STORAGE - 1)
			{
				index_imu = 0;
			}

			//Serial.print("dt="); Serial.println(dt, 4);
		}
		

		
		// Do sensor fusion if image data is received
		if (Udp.parsePacket() == sizeof(cam.pose) - 4 && tcap_changed == true)
		{
			UDP_Recv_Data();
			//tproc = (float)micros() / 1000.0;
			
			if (tcap > tproc && tproc - tcap_prev < 150 && tproc - tcap_prev > 25)
			{
				index_forfusion = index_imu - floor((tproc - tcap_prev) / (DELTA_T * 1000)) >= 0 ? index_imu - floor((tproc - tcap_prev) / (DELTA_T * 1000)) : index_imu - floor((tproc - tcap_prev) / (DELTA_T * 1000)) + MAX_STORAGE;
				ndt = floor(tproc - tcap_prev) / 1000.0f;
				
				if (fusion_cnt%fusion_div==0)
					sensor_fusion();
				fusion_cnt++;

				/*
				Serial.print("index_imu="); Serial.println(index_imu);
				Serial.print("index_forfusion="); Serial.println(index_forfusion);
				Serial.println();
				*/

				cam.pose.t_cap = tcap_prev;
			}
			else if (tproc - tcap < 150 && tproc-tcap > 25)
			{
				index_forfusion = index_imu - floor((tproc - tcap) / (DELTA_T * 1000)) >= 0 ? index_imu - floor((tproc - tcap) / (DELTA_T * 1000)) : index_imu - floor((tproc - tcap) / (DELTA_T * 1000)) + MAX_STORAGE;
				ndt = floor(tproc - tcap) / 1000.0f;
				
				if (fusion_cnt%fusion_div == 0)
					sensor_fusion();

				fusion_cnt++;

				/*
				Serial.print("index_imu="); Serial.println(index_imu);
				Serial.print("index_forfusion="); Serial.println(index_forfusion);
				Serial.println();
				*/

				cam.pose.t_cap = tcap;
			}
			
			tcap_changed = false;
			/*
			Serial.print("t_cap_prev=");  Serial.println(tcap_prev, 4);
			Serial.print("t_cap="); Serial.println(tcap, 4);
			Serial.print("t_proc=");  Serial.println(tproc, 4);
			Serial.print("index_imu="); Serial.println(index_imu);
			Serial.print("index_forfusion="); Serial.println(index_forfusion);
			Serial.println();
			*/
		}

		check_for_stop();

	}

}