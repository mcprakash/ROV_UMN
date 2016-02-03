/*********************************************************************
Helping functions Source code

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

#include "Arduino.h"
#include "yei.h"
#include "base_fcns.h"
#include "pololu_imu.h"
#include "nav.h"

char ch;
int t_print;

unsigned int port_pc = 6500, localPort = 8888, rpi_port = 7500;
EthernetUDP Udp;

// --------------------------------------------------- Ethernet ------------------------------------------------------------
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0x04, 0x6E };
IPAddress ip(10, 0, 0, 10), ip_rpi(10, 0, 0, 50), ip_pc(10, 0, 0, 100);

// --------------------------------------------------- Camera Interrupt ------------------------------------------------------------
const int ISR_RPI_PIN_0 = 11;
const int ISR_RPI_PIN_1 = 13;

float tcap=0, tproc=0, tcap_prev=0, dt=0.005, ndt=0.05;

bool tcap_changed = false;

void wait_for_start()
{
	while (1)
	{
		if (Serial.available() > 0)
		{
			ch = Serial.read();
			if (ch == 'a')
			{
				t_print = millis();
				break;
			}
		}
	}
}

void check_for_stop()
{
	if (Serial.available() > 0)
	{
		ch = Serial.read();
		if (ch == 'b')
		{
			Serial.print("Fusion count ="); Serial.println(fusion_cnt);
			print_quatstorage();
			Serial.println("Exiting Program");
			IMUsetColor(0, 1, 0);
			IMUstop_streaming();
			IMUsetColor(0, 1, 0);
			Serial2.flush();
			Serial2.end();
			Serial.flush();
			Serial.end();
			exit(0);
		}
	}
}


void send_data()
{
	Serial.write(nav.ch, sizeof(nav.data));
}

void print_data()
{ 
	if (millis() - t_print >= 1000)
	{
		t_print = millis();
	}
}

void custom_delay()
{
	while (1)
	{
		if (millis() - nav.data.t >= 1000)
		{
			break;
		}
	}
}

// --------------------------------------------------- Ethernet Send Data -----------------------------------------------------------
void UDP_Send_Data()

{
	Udp.beginPacket(ip_rpi, rpi_port);
	Udp.write(nav.ch, sizeof(nav.data));
	Udp.write(cam.ch, sizeof(cam.pose));
	Udp.write(imu.ch, sizeof(imu.imu_data));
	Udp.endPacket();
}

// --------------------------------------------------- Ethernet Receive Data ------------------------------------------------------------
void UDP_Recv_Data()
{
	Udp.read(cam.ch, sizeof(cam.pose)-4);
}

// --------------------------------------------------- Initialize UDP ------------------------------------------------------------
void init_UDP()
{
	Ethernet.begin(mac, ip);
	Udp.begin(localPort);
}

// --------------------------------------------------- ISR Routines for RPI  Camera Capture Timing Selection------------------------------------------------------------
void ISR_tcap()
{
	if (tcap_changed == false)
	{
		tcap_prev = tcap;
		tcap = (float)micros() / 1000.0;
		tcap_changed = true;
		//Serial.print("tcap="); Serial.println(tcap, 4);
	}
}
void ISR_tproc()
{
	tproc = (float)micros()/1000.0;
	//Serial.print("t_proc="); Serial.println(tproc, 4);
}

// --------------------------------------------------- Initialize RPI signal interrupts ------------------------------------------------------------
void init_interrupts()
{
	attachInterrupt(ISR_RPI_PIN_0, ISR_tcap, RISING);
	attachInterrupt(ISR_RPI_PIN_1, ISR_tproc, RISING);
}

// --------------------------------------------------- Function to print VectorXf variable ------------------------------------------------------------
void print_vectorxf(char name[], VectorXf& vec, int n)
{
	Serial.print(name); Serial.print("=");
	for (int i = 0; i < n - 1; i++)
	{
		Serial.print(vec(i), 4); Serial.print(",");
	}
	Serial.println(vec(n - 1), 4);
	Serial.println();
}

// --------------------------------------------------- Function to print MatrixXf variable ------------------------------------------------------------
void print_matrixxf(char name[], MatrixXf& mat, int m, int n)
{
	Serial.print(name); Serial.print("=");
	for (int i = 0; i < m ; i++)
	{
		for (int j = 0; j < n; j++)
		{
			Serial.print(mat(i,j), 4); Serial.print(",");
		}
		Serial.println();
	}
	Serial.println();
}