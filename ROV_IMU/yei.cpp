/*********************************************************************
YEI IMU Source code

Functions to read YEI IMU for reference
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

// --------------------------------------------------- Header Files ------------------------------------------------------------
#include "Arduino.h"
#include "string.h"
#include "Ethernet.h"
#include "EthernetUdp.h"
#include "SPI.h" 
#include "yei.h"
#include "base_fcns.h"
#include "pololu_imu.h"

// --------------------------------------------------- Variable to know when to update IMU ------------------------------------------------------------
bool updateimu_flag = false;

// --------------------------------------------------- Buffers and Variables ------------------------------------------------------------

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
boolean linkup = false;
byte packet[255];
int avail = 0;
float state_update[3] = { 0 };

// --------------------------------------------------- YEI-IMU Variables ------------------------------------------------------------

float deltaT = 0.005; // interrupt interval
const boolean IS_BYTE = 1, NOT_BYTE = 0;
byte IntPinMode1[2], IntPinMode[2] = { 0, 0 }; //{0x01, 0x01};  //{0,0}

// unsigned int SRate=0, SRate1;

// --------------------------------------------------- YEI Streaming Variables ------------------------------------------------------------
byte streamChannel1[8], streamChannel[8] = { 0, 41, 39, 43, 38, 255, 255, 255 }; //{0,41,39,43,255,255,255,255};
const int BAUD = 460800, IMU_STREAM_SIZE = 60; // 48 including timestamp
unsigned int header = 2;  // timestamp - 4bytes
//const int streamTimes[3] = { 5000, 0xFFFFFFFF, 1000000 }; //{5000,0xFFFFFFFF,1000000}; //{Period, length, delay}
const int streamTimes[3] = { 10000, 0xFFFFFFFF, 1000000 }; //{5000,0xFFFFFFFF,1000000}; //{Period, length, delay}
const int init_timestamp = 0;

// Union for data transfer from IMU
union SPIdata {
	byte b[4];
	float fval;
	unsigned int ival;
};
union SPIdata acc[3], mag[3], quat[4], acc_raw[3], temp, timestamp, int_status, color[3], color1[3], dummy, streamTimes1[3],yei_gyro[3];
union SPIdata streamPack[11], cmd;
//float q_raw[4] = { 1, 0, 0, 0 }, q_norm=1;

//This is a convenience function to calc the last byte in the packet
// commands without parameters can use the same number as the command
unsigned char createChecksum(const unsigned char *pack,
	const unsigned int &num_bytes) {
	unsigned int chkSum = 0;
	for (unsigned int i = 0; i < num_bytes; i++) {
		chkSum += pack[i + 1];
	}
	return ((unsigned char)(chkSum % 256));
}

// --------------------------------------------------- IMU Functions------------------------------------------------------------

void IMU_Rd_Channel(union SPIdata *var, int num) {
	for (int ii = 0; ii < num; ii++) {
		for (int jj = 3; jj >= 0; jj--) {
			var[ii].b[jj] = Serial2.read();
		}
	}
}

void IMU_Rd_command(byte cmd, int num, boolean byte_type, union SPIdata *var) {
	// 0x00= tared quaternion; - 16bytes
	// 0x02= tared roation matrix; - 36bytes
	// 0x03= tared axis-angle; 16bytes
	// [ 0x1F= read interrupt status; 1 byte ] does not fit data structure
	// 0x29= tared linear acceleration; 12 bytes
	// 0x2B= get temperature - C; 4bytes

	// If byte_tyle == IS_BYTE, num means number of bytes (must be <= 4)
	byte a_word[4];
	int ii;
	IMU_Send_command(cmd, 0, IS_BYTE, &dummy);
	if (byte_type)  Serial2.readBytes((char*)var[0].b, num);
	else  for (ii = 0; ii < num; ii++) {
		Serial2.readBytes((char*)a_word, 4);
		var[ii].b[3] = a_word[0];
		var[ii].b[2] = a_word[1];
		var[ii].b[1] = a_word[2];
		var[ii].b[0] = a_word[3];
	}
}

void IMU_Rd_Bytes(byte cmd, int num, byte* var) {
	IMU_Send_Bytes(cmd, 0, (byte*)&dummy);
	Serial2.readBytes((char*)var, num);
}

void IMU_Send_Bytes(byte cmd, int num, byte* data) {
	byte packet[15];
	int jj, k, num_bytes;
	packet[0] = 0xF7;
	packet[1] = cmd;
	k = 2;
	num_bytes = num + 1;
	for (jj = 0; jj < num; jj++) {
		packet[k++] = data[jj];
	}
	packet[k++] = createChecksum(packet, num_bytes);
	Serial2.write(packet, k);
}

void IMU_Gyro_Calibrate()
{
	byte packet[3] = { 0xF7, 165, 165};
	Serial2.write(packet, 3);
}

void IMU_Send_command(byte cmd, int num, boolean byte_type, union SPIdata *data) {
	// 0x60= tare with current orientation - 0 bytes
	// 0x61= tare with quaternion - (4 floats)
	// 0x67= set update rate (micro sec) - (unsigned int)
	// 0xee= set LED color RGB - 12bytes (3 floats)
	// If bytletyle == IS_BYTE, num means number of bytes (must be <= 4)

	byte packet[15];
	int jj, k, num_bytes;
	if (cmd == 84 || cmd == 85)
		packet[0] = 0xF9;
	else
		packet[0] = 0xF7;
	packet[1] = cmd;
	k = 2;
	if (byte_type) {
		num_bytes = num + 1;
		for (jj = 0; jj < num; jj++) {
			packet[k++] = data[0].b[jj];
		}
	}
	else {
		num_bytes = 4 * num + 1;
		for (int ii = 0; ii < num; ii++) {
			for (int jj = 3; jj >= 0; jj--) {
				packet[k++] = data[ii].b[jj];
			}
		}
	}
	packet[k++] = createChecksum(packet, num_bytes);
	//for (int ii=0;ii<k; ii++)  Serial.println(packet[ii], DEC);
	//Serial.print("Number of bytes sent = ");
	Serial2.write(packet, k);
}

void IMUsetColor(float red, float green, float blue) {
	color[0].fval = red;  color[1].fval = green; color[2].fval = blue;
	IMU_Send_command(238, 3, NOT_BYTE, color);
}

void IMUreadColor() {
	IMU_Rd_command(239, 3, NOT_BYTE, color1);
	Serial.print("Color set is = "); Serial.print(color1[0].fval); Serial.print(" ");
	Serial.print(color1[1].fval); Serial.print(" "); Serial.println(color1[2].fval);
}

void tare() {
	IMU_Send_command(96, 0, NOT_BYTE, &dummy);
}

void IMUstop_streaming() {
	IMU_Send_command(86, 0, IS_BYTE, (union SPIdata*) &dummy);
}

void IMUstart_streaming() {
	IMU_Send_command(85, 0, IS_BYTE, (union SPIdata*) &dummy);
}

// --------------------------------------------------- Initialize YEI ------------------------------------------------------------

void init_YEI()
{
	// Serial Port Configuration for YEI
	Serial2.begin(BAUD);   // :231,460800
	delay(10);

	// Clear Serial 2 for IMU Read
	Serial2.flush();
	while (Serial2.available())
		Serial2.read();
	delay(10);

	// Read IMU Baud Rate
	IMU_Rd_command(232, 1, NOT_BYTE, &dummy);
	 Serial.print("IMU Baudrate:");
	 Serial.println(dummy.ival);
	delay(10);

	//Auto-Calibrate IMU gyroscopes
	//IMU_Gyro_Calibrate();
	//delay(5000);

	//Configure IMU LED Color
	IMUsetColor(1, 0, 0);
	// IMUreadColor();
	IMU_Rd_command(43, 1, NOT_BYTE, &temp);

	// Get Some Variables
	 Serial.print("Temperature ");
	 Serial.println(temp.fval);
	
	 Serial.println("Setting up streaming");
	IMU_Send_command(80, 8, IS_BYTE, (union SPIdata*) streamChannel);
	delay(1);
	IMU_Rd_command(81, 8, IS_BYTE, (union SPIdata*) streamChannel1);


	for (int k = 0; k < 7; k++)
	{
		Serial.print(streamChannel1[k]); Serial.print(" ");
	}
	 Serial.println(streamChannel1[7]);
	IMU_Send_command(221, 1, NOT_BYTE, (union SPIdata*) &header);
	IMU_Send_command(82, 3, NOT_BYTE, (union SPIdata*) streamTimes);
	IMU_Rd_command(83, 3, NOT_BYTE, streamTimes1);
	
	for (int k = 0; k < 2; k++) {
		Serial.print(streamTimes1[k].ival); Serial.print(" ");
	}
	Serial.println(streamTimes1[2].ival);

	tare(); 
	
	delay(1);

	Serial.println("Initialized YEI IMU.");

	IMU_Send_command(95, 1, NOT_BYTE, (union SPIdata*) &init_timestamp); // reset timestamp
	IMUstart_streaming();       // Start streaming
	delay(100);

	// Clear Serial2 for IMU Read
	Serial2.flush();
	while (Serial2.available())
		Serial2.read();

}

// --------------------------------------------------- Update YEI ------------------------------------------------------------

void update_YEI()
{
	avail = Serial2.available();
	if (avail >= IMU_STREAM_SIZE)
	{
		updateimu_flag = true;
		isrIMU();
	}
	else
		updateimu_flag = false;

}

// --------------------------------------------------- IMU Interrupt Service ------------------------------------------------------------
void isrIMU()
{
	IMU_Rd_Channel(&timestamp, 1);
	IMU_Rd_Channel(quat, 4);
	IMU_Rd_Channel(mag, 3);
	IMU_Rd_Channel(acc_raw, 3);
	IMU_Rd_Channel(&temp, 1);
	IMU_Rd_Channel(yei_gyro, 3);

	nav.data.q_yei[0] = quat[0].fval;
	nav.data.q_yei[1] = quat[1].fval;
	nav.data.q_yei[2] = quat[2].fval;
	nav.data.q_yei[3] = quat[3].fval;

	imu.imu_data.mag_yei[0] = mag[0].fval - yei_mag_biasx;
	imu.imu_data.mag_yei[1] = mag[1].fval - yei_mag_biasy;
	imu.imu_data.mag_yei[2] = mag[2].fval - yei_mag_biasz;

	imu.imu_data.acc_yei[0] = acc_raw[0].fval;
	imu.imu_data.acc_yei[1] = acc_raw[1].fval;
	imu.imu_data.acc_yei[2] = acc_raw[2].fval;

	imu.imu_data.omega_yei[0] = yei_gyro[0].fval;
	imu.imu_data.omega_yei[1] = yei_gyro[1].fval;
	imu.imu_data.omega_yei[2] = yei_gyro[2].fval;
}
