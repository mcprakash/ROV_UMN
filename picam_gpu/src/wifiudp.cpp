/*********************************************************************
SO3-PnP

Wifi-Udp protocol to transmit data to PC/Laptop
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

//UDPClient.c

/*
* gcc -o client UDPClient.c
* ./client <server-ip>
*/

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "so3.h"
#include "wifiudp.h"

#define NAV_DATALEN 28*4
#define IMU_DATALEN 29*4
#define CAM_DATALEN 8*4
#define BUFLEN NAV_DATALEN+IMU_DATALEN+CAM_DATALEN
#define PC_PORT 6500
#define ARD_PORT 8888
#define RPI_PORT 7500

//const char* pc_ip="10.0.0.100";
const char* pc_ip="192.168.2.100";
const char* ard_ip="10.0.0.10";

void err(const char *s)
{
	perror(s);
	exit(1);
}

struct sockaddr_in wifi_addr, ard_addr, rpi_addr, cli_addr;
int sock_ard, sock_pc;
socklen_t slen, cli_len;
char buf[BUFLEN];

void init_wifiudp()
{
	printf("Initializing UDP Module \r\n");
	slen = sizeof(wifi_addr);
	
	// <-------------------- Create Sockets ------------------->
	if ((sock_ard = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		err("socket");
	else
		printf("Socket for Arduino created successfully\n");
		
	if ((sock_pc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
		err("socket");
	else
		printf("Socket for Laptop/PC created successfully\n");
		
	// <-------------------- RPI Configuration ------------------->
	bzero(&rpi_addr, sizeof(rpi_addr));
	rpi_addr.sin_family = AF_INET;
	rpi_addr.sin_port = htons(RPI_PORT);
	if (bind(sock_ard, (struct sockaddr*) &rpi_addr, sizeof(rpi_addr)) == -1)
		err("bind");
	else
		printf("Server for Arduino Receive: bind() successful\n");
		
    // <-------------------- Wifi Configuration ------------------->
	bzero(&wifi_addr, sizeof(wifi_addr));
	wifi_addr.sin_family = AF_INET;
	wifi_addr.sin_port = htons(PC_PORT);
	if (inet_aton(pc_ip, &wifi_addr.sin_addr) == 0)
	{
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}
	
	// <-------------------- Arduino Configuration ----------------->
	bzero(&ard_addr, sizeof(ard_addr));
	ard_addr.sin_family = AF_INET;
	ard_addr.sin_port = htons(ARD_PORT);
	if (inet_aton(ard_ip, &ard_addr.sin_addr) == 0)
	{
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}
	
	for(int i=0;i<BUFLEN;i++)
		buf[i]=0;
		
	printf("Completed Initializing UDP Module \r\n \r\n");
}

void wifiudp_send()
{
		if (sendto(sock_pc, buf, BUFLEN, 0, (struct sockaddr*)&wifi_addr, slen) == -1)
			err("wifi sendto()");
}

void ardudp_send()
{
	if (sendto(sock_ard, cam_data.buf, sizeof(cam_data.buf), 0, (struct sockaddr*)&ard_addr, slen) == -1)
			err("arduino sendto()");
}

void ardudp_recv()
{
	if(recvfrom(sock_ard,buf,BUFLEN,0,(struct sockaddr*)&cli_addr, &cli_len)== -1)
			err("recvfrom()");
	
	//cout<<"Received IMU Data"<<endl;
	// Copy Camera Data into buf to send to Laptop/PC		
	//pthread_mutex_lock(&cam_data_mutex);
	
	//for(int i=IMU_DATALEN; i <BUFLEN;i++)
	//{
	//	buf[i]=cam_data.buf[i-IMU_DATALEN];
	//}
	
	//pthread_mutex_unlock(&cam_data_mutex);
}
			

void close_wifiudp()
{
	close(sock_ard);
}
