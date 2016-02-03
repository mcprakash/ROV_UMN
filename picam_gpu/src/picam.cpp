/*********************************************************************
SO3-PnP

Main Function - Multi-threaded

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

// <------------------------------------------------ Common Libraries ---------------------------------------------->
#include <stdio.h>
#include <unistd.h>
#include <curses.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>

// <------------------------------------------------ Camera Libraries ---------------------------------------------->
#include "camera.h"
#include "graphics.h"

// <------------------------------------------------ Eigen Libraries ---------------------------------------------->
#include <eigen3/Eigen/Dense>

// <------------------------------------------------ OpenCV Libraries ---------------------------------------------->
#include <opencv2/opencv.hpp>

// <------------------------------------------------ SO3 Functions ---------------------------------------------->
#include "so3.h"

// <------------------------------------------------ OpenCV Functions ---------------------------------------------->
#include "opencv_fcns.h"

// <------------------------------------------------ OpenGL Functions ---------------------------------------------->
#include "opengl_fcns.h"

// <------------------------------------------------ Wifi UDP Functions ---------------------------------------------->
#include "wifiudp.h"

// <------------------------------------------------ Include Namespaces ---------------------------------------------->
using namespace std;
using namespace cv;
using namespace Eigen;


#define FPS 10
#define DELAY_CAM 5

union float_char cam_data;

// <------------------------------------------------ Variables ---------------------------------------------->

// <-----------Loop Variables------------->
int i=0,j=0;
int frame_cnt=0;
char c;
int term_val=0;

// <------------ Variable to enable user to select points on image ------------------------>

bool dbg_sel=1;

// <------------ Variable to enable mutex lock/unlock ------------------------>
pthread_mutex_t cam_data_mutex;

// <------------ Timer Variables ------------------------>
struct timespec now, last, t1,t2,t3,t4,t5, t6;


// <------------------------------------------------ Main Function ---------------------------------------------->

void *VISION_MAIN( void *dummy)
{

	// <------------------------------------------------ Define OpenCV windows ---------------------------------------------->
	//namedWindow("img",1);
	//waitKey(1);
	
	// <------------------------------------------------ Initialize Camera and graphics ----------------------------------------------> 
	InitGraphics();
	cam = StartCamera(MAIN_TEXTURE_WIDTH, MAIN_TEXTURE_HEIGHT,FPS,1,false);
	// <------------------------------------------------ Initialize SO3 Functions ----------------------------------------------> 
	init_SO3();
	
	// <------------------------------------------------ Initialize OpenGL Functions ----------------------------------------------> 
	init_opengl_fcns();
	
	// <------------------------------------------------ Initialize OpenCV Functions ----------------------------------------------> 
	init_opencv_fcns();
	
	// <------------------------------------------------ Initialize Wifi UDP module ----------------------------------------------> 
	init_wifiudp();
	
	sleep(DELAY_CAM);
	
	// <------------------------------------------------ Debug module to select colors ----------------------------------------------> 
	
	if(dbg_sel==true)
	{
		
		namedWindow("dbg_win", 1);
		moveWindow("dbg_win",250,110);

		//CallBack function for debug window
		setMouseCallback("dbg_win", CallBackFunc, NULL);
		
		//Debug window for point click
		imshow("dbg_win", img_dbg);
		waitKey(10);
		
		while(mouse_click_flag)
		{
			cam_update();
			opengl_draw_rgb();
			imshow("dbg_win", img_dbg);
			c=waitKey(100);
			if(c==27)
				break;	
		}
		
		destroyWindow("dbg_win");
		//exit(0);
		
		cam_update();
		opengl_draw();
		
		char ch_temp;
		cout<<"Do you want to store this marker color? (y/n)"<<endl;
		cin>>ch_temp;
		
		if(ch_temp=='y')
		{
			ofstream f_marker;
			f_marker.open("cam_calib/markercolor.txt");
		
			for(int i=0; i<3; i++)
			{
			f_marker<<lab_red[i]<<lab_green[i] << lab_blue[i]<<lab_orange[i]<<endl;
			}
		
			f_marker.close();
		}

	}
	else
	{
		ifstream f_marker;
		f_marker.open("cam_calib/markercolor.txt");
		
		for(int i=0; i<3; i++)
		{
		  f_marker>>lab_red[i]>>lab_green[i] >> lab_blue[i]>>lab_orange[i];
		}
		
	    f_marker.close();
	    
	    cout<<"Reading Marker Point Color data from file"<<endl;
	    
	    cout << "lab red_val=" <<  lab_red[0] << "\t" << lab_red[1] << "\t" << lab_red[2] << endl;
		cout << "lab green_val=" << lab_green[0] << "\t" << lab_green[1] << "\t" << lab_green[2] << endl;
		cout << "lab blue_val=" << lab_blue[0] << "\t" << lab_blue[1] << "\t" << lab_blue[2] << endl;
		cout << "lab orange_val=" << lab_orange[0] << "\t" << lab_orange[1] << "\t" << lab_orange[2] << endl;
		
		// Set Lab Filter values to static ones
		
		//lab_red[0]=0.596078; lab_red[1]=0.454902; lab_red[2]=0.709804;
		//lab_green[0]=0.501961; lab_green[1]=0.34902; lab_green[2]=0.576471;
		//lab_blue[0]=0.447059; lab_blue[1]=0.384314; lab_blue[2]=0.509804;
		//lab_orange[0]=0.533333; lab_orange[1]=0.694118; lab_orange[2]=0.54902;
		
		//lab_red[0]=0.596078; lab_red[1]=0.454902; lab_red[2]=0.709804;
		//lab_green[0]=0.501961; lab_green[1]=0.34902; lab_green[2]=0.639216;
		//lab_blue[0]=0.447059; lab_blue[1]=0.384314; lab_blue[2]=0.529412;
		//lab_orange[0]=0.533333; lab_orange[1]=0.701961; lab_orange[2]=0.596078;
		
		// Static Values in ME3130 Lab with Gaussian Blur Filter
		//lab_red[0]=0.596078; lab_red[1]=0.584314; lab_red[2]=0.698038;
		//lab_green[0]=0.501961; lab_green[1]=0.34902; lab_green[2]=0.639216;
		//lab_blue[0]=0.447059; lab_blue[1]=0.384314; lab_blue[2]=0.529412;
		//lab_orange[0]=0.533333; lab_orange[1]=0.701961; lab_orange[2]=0.596078;
		
		
		// Static Values in ME3130 Lab without Gaussian Blur Filter
		/*
		lab_red[0]=0.364195; lab_red[1]=0.431968; lab_red[2]=0.779451;
		lab_green[0]=0.289244; lab_green[1]=0.299753; lab_green[2]=0.577866;
		lab_blue[0]=0.278405; lab_blue[1]=0.389292; lab_blue[2]=0.445547;
		lab_orange[0]=0.4481; lab_orange[1]=0.6634; lab_orange[2]=0.5915;
		*/
		// Violet Ball Static Colors at ME3130 Lab
		//lab_orange[0]=0.134839; lab_orange[1]=0.501032; lab_orange[2]=0.494114;
		
		
	}
	// <------------------------------------------------ Run program till user presses ESC or certain frames pass by ----------------------------------------------> 
	printf("Running frame loop\n");
	
	
	while(1)
	{
		
		frame_cnt++;
		
		cam_update();
		
		// <----------------Draw Processed image in OpenGL window ----------------> 
		
		opengl_draw();
		digitalWrite (PIN_TCAP, LOW) ;	
		digitalWrite (PIN_TPROC, LOW) ;
			
		// <----------------Find Ball Co-ordinates ----------------> 
		
		opencv_detect();
		
		if(flag_detect==true)
			{
				digitalWrite (PIN_TPROC, HIGH);	
				ardudp_send();
			}
		
		// <----------------Send data t0 Arduino for Fusion ----------------->
		//clock_gettime(CLOCK_REALTIME,&t2);
		//clock_gettime(CLOCK_REALTIME,&t1)
		/*
		
		double diff =(t2.tv_sec-t1.tv_sec)*1000+(t2.tv_nsec-t1.tv_nsec)/1000000.0f;
	
		cout<<"t1="<<diff<<endl;
		
		*/
		
	}
	
	return &term_val;
	
}


void *IMU_MAIN( void *dummy)
{
	while(1)
	{
		ardudp_recv();
		wifiudp_send();
	}
	return &term_val;
}

int main()
{
	// <------------------------------------------------ Initialize Wiring Pi Module ----------------------------------------------> 
	
	wiringPiSetup () ;
    pinMode (PIN_TCAP, OUTPUT) ;
    pinMode (PIN_TPROC, OUTPUT) ;
    digitalWrite (PIN_TCAP, LOW) ;	
    digitalWrite (PIN_TPROC, LOW) ;	
    
	clock_gettime(CLOCK_REALTIME,&last);
	
	//<------------------ User  Choices ------------------------->
	
	while(1)
	{
		cout<<"Do you want to click points on marker or use previous colors ? [y/n]"<<endl;
	
		char ch_temp;
	
		cin>>ch_temp;
	
		if(ch_temp=='y')
		{
			dbg_sel=true;
			break;
		}
	
		else if(ch_temp=='n')
		{
			dbg_sel=false;
			break;
		}
		else
		{
			cout<<"Please choose again"<<endl<<endl;
		}
	}
	
	//*********************** Thread Programming *****************************

	pthread_t imu_thread, vision_thread;
	printf("In main: creating threads for UDP and Vision \n");
	int err_no,dummy=0;
	
	// Create Vision Thread
	err_no = pthread_create(&vision_thread, NULL, VISION_MAIN, (void *)(intptr_t)dummy);
	if (err_no)
	{
		printf("Error in Creating UPD Thread with Value=%d \r\n", err_no);
		return(-1);
	}
	else
	{
		printf("Vision Thread Created \r\n");
	}
	
	sleep(1);
	// Create IMU Thread
	err_no = pthread_create(&imu_thread, NULL, IMU_MAIN, (void *)(intptr_t)dummy);
	if (err_no)
	{
		printf("Error in Creating UPD Thread with Value=%d \r\n", err_no);
		return(-1);
	}
	else
	{
		printf("IMU Thread Created \r\n");
	}

	// Run Main for Specific Time or Infinitely
	sleep(60000);
	
	
	clock_gettime(CLOCK_REALTIME,&now);
		
	double diff =(now.tv_sec-last.tv_sec)+(now.tv_nsec-last.tv_nsec)/1000000000.0f - DELAY_CAM;
	
	cout <<"frames="<<frame_cnt<<endl;
	cout<<"ts="<<diff<<endl;
	
	double fps1= ((double)frame_cnt)/diff;
	
	cout<<"fps="<<fps1<<endl;
	
	// <------------------------------------------------ Finish Program ---------------------------------------------->

	StopCamera();
	
	close_wifiudp();

	endwin();
	
	
	// Exit from Main
	printf("Exiting Main\n");
	return(0);


}
