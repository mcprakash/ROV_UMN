/*********************************************************************
SO3-PnP

Source code to enable landmark color selection by User
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

// <------------------------------------------------ Camera Libraries ---------------------------------------------->
#include "camera.h"
#include "graphics.h"

// <------------------------------------------------ Eigen Libraries ---------------------------------------------->
#include <eigen3/Eigen/Dense>

// <------------------------------------------------ OpenCV Libraries ---------------------------------------------->
#include <opencv2/opencv.hpp>

// <------------------------------------------------ SO3 Functions ---------------------------------------------->
#include "so3.h"

// <------------------------------------------------ Mouse-Click Functions ---------------------------------------------->
#include "mouse_click.h"

// <------------------------------------------------ Include Namespaces ---------------------------------------------->
using namespace std;
using namespace cv;
using namespace Eigen;

#define W_img 1280
#define H_img 720

char dbg_val = 0;
Mat img_dbg(H_img,W_img,CV_8UC3);

Scalar orange(255, 255, 0);

char hsv_red[3], hsv_green[3], hsv_blue[3], hsv_orange[3];

Vec3b rgb_val;

bool mouse_click_flag=1;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	//EVENT_FLAG_CTRLKEY; EVENT_FLAG_SHIFTKEY;

	if (flags == (EVENT_FLAG_LBUTTON))
	{
		cout<< "Pressed Left Mouse \t db_val=" << (int)dbg_val <<endl;
		switch (dbg_val)
		{
		case 0:
			circle(img_dbg, Point(x, y), 5, red, -1, 8, 0);
			glReadPixels(x,H_img-y,1,1, GL_RGB, GL_UNSIGNED_BYTE, hsv_red);
			dbg_val++;
			break;
		case 1:
			circle(img_dbg, Point(x, y), 5, green, -1, 8, 0);
			glReadPixels(x,H_img-y,1,1, GL_RGB, GL_UNSIGNED_BYTE, hsv_green);
			dbg_val++;
			break;
		case 2:
			circle(img_dbg, Point(x, y), 5, blue, -1, 8, 0);
			glReadPixels(x,H_img-y,1,1, GL_RGB, GL_UNSIGNED_BYTE, hsv_blue);
			dbg_val++;
			break;
		case 3:
			circle(img_dbg, Point(x, y), 5, orange, -1, 8, 0);
			glReadPixels(x,H_img-y,1,1, GL_RGB, GL_UNSIGNED_BYTE, hsv_orange);
			dbg_val++;
			break;
		}

	}
	else if (flags == EVENT_FLAG_RBUTTON )
	{
		dbg_val = 0;
		img_dbg.setTo(white);
		cout<<"Pressed Right Mouse " <<endl;
		
	}

	else if(flags == EVENT_FLAG_MBUTTON)
	{
		mouse_click_flag=0;
		
		
		cout << "red_val=" << (int) hsv_red[0] << "\t" << (int)hsv_red[1] << "\t" << (int)hsv_red[2] << endl;
		cout << "green_val=" << (int)hsv_green[0] << "\t" << (int)hsv_green[1] << "\t" << (int)hsv_green[2] << endl;
		cout << "blue_val=" << (int)hsv_blue[0] << "\t" << (int)hsv_blue[1] << "\t" << (int)hsv_blue[2] << endl;
		cout << "orange_val=" << (int)hsv_orange[0] << "\t" << (int)hsv_orange[1] << "\t" << (int)hsv_orange[2] << endl;
		
	}
	
	waitKey(1);
}


void init_mouse_click()
{
	img_dbg.setTo(white);
}

void mouse_click()
{
	
	//Create a window
	namedWindow("dbg_win", 1);
	moveWindow("dbg_win",250,110);

	//set the callback function for any mouse event
	setMouseCallback("dbg_win", CallBackFunc, NULL);
	
	//show the image
	imshow("dbg_win", img_dbg);

	// Wait until user press some key
	if(waitKey(1)==27)
	{
		destroyWindow("dbg_win");
	}
	
}
