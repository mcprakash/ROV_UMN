/*********************************************************************
SO3-PnP

Source code to detect landmark (object) using OpenCV Library
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

// <------------------------------------------------ Include Namespaces ---------------------------------------------->
using namespace std;
using namespace cv;
using namespace Eigen;

// <-----------OpenCV Mat Variables------------->
Mat cvrgb(H_img,W_img,CV_8UC3), cvgray(H_img,W_img,CV_8UC1); 		 // Mat to hold main RGB and gray images
//Mat cvdisp(H_img/4, W_img/4, CV_8UC3), cvdisph(H_img,W_img,CV_8UC3); // Mat to display detected centers
Mat cvtemp(H_img,W_img,CV_8UC1);
Mat cvroi;															 // Mat to get ROI 

// <-----------OpenCV Color Variables ------------->
Scalar green(0, 255, 0);
Scalar blue(255, 0, 0);
Scalar red(0, 0, 255);
Scalar white(255, 255, 255);
Scalar black(0,0,0);
Scalar orange(255, 255, 0);

// <-----------OpenCV Contour Variables ------------->
MemStorage storage;	
vector< vector <Point> > contours;
vector<Vec4i> hierarchy;
vector<Point> contours_poly;
Moments mu;
Vec3b color;
float tempx,tempy;

int pt_index=0, pt_cnt=0;
float circ_perim=0, circ_area=0;
float check_val=0;
float circ_area_check=750;


// <-----------ROI Variables ------------->
bool flag_detect=false, isROI=false;						// Flags to see if ROI is to be selected
int x_roi, y_roi, h_roi, w_roi, xb_roi, yb_roi;				// ROI co-ordinates and dimensions
int ROI_GAP_X = 50, ROI_GAP_Y= 50;						    // MAX_ROI_WIDTH, MAX ROI HEIGHT
int w_cols=W_img, h_rows=H_img;								// ROI width and Height 
MatrixXf::Index minROIIndex[2], maxROIIndex[2];					// ROI min and max pixel indices
VectorXf minROIVal(2), maxROIVal(2);						// ROI min and max pixel co-ordinates
int debug_val=0; 											// Debug Value to check for ROI
MatrixXf pt_img_temp(4,2);

// <-----------Mouse Click Variables ------------->
char dbg_val = 0;
Mat img_dbg(H_img,W_img,CV_8UC3);							// Mat for debug window (choosing points) 
char rgb_red[3], rgb_green[3], rgb_blue[3], rgb_orange[3];  // Int Lab Standard Colors
float lab_red[3], lab_green[3], lab_blue[3], lab_orange[3]; // Lab standard colors
bool mouse_click_flag=1;									// Flag for debug window (user choices)								

vector<Vec3f> circles; // Hough circles detection

struct timespec t8,t9;

bool opencv_detect()
{	
	//cout<<"flag="<<flag_detect<<endl;

	namedWindow("img",WINDOW_AUTOSIZE);
	//namedWindow("temp",WINDOW_AUTOSIZE);
		// <----------------Marker Detection in OpenCV ----------------> 
		
		//flag_detect=false;
		
		if (flag_detect == false)
		{
			isROI = false;
			w_cols=W_img;
			h_rows=H_img;
			cvtColor(cvrgb, cvgray, CV_BGR2GRAY);
			//cout<<"No ROI"<<endl;
		}
		else
		{
			flag_detect = false;
			isROI = true;

			minROIVal(0)=pt_img.col(0).minCoeff(&minROIIndex[0]);
			minROIVal(1)=pt_img.col(1).minCoeff(&minROIIndex[1]);
			
			maxROIVal(0)=pt_img.col(0).maxCoeff(&maxROIIndex[0]);
			maxROIVal(1)=pt_img.col(1).maxCoeff(&maxROIIndex[1]);
			
			//cout<<"Reached ROI Case"<<endl;

			if (minROIVal(0) < ROI_GAP_X)
			{
				x_roi = 0;
				debug_val=1;
				//cout<<debug_val<<endl;
			}
				
			else
			{
				x_roi = (int)minROIVal(0) - ROI_GAP_X;
				debug_val=2;
				//cout<<debug_val<<endl;
			}

			if (maxROIVal(0) > W_img - ROI_GAP_X)
			{
				xb_roi = W_img;
				debug_val=3;
				//cout<<debug_val<<endl;
			}
			else
			{
				xb_roi = (int)maxROIVal(0) + ROI_GAP_X;
				debug_val=4;
				//cout<<debug_val<<endl;
			}

			if (minROIVal(1) < ROI_GAP_Y)
			{
				y_roi = 0;
				debug_val=5;
				//cout<<debug_val<<endl;
			}
			else
			{
				y_roi = (int)minROIVal(1) - ROI_GAP_Y;
				debug_val=6;
				//cout<<debug_val<<endl;
			}

			if (maxROIVal(1) > H_img - ROI_GAP_Y)
			{
				yb_roi = H_img;
				debug_val=7;
				//cout<<debug_val<<endl;
			}
			else
			{
				yb_roi = (int)maxROIVal(1) + ROI_GAP_Y;
				debug_val=8;
				//cout<<debug_val<<endl;
			}

			//cout<<"x_roi="<<x_roi<<"\ty_roi="<<y_roi<<"\txb_roi="<<xb_roi<<"\tyb_roi="<<yb_roi<<endl;
			
			cvroi = cvrgb(Rect(x_roi, y_roi, xb_roi-x_roi-1, yb_roi-y_roi-1));

			w_cols=xb_roi-x_roi;
			h_rows=yb_roi-y_roi;
			
			//cout<<pt_img<<endl;
	
			cvtColor(cvroi, cvgray, CV_BGR2GRAY);
			//cout<<"Yes ROI"<<endl;
			
		}

		//cvtemp=cvgray;
		findContours(cvgray, contours,hierarchy, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE ,Point(0, 0));

		//rectangle(cvgray,Rect(x_roi-10,y_roi-10, xb_roi-x_roi-20, yb_roi-y_roi-20),white,1,8,0);
		
		if (contours.size() < 4)
		{
			flag_detect = false;
			return 0;
		}
		
		//cout<<"Detected Contours="<<contours.size()<<"\n";
		pt_cnt=0;
		
		//cvdisp.setTo(white);
		//cvdisph.setTo(white);
		
		for (size_t i = 0; i<contours.size(); i++)		
		{
			circ_perim=arcLength(contours[i],true);			
			approxPolyDP(contours[i], contours_poly, circ_perim * 0.02, true);
			//drawContours(cvtemp,contours,i,white,2,8,hierarchy,0,Point());

			// *********************** Get Sphere in Triangle ***************************** 

			if (contours_poly.size() >=6)
			{
			 
 			  circ_area=fabs(contourArea(contours[i]));
 			  
 			  //cout<<"Entered Loop 1"<<endl;
			  
			  if(circ_area > circ_area_check)		
			  {

				check_val=circ_perim*circ_perim/4/M_PI/circ_area;
				
				if(check_val<1.75 && check_val>0.75)
				{
					
					//cout<<"Entered Loop 2"<<endl;
					mu=moments(contours[i],false);
					
					if(isROI)
					{
					  tempx = mu.m10 / mu.m00 + x_roi;
					  tempy = mu.m01 / mu.m00 + y_roi;
					  circle(cvgray,Point(tempx-x_roi,tempy-y_roi),3,white,1,8,0);

					}
					else
					{
					  tempx = mu.m10 / mu.m00;
					  tempy = mu.m01 / mu.m00;
					  circle(cvgray,Point(tempx,tempy),3,white,1,8,0);

					}

					color=cvrgb.at<Vec3b>(tempy,tempx);

					// Red Ball
					if(color[0]>=128 && color[1]<64 && color[2]<64)
					{
						//cout<<"R";
						pt_index=0;
						
						//cout<<"r\t"<<contours[i].size()<<endl;

					}
					// Green Ball
					else if(color[0]<64 && color[1]>128 && color[2]<64)
					{
						//cout<<"G";
						pt_index=1;
						
						//cout<<"g\t"<<contours[i].size()<<endl;

					}
					// Blue Ball
					else if(color[0]<64 && color[1]<64 && color[2]>128)
					{
						//cout<<"B";
						pt_index=2;
						
						//cout<<"b\t"<<contours[i].size()<<endl;

					}
					// Orange Ball
					else if(color[0]>128 && color[1]<64 && color[2]>128)
					{
						//cout<<"A";
						pt_index=3;
						
						//cout<<"p\t"<<contours[i].size()<<endl;

					}

					else 
						break;

					pt_img_temp(pt_index,0)=tempx;
					pt_img_temp(pt_index,1)=tempy;
					
					
					pt_cnt++;
					if (pt_cnt == 4)
					{
						pt_img=pt_img_temp;
						//cout<<endl;
						break;
					}
					//else 
						//return 0;

				} 
					
			  }
			}
		}

		//cout<<"pt_index="<<pt_index<<endl;

		contours_poly.clear();
		contours.clear();
		
		
		if(pt_cnt==4)
		{
		
		 findPosSO3(pt_img,R);

		 circ_area_check=1000.0*4/abs(r_cam(2));
		
		 //reproject();

		 /*
		 circle(cvdisp,Point(pt_img(0,0)/4,pt_img(0,1)/4),3,red,1,0,0);
		 circle(cvdisp,Point(pt_img(1,0)/4,pt_img(1,1)/4),3,green,1,0,0);
		 circle(cvdisp,Point(pt_img(2,0)/4,pt_img(2,1)/4),3,blue,1,0,0);
		 circle(cvdisp,Point(pt_img(3,0)/4,pt_img(3,1)/4),3,black,1,0,0);

		 line(cvdisp,Point(pt_reproject(0,0)/4,pt_reproject(0,1)/4), Point(pt_reproject(1,0)/4,pt_reproject(1,1)/4),red,1,0,0);
		 line(cvdisp,Point(pt_reproject(1,0)/4,pt_reproject(1,1)/4), Point(pt_reproject(2,0)/4,pt_reproject(2,1)/4),green,1,0,0);
		 line(cvdisp,Point(pt_reproject(2,0)/4,pt_reproject(2,1)/4), Point(pt_reproject(3,0)/4,pt_reproject(3,1)/4),blue,1,0,0);
		 line(cvdisp,Point(pt_reproject(3,0)/4,pt_reproject(3,1)/4), Point(pt_reproject(0,0)/4,pt_reproject(0,1)/4),black,1,0,0);

		 */
		 //circle(cvdisph,Point(pt_img(0,0),pt_img(0,1)),3,red,1,0,0);
		 //circle(cvdisph,Point(pt_img(1,0),pt_img(1,1)),3,green,1,0,0);
		 //circle(cvdisph,Point(pt_img(2,0),pt_img(2,1)),3,blue,1,0,0);
		 //circle(cvdisph,Point(pt_img(3,0),pt_img(3,1)),3,black,1,0,0);

		 //line(cvdisph,Point(pt_reproject(0,0),pt_reproject(0,1)), Point(pt_reproject(1,0),pt_reproject(1,1)),red,1,0,0);
		 //line(cvdisph,Point(pt_reproject(1,0),pt_reproject(1,1)), Point(pt_reproject(2,0),pt_reproject(2,1)),green,1,0,0);
		 //line(cvdisph,Point(pt_reproject(2,0),pt_reproject(2,1)), Point(pt_reproject(3,0),pt_reproject(3,1)),blue,1,0,0);
		 //line(cvdisph,Point(pt_reproject(3,0),pt_reproject(3,1)), Point(pt_reproject(0,0),pt_reproject(0,1)),black,1,0,0);

		 
		 flag_detect = true;
		 
		}
		else 
			flag_detect=false;
		
		//imshow("img",cvrgb);
		//imshow("img",cvdisp);
		//imshow("img",cvdisph);
		
		imshow("img",cvgray);
		//imshow("temp",cvtemp);
		waitKey(1);

		return 1;
		 
}

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
			glReadPixels(x,(H_img-y),1,1, GL_RGB, GL_UNSIGNED_BYTE, rgb_red);
			//rgb2lab(rgb_red, lab_red);
			rgb2rgbf(rgb_red, lab_red);
			dbg_val++;
			break;
		case 1:
			circle(img_dbg, Point(x, y), 5, green, -1, 8, 0);
			glReadPixels(x,(H_img-y),1,1, GL_RGB, GL_UNSIGNED_BYTE, rgb_green);
			//rgb2lab(rgb_green, lab_green);
			rgb2rgbf(rgb_green, lab_green);
			dbg_val++;
			break;
		case 2:
			circle(img_dbg, Point(x, y), 5, blue, -1, 8, 0);
			glReadPixels(x,(H_img-y),1,1, GL_RGB, GL_UNSIGNED_BYTE, rgb_blue);
			//rgb2lab(rgb_blue, lab_blue);
			rgb2rgbf(rgb_blue, lab_blue);
			dbg_val++;
			break;
		case 3:
			circle(img_dbg, Point(x, y), 5, orange, -1, 8, 0);
			glReadPixels(x,(H_img-y),1,1, GL_RGB, GL_UNSIGNED_BYTE, rgb_orange);
			//rgb2lab(rgb_orange, lab_orange);
			rgb2rgbf(rgb_orange,lab_orange);
			dbg_val++;
			break;
		}

	}
	else if (flags == EVENT_FLAG_RBUTTON )
	{
		dbg_val = 0;
		//img_dbg.setTo(white);
		//cout<<"Pressed Right Mouse " <<endl;
		
	}

	else if(flags == EVENT_FLAG_MBUTTON)
	{
		mouse_click_flag=0;
		
		
		cout << "red_val=" << (int) rgb_red[0] << "\t" << (int)rgb_red[1] << "\t" << (int)rgb_red[2] << endl;
		cout << "green_val=" << (int)rgb_green[0] << "\t" << (int)rgb_green[1] << "\t" << (int)rgb_green[2] << endl;
		cout << "blue_val=" << (int)rgb_blue[0] << "\t" << (int)rgb_blue[1] << "\t" << (int)rgb_blue[2] << endl;
		cout << "orange_val=" << (int)rgb_orange[0] << "\t" << (int)rgb_orange[1] << "\t" << (int)rgb_orange[2] << endl;
		
		/*
		cout << "hsv red_val=" <<  hsv_red[0] << "\t" << hsv_red[1] << "\t" << hsv_red[2] << endl;
		cout << "hsv green_val=" << hsv_green[0] << "\t" << hsv_green[1] << "\t" << hsv_green[2] << endl;
		cout << "hsv blue_val=" << hsv_blue[0] << "\t" << hsv_blue[1] << "\t" << hsv_blue[2] << endl;
		cout << "hsv orange_val=" << hsv_orange[0] << "\t" << hsv_orange[1] << "\t" << hsv_orange[2] << endl;
		*/
		
		cout << "lab red_val=" <<  lab_red[0] << "\t" << lab_red[1] << "\t" << lab_red[2] << endl;
		cout << "lab green_val=" << lab_green[0] << "\t" << lab_green[1] << "\t" << lab_green[2] << endl;
		cout << "lab blue_val=" << lab_blue[0] << "\t" << lab_blue[1] << "\t" << lab_blue[2] << endl;
		cout << "lab orange_val=" << lab_orange[0] << "\t" << lab_orange[1] << "\t" << lab_orange[2] << endl;
		
	}
	
	waitKey(10);
}


void init_opencv_fcns()
{
	cout << "Initializing OpenCV Module" << endl;
	img_dbg.setTo(white);
	pt_img_temp.setZero();
	cout << "Completed Initializing OpenCV Module"<<endl<<endl;
}

void rgb2rgbf(char src[],float dst[])
{
	dst[0] = src[0] / 255.0f;
    dst[1] = src[1] / 255.0f;
    dst[2] = src[2] / 255.0f;
}

