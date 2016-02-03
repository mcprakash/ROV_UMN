/*********************************************************************
SO3-PnP

Header file to detect landmark (object) using OpenCV Library
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

#ifndef opencv_fcns_lib
#define opencv_fcns_lib

// <------------------------------------------------ OpenCV Libraries ---------------------------------------------->
#include <opencv2/opencv.hpp>


#define W_img MAIN_TEXTURE_WIDTH
#define H_img MAIN_TEXTURE_HEIGHT
#define min_f(a, b, c)  (fminf(a, fminf(b, c)))
#define max_f(a, b, c)  (fmaxf(a, fmaxf(b, c)))

using namespace cv;


// <-----------Inline Functions------------->
inline float min_val(float x, float y) { return x <= y ? x : y; }
inline float max_val(float x, float y) { return x <= y ? y : x; }

// <-----------Logic Variables------------->
extern bool flag_detect;

// <-----------Loop Variables------------->
extern int i,j;
extern int img_cnt;

// <-----------OpenCV Mat Variables------------->
extern Mat cvrgb, cvgray;
extern Mat cvdisp;
extern Mat cvroi;

// <-----------Mouse Click Variables ------------->
extern Mat img_dbg;

extern float lab_red[], lab_green[], lab_blue[], lab_orange[];
extern bool mouse_click_flag;

// <-----------Function Declarations ------------->
void CallBackFunc(int event, int x, int y, int flags, void* userdata);

void init_opencv_fcns();
bool opencv_detect();

void rgb2rgbf(char[], float[]);

#endif
