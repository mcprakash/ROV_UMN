/*********************************************************************
SO3-PnP

Header file to enable landmark color selection by User
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

#ifndef mouse_click_lib

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

inline float min_val(float x, float y) { return x <= y ? x : y; }
inline float max_val(float x, float y) { return x <= y ? y : x; }

extern Scalar red, blue,green,white;

extern char hsv_red[], hsv_green[], hsv_blue[], hsv_orange[];

extern Mat cvrgb;

extern int i;

extern bool mouse_click_flag;

void mouse_click();

void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void init_mouse_click();

#endif
