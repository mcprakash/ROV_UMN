/*********************************************************************
SO3-PnP

Header file to perform kernel operations on images using GLSL shaders
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

#ifndef opengl_fcns_lib
#define opengl_fcns_lib

// <------------------------------------------------ Wiring Pi Library ---------------------------------------------->
#include <wiringPi.h>
#define PIN_TCAP 0
#define PIN_TPROC 1

// <-----------OpenGL Tex Variables ------------->
extern int frame_sz;
extern int ypitch, ysize, uvpitch, uvsize, upos, vpos;
extern float tdiff;
extern char c;

extern const void* frame_data; 
extern const uint8_t* data;
extern CCamera* cam;

void init_opengl_fcns();

void opengl_draw();

void cam_update();

void opengl_draw_rgb();

#endif
