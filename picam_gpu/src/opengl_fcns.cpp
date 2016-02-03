/*********************************************************************
SO3-PnP

Source code to perform kernel operations on images using GLSL shaders
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

// <------------------------------------------------ OpenCV Functions ---------------------------------------------->
#include "opencv_fcns.h"

// <------------------------------------------------ OpenGL Functions ---------------------------------------------->
#include "opengl_fcns.h"

// <------------------------------------------------ Include Namespaces ---------------------------------------------->
using namespace std;
using namespace cv;
using namespace Eigen;

// <-----------OpenGL Tex Variables ------------->
int frame_sz;
int ypitch, ysize, uvpitch, uvsize, upos, vpos;
float tdiff;

GfxTexture ytexture,utexture,vtexture,tex, gray, hsv, lab, kernel, kernelx, kernely, laplacian, undistort_img;

const void* frame_data; 
const uint8_t* data;
CCamera* cam;

void init_opengl_fcns ()
{
	cout << " Initializing OpenGL Graphics Library"<<endl;
	// <------------------------------------------------ Generate textures for image processing in OpenGL ----------------------------------------------> 

	ytexture.CreateGreyScale(MAIN_TEXTURE_WIDTH,MAIN_TEXTURE_HEIGHT);
	utexture.CreateGreyScale(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	vtexture.CreateGreyScale(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);

	hsv.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	hsv.GenerateFrameBuffer();

	gray.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	gray.GenerateFrameBuffer();

	tex.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	tex.GenerateFrameBuffer();
	
	lab.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	lab.GenerateFrameBuffer();
	
	kernel.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	kernel.GenerateFrameBuffer();
	
	kernelx.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	kernelx.GenerateFrameBuffer();
	
	kernely.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	kernely.GenerateFrameBuffer();
	
	laplacian.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	laplacian.GenerateFrameBuffer();
	
	undistort_img.CreateRGBA(MAIN_TEXTURE_WIDTH/2,MAIN_TEXTURE_HEIGHT/2);
	undistort_img.GenerateFrameBuffer();
	
	cout<<"Completed initializing OpenGL Graphics"<<endl<<endl;
}

void opengl_draw()
{
	
		// <----------------Image Processing in GPU using OpenGL----------------> 
		BeginFrame();
		
		DrawYUVTextureRect(&ytexture,&utexture,&vtexture,&tex);
		
		//DrawUndistortRect(&tex,&undistort_img);

		//DrawHSVRect(&tex,&hsv);
		
		//DrawKernelxRect(&hsv,&kernelx);
		
		DrawLABRect(&tex,&lab);
		
		DrawKernelxRect(&lab,&kernelx);
		
		DrawKernelyRect(&kernelx,&kernely);
		
		DrawTextureRect(&kernely,-1.f,1.f,1.f,-1.f,NULL);
	    
	    /*
	    Mat flipped;
	
		glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data);  
		
		flip(cvrgb,flipped,0); 
	
		cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
		imwrite("img_labgaussian.png",cvrgb);		
		*/

		DrawInRangeRect(&kernely,lab_red,lab_green,lab_blue,lab_orange,&gray);
		
		
		/*
		DrawTextureRect(&gray,-1.f,1.f,1.f,-1.f,NULL);
		
		glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data);  
		
		flip(cvrgb,flipped,0); 
	
		cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
		imwrite("img_gray.png",cvrgb);
		*/
		

		DrawKernelxRect(&gray,&kernelx);
		
		DrawKernelyRect(&kernelx,&kernely);
		
		DrawTextureRect(&kernely,-1.f,-1.f,1.f,1.f,NULL);
		
		//DrawLaplacianRect(&gray, &laplacian);
		
		//DrawTextureRect(&gray,-1.f,-1.f,1.f,1.f,NULL);
		
		glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data);  
		
		DrawTextureRect(&kernely,-1.f,1.f,1.f,-1.f,NULL);
		
		/*
		flip(cvrgb,flipped,0); 
	
		cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
		imwrite("img_graygaussian.png",cvrgb);
		
		
		DrawTextureRect(&tex,-1.f,1.f,1.f,-1.f,NULL);
		
		glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data);  
		
		flip(cvrgb,flipped,0); 
	
		cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
		imwrite("img_tex.png",cvrgb);
		
		
		
		DrawTextureRect(&lab,-1.f,1.f,1.f,-1.f,NULL);
		
		glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data);  
		
		flip(cvrgb,flipped,0); 
	
		cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
		imwrite("img_lab.png",cvrgb);

		exit(0);*/
		
		if(c=='a')
		{
		
		DrawTextureRect(&kernely,-1.f,1.f,1.f,-1.f,NULL);
		
		glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data);  
		
		Mat flipped;
	
		flip(cvrgb,flipped,0); 
	
		cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
		imwrite("img_labgaussian.png",cvrgb);
	
		exit(0);
		}
		
		EndFrame();
	
}

void cam_update()
{
	
	// <------------------------------------------------ Wait for new image to appear ----------------------------------------------> 
		while(!cam->BeginReadFrame(0,frame_data,frame_sz)) {};
		
		digitalWrite(PIN_TCAP, HIGH) ;
		
		// <----------------lock the chosen frame buffer, and copy it directly into the corresponding open gl texture ----------------> 
			data = (const uint8_t*)frame_data;
			ypitch = MAIN_TEXTURE_WIDTH;
			ysize = ypitch*MAIN_TEXTURE_HEIGHT;
			uvpitch = MAIN_TEXTURE_WIDTH/2;
			uvsize = uvpitch*MAIN_TEXTURE_HEIGHT/2;
			upos = ysize;
			vpos = upos+uvsize;
			ytexture.SetPixels(data);
			utexture.SetPixels(data+upos);
			vtexture.SetPixels(data+vpos);
			cam->EndReadFrame(0);

}

void opengl_draw_rgb()
{
	BeginFrame();
	
	DrawYUVTextureRect(&ytexture,&utexture,&vtexture,&tex);
	
	DrawTextureRect(&tex,-1.f,1.f,1.f,-1.f,NULL);
	
	glReadPixels(0,0,cvrgb.cols,cvrgb.rows, GL_RGB, GL_UNSIGNED_BYTE, cvrgb.data); 
	
	//DrawHSVRect(&tex,&hsv);
		
	//DrawKernelxRect(&hsv,&kernelx);
		
	DrawLABRect(&tex,&lab);
		
	DrawKernelxRect(&lab,&kernelx);
		
	DrawKernelyRect(&kernelx,&kernely);
         
	DrawTextureRect(&kernely,-1.f,1.f,1.f,-1.f,NULL);
	
	//DrawLABRect(&tex,&lab);

	//DrawTextureRect(&lab,-1.f,1.f,1.f,-1.f,NULL);
	
	
	Mat flipped;
	
	flip(cvrgb,flipped,0); 
	
	cvtColor(flipped, cvrgb, CV_RGB2BGR);
	
	//imwrite("img_lab.png",cvrgb);
	
	img_dbg=cvrgb;
	
	EndFrame();
	
}
