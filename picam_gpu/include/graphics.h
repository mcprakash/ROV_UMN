/*********************************************************************
SO3-PnP

Header file to implement shaders on Raspberry-Pi
**********************************************************************
Copyright(C)  <December 2015>  <Chandra P. Mangipudi

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

#pragma once

#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include "lodepng.h"

// <------------------------------------------------ Camera Properties ---------------------------------------------->
#define MAIN_TEXTURE_WIDTH 1280
#define MAIN_TEXTURE_HEIGHT 720

// <------------------------------------------------ Camera Functions ---------------------------------------------->
void InitGraphics();
void ReleaseGraphics();
void BeginFrame();
void EndFrame();

// <------------------------------------------------ Classes for convenience ---------------------------------------------->
class GfxShader
{
	GLchar* Src;
	GLuint Id;
	GLuint GlShaderType;

public:

	GfxShader() : Src(NULL), Id(0), GlShaderType(0) {}
	~GfxShader() { if(Src) delete[] Src; }

	bool LoadVertexShader(const char* filename);
	bool LoadFragmentShader(const char* filename);
	GLuint GetId() { return Id; }
};

class GfxProgram
{
	GfxShader* VertexShader;
	GfxShader* FragmentShader;
	GLuint Id;

public:

	GfxProgram() {}
	~GfxProgram() {}

	bool Create(GfxShader* vertex_shader, GfxShader* fragment_shader);
	GLuint GetId() { return Id; }
};

class GfxTexture
{
	int Width;
	int Height;
	GLuint Id;
	bool IsRGBA;

	GLuint FramebufferId;
public:

	GfxTexture() : Width(0), Height(0), Id(0), FramebufferId(0) {}
	~GfxTexture() {}

	bool CreateRGBA(int width, int height, const void* data = NULL);
	bool CreateGreyScale(int width, int height, const void* data = NULL);
	bool GenerateFrameBuffer();
	void SetPixels(const void* data);
	GLuint GetId() { return Id; }
	GLuint GetFramebufferId() { return FramebufferId; }
	int GetWidth() {return Width;}
	int GetHeight() {return Height;}
};

// <------------------------------------------------ Texture Functions ---------------------------------------------->
void DrawTextureRect(GfxTexture* texture, float x0, float y0, float x1, float y1, GfxTexture* render_target);
void DrawYUVTextureRect(GfxTexture* ytexture, GfxTexture* utexture, GfxTexture* vtexture, GfxTexture* render_target);
void DrawHSVRect(GfxTexture* texture, GfxTexture* render_target);
void DrawLABRect(GfxTexture* texture, GfxTexture* render_target);
void DrawInRangeRect(GfxTexture* texture, float hsv_red[3], float hsv_green[3], float hsv_blue[3], float hsv_orange[3],GfxTexture* render_target);
void DrawKernelRect(GfxTexture* texture, GfxTexture* render_target);
void DrawKernelxRect(GfxTexture* texture, GfxTexture* render_target);
void DrawKernelyRect(GfxTexture* texture, GfxTexture* render_target);
void DrawLaplacianRect(GfxTexture* texture, GfxTexture* render_target);
void DrawUndistortRect(GfxTexture* texture, GfxTexture* render_target);


