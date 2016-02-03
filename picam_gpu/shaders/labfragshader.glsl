##########################################################################
#  LAB Frag Shader														 #
#																		 #
#  Shader to convert RGB color space to LAB			 				     #
##########################################################################
#  Copyright(C)  <December 2015>  <Chandra P. Mangipudi					 #
#																		 #
#  This program is free software : you can redistribute it and/or modify #
#  it under the terms of the GNU General Public License as published by  # 
#  the Free Software Foundation, either version 3 of the License, or	 #
#  (at your option) any later version.									 #
#																		 #
#  This program is distributed in the hope that it will be useful,		 #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of		 #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the			 #
#  GNU General Public License for more details.							 #
#																		 #
#  You should have received a copy of the GNU General Public License	 #
#  along with this program.If not, see <http://www.gnu.org/licenses/>.	 #
##########################################################################

varying vec2 tcoord;
uniform sampler2D tex;

vec4 rgb2xyz(vec4 color) {
	float var_R = (color.r); //R from 0.0 to 255.0
	float var_G = (color.g); //G from 0.0 to 255.0
	float var_B = (color.b); //B from 0.0 to 255.0

	if (var_R > 0.04045) {
		var_R = pow(((var_R + 0.055) / 1.055), 2.4);
	} else {
		var_R = var_R / 12.92;
	}
	if (var_G > 0.04045) {
		var_G = pow(((var_G + 0.055) / 1.055), 2.4);
	} else {
		var_G = var_G / 12.92;
	}

	if (var_B > 0.04045) {
		var_B = pow(((var_B + 0.055) / 1.055), 2.4);
	} else {
		var_B = var_B / 12.92;
	}

	var_R = var_R * 100.0;
	var_G = var_G * 100.0;
	var_B = var_B * 100.0;

	//Observer. = 2.0°, Illuminant = D65
	float X = var_R * 0.4124 + var_G * 0.3576 + var_B * 0.1805;
	float Y = var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722;
	float Z = var_R * 0.0193 + var_G * 0.1192 + var_B * 0.9505;

	return vec4(X, Y, Z, color.a);
}

vec4 xyz2lab(vec4 color) {

	float ref_X = 95.047; //Observer= 2.0°, Illuminant= D65
	float ref_Y = 100.000;
	float ref_Z = 108.883;

	float var_X = color.r / ref_X;
	float var_Y = color.g / ref_Y;
	float var_Z = color.b / ref_Z;

	if (var_X > 0.008856) {
		var_X = pow(var_X, (1.0 / 3.0));
	} else {
		var_X = (7.787 * var_X) + (16.0 / 116.0);
	}
	if (var_Y > 0.008856) {
		var_Y = pow(var_Y, (1.0 / 3.0));
	} else {
		var_Y = (7.787 * var_Y) + (16.0 / 116.0);
	}
	if (var_Z > 0.008856) {
		var_Z = pow(var_Z, (1.0 / 3.0));
	} else {
		var_Z = (7.787 * var_Z) + (16.0 / 116.0);
	}

	float L;
	if(var_Y>0.008856)
	{
		  L = ((116.0 * var_Y) - 16.0)/101.0;
		}
	else
	{
	  L=903.3*var_Y/101.0;
	}

	float a = (500.0 * (var_X - var_Y))/221.0+0.5;
	float b = (200.0 * (var_Y - var_Z))/221.0+0.5;

	return vec4(L, a, b, 0.2);
}



vec4 rgb2lab(vec4 color) {
	vec4 xyz = rgb2xyz(color);
	vec4 lab = xyz2lab(xyz);
	return lab;
}


void main(void) 
{
    gl_FragColor = clamp(rgb2lab(texture2D(tex,tcoord)),vec4(0),vec4(1));
}
