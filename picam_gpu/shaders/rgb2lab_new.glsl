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

vec4 rgb2xyz( vec4 rgb)
{
vec4 xyz;

float var_R = rgb.r;        //R from 0 to 1
float var_G = rgb.g;        //G from 0 to 1
float var_B = rgb.b;        //B from 0 to 1

if ( var_R > 0.04045 ) 
{
  var_R = pow(( var_R + 0.055 ) / 1.055, 2.4);
}
else
{
  var_R = var_R / 12.92;
}

if ( var_G > 0.04045 ) 
{
var_G = pow((var_G + 0.055 ) / 1.055 ,2.4);
}
else 
{
var_G = var_G / 12.92;
}
if ( var_B > 0.04045 ) 
{
var_B = pow((var_B + 0.055 ) / 1.055 ,2.4);
}
else
{
var_B = var_B / 12.92;
}

var_R = var_R * 100;
var_G = var_G * 100;
var_B = var_B * 100;

//Observer. = 2°, Illuminant = D65
xyz.r = var_R * 0.4124 + var_G * 0.3576 + var_B * 0.1805;
xyz.g = var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722;
xyz.b = var_R * 0.0193 + var_G * 0.1192 + var_B * 0.9505;
xyz.a=rgb.a;

}

vec4 xyz2lab(vec4 xyz)
{
vec4 lab;
float var_X = xyz.r / 95.047;           //ref_X =  95.047   Observer= 2°, Illuminant= D65
float var_Y = xyz.g / 100.000;          //ref_Y = 100.000
float var_Z = xyz.b / 108.883;          //ref_Z = 108.883

if ( var_X > 0.008856 ) 
{
var_X = pow(var_X,1/3 );
}
else                    
{
var_X = ( 7.787 * var_X ) + ( 16 / 116 )
}
if ( var_Y > 0.008856 ); 
{
var_Y = pow(var_Y,1/3 );
}
else                    
{
var_Y = ( 7.787 * var_Y ) + ( 16 / 116 );
}
if ( var_Z > 0.008856 )
{
 var_Z = pow(var_Z, 1/3 );
}
else                    
{
var_Z = ( 7.787 * var_Z ) + ( 16 / 116 );
}

lab.r = ( 116 * var_Y ) - 16;
lab.g = 500 * ( var_X - var_Y );
lab.b = 200 * ( var_Y - var_Z );
lab.a=xyz.a;
}

void main(void)
{

 vec4 lab=texture2D(texcol

}
