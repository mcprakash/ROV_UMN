##########################################################################
#  Color Filter Shader													 
#																		 
#  Detect Landmark spheres and color code them		 				     
##########################################################################
#  Copyright(C)  <December 2015>  <Chandra P. Mangipudi					 
#																		 
#  This program is free software : you can redistribute it and/or modify 
#  it under the terms of the GNU General Public License as published by  
#  the Free Software Foundation, either version 3 of the License, or	 
#  (at your option) any later version.									 
#																		 
#  This program is distributed in the hope that it will be useful,		 
#  but WITHOUT ANY WARRANTY; without even the implied warranty of		 
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the			 
#  GNU General Public License for more details.							 
#																		 
#  You should have received a copy of the GNU General Public License	 
#  along with this program.If not, see <http://www.gnu.org/licenses/>.	 
##########################################################################

varying vec2 tcoord;
uniform sampler2D tex;
uniform vec4 col1;
uniform vec4 col2;
uniform vec4 col3;
uniform vec4 col4;

void main(void) 
{
	vec4 texcol = texture2D(tex,tcoord);

	//float tol_val=0.0117647;
	
	// Working values without Gaussian Filter
	//vec4 tol_val=vec4(0.01 ,0.005 ,0.005 ,0.01);
	
	// Prototype values
	//vec4 tol_val=vec4(0.025 ,0.015 ,0.004, 0.005);
	
	// Trial values for HSV space
	//vec4 tol_val=vec4(0.05 ,0.03 ,0.04 ,0.03);
	
	// Proof of Concept values
	//vec4 tol_val=vec4(0.006 ,0.003 ,0.0020, 0.012);
	
	// Proof of Concept values uw
	vec4 tol_val=vec4(0.008 ,0.0035 ,0.040, 0.012);
	
	
	//float check_val1= 0.5*(texcol.r-col1.r)*(texcol.r-col1.r)+(texcol.g-col1.g)*(texcol.g-col1.g)+(texcol.b-col1.b)*(texcol.b-col1.b);
	//float check_val2= 0.5*(texcol.r-col2.r)*(texcol.r-col2.r)+(texcol.g-col2.g)*(texcol.g-col2.g)+(texcol.b-col2.b)*(texcol.b-col2.b);
	float check_val3= 0.25*(texcol.r-col3.r)*(texcol.r-col3.r)+(texcol.g-col3.g)*(texcol.g-col3.g)+(texcol.b-col3.b)*(texcol.b-col3.b);
	//float check_val4= (texcol.r-col4.r)*(texcol.r-col4.r)+(texcol.g-col4.g)*(texcol.g-col4.g)+(texcol.b-col4.b)*(texcol.b-col4.b);
	
	float check_val1= (texcol.g-col1.g)*(texcol.g-col1.g)+(texcol.b-col1.b)*(texcol.b-col1.b);
	float check_val2= (texcol.g-col2.g)*(texcol.g-col2.g)+(texcol.b-col2.b)*(texcol.b-col2.b);
	//float check_val3= (texcol.g-col3.g)*(texcol.g-col3.g)+(texcol.b-col3.b)*(texcol.b-col3.b);
	float check_val4= (texcol.g-col4.g)*(texcol.g-col4.g)+(texcol.b-col4.b)*(texcol.b-col4.b);
	
	// Red Ball
	if(check_val1<tol_val.r)
	{
		gl_FragColor = vec4(1.0,0.0,0.0,0.8);
	}

	// Green Ball
	else if(check_val2<tol_val.g)
	{
		gl_FragColor = vec4(0.0,1.0,0.0,0.8);
	}
	
	// Orange Ball
	else if(check_val4<tol_val.a)
	{
		gl_FragColor = vec4(1.0,0.0,1.0,0.8);
	}

	// Blue Ball
	else if(check_val3<tol_val.b)
	{
		gl_FragColor = vec4(0.0,0.0,1.0,0.8);
	}

	
	else
	{
		gl_FragColor = vec4(0.0,0.0,0.0,0.8);
	}

	
}
