##########################################################################
#  Laplacian Frag Shader												 
#																		 
#  Shader to implement Laplacian operation of LOG	 				     
##########################################################################
#  Copyright(C)  <December 2015>  <Chris Cummings 
#  MyRobotBlogPost -> http://robotblogging.blogspot.com/2013/
#  Shader files by cheerfulprogrammer ->
#       http://www.cheerfulprogrammer.com/downloads/picamgpu/picam_gpu.zip
#						
#  Chandra P. Mangipudi -> Update code for LAB
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
uniform vec2 texelsize;

mat3 m = mat3(-1.0,-1.0,-1.0, 
			 -1.0, 8.0,-1.0,
			 -1.0,-1.0,-1.0);

float fac;
void main(void) 
{
	//vec4 col=texture2D(tex,tcoord)*m[1][1];
	vec4 col=vec4(0);
	
	for(int xoffset = -1; xoffset <= 1; xoffset++)
	{
		for(int yoffset = -1; yoffset <= 1; yoffset++)
		{
			if(xoffset==0 && yoffset==0)
			{
				fac=8.0;
			}
			else
			{
				fac=-1.0;
			}
			
			vec2 offset = vec2(xoffset,yoffset);
			col = col+texture2D(tex,tcoord+offset*texelsize)*fac;
		}
	}
	
	/*for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			col=col+texture2D(tex,(tcoord+vec2(i-1.0,j-1.0)*texelsize))*m[i][j];
		}	
	}
	*/
	col.a=0.80;
	
	gl_FragColor = clamp(col,vec4(0),vec4(1));
}
