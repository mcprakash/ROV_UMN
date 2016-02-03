##########################################################################
#  Kernel-y Frag Shader													 #
#																		 #
#  Shader to implement a 5 point kernel	on y-axis	 				     #
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
uniform vec2 texelsize;

float offset[5], weight[5];

void main(void) 
{
	offset[0]=0.0; 
	offset[1]=1.0; 
	offset[2]=2.0; 
	offset[3]=3.0; 
	offset[4]=4.0; 
	
	weight[0]=0.2270270270;
	weight[1]=0.1945945946;
	weight[2]=0.1216216216;
	weight[3]=0.0540540541;
	weight[4]=0.0162162162;
	
	vec4 col=texture2D(tex,tcoord)*weight[0];
	
	for(int i=1;i<5;i++)
	{
		col=col+texture2D(tex,(tcoord+vec2(0.0,offset[i])*texelsize.y))*weight[i];
		col=col+texture2D(tex,(tcoord-vec2(0.0,offset[i])*texelsize.y))*weight[i];
	}
	
	col.a=0.80;
	
	gl_FragColor = clamp(col,vec4(0),vec4(1));
}
