##########################################################################
#  Undistort Frag Shader												 #
#																		 #
#  Shader to implement the undistort function of OpenCV				     #
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
uniform vec2 ff;
uniform vec2 cen;
uniform vec4 distcoeff;

void main(void) 
{
	vec2 xy=(tcoord-cen/texelsize)/ff;
	float r2=dot(xy,xy);
	float r4=r2*r2;
	float coeff=distcoeff.x*r2+distcoeff.y*r4;
	
	vec2 dxy= vec2(distcoeff.z*2.0*xy.x*xy.y+distcoeff.w*(r2+2.0*xy.x*xy.x),distcoeff.z*(r2+2.0*xy.y*xy.y)+distcoeff.w*2.0*xy.x*xy.y);
	
	xy=((xy+xy*coeff+dxy)*ff)+cen/texelsize;
	
	//xy=((tcoord+tcoord*coeff+dxy)*ff+cen)/texelsize;
	
	//xy=tcoord+dxy;
	
    gl_FragColor = clamp(texture2D(tex,xy),vec4(0),vec4(1));
}
