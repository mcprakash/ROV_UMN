##########################################################################
#  HSV Frag Shader														 
#																		
#  Shader to convert RGB color space to HSV			 				     
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

vec3 rgb2hsv(vec3 c)
{
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 # d + e)), d / (q.x + e), q.x);
}

void main(void) 
{
    vec4 hsv = texture2D(tex,tcoord);
    

    gl_FragColor = clamp(vec4(rgb2hsv(hsv.rgb),0.8),vec4(0),vec4(1));
}
