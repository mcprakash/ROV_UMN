##########################################################################
#  YUV Frag Shader														 
#																		 
#  Shader to convert YUV color space to RGB			 				     
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
uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;

vec3 rgb2hsv(vec3 c)
{
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
}

void main(void) 
{
	float y = texture2D(tex0,tcoord).r;
	float u = texture2D(tex1,tcoord).r;
	float v = texture2D(tex2,tcoord).r;

	vec4 res;
	res.r = (y + (1.370705 * (v-0.5)));
	res.g = (y - (0.698001 * (v-0.5)) - (0.337633 * (u-0.5)));
	res.b = (y + (1.732446 * (u-0.5)));
	res.a = 0.8;

    	//gl_FragColor = clamp(vec4(rgb2hsv(res.rgb),0.8),vec4(0),vec4(1));

    	gl_FragColor = clamp(res,vec4(0),vec4(1));
}


