##########################################################################
#  Simple Vertex Shader													 #
#																		 #
#  Shader to simply locate the position on a texture 				     #
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

attribute vec4 vertex;
uniform vec2 offset;
uniform vec2 scale;
varying vec2 tcoord;
void main(void) 
{
	vec4 pos = vertex;
	tcoord.xy = pos.xy;
	pos.xy = pos.xy*scale+offset;
	gl_Position = pos;
}
