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
