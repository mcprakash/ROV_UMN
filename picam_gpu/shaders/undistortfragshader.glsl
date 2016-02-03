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
