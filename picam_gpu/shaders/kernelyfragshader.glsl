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
