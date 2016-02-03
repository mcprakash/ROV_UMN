// Obsolete Functions from all files 

// opencv_fcns.h

void rgb2lab(char[], float[]);
void rgb2hsv(char[], float[]);
extern float hsv_red[], hsv_green[], hsv_blue[], hsv_orange[];
extern char rgb_red[], rgb_green[], rgb_blue[], rgb_orange[];

// <-----------OpenCV Color Variables ------------->
extern Scalar green, red, blue, white,black;

//opencv_fcns.cpp

float hsv_red[3], hsv_green[3], hsv_blue[3], hsv_orange[3];

void rgb2lab(char src[], float dst[])
{
	float var_R = (float)((int)src[0])/255.0f; //R from 0.0 to 255.0
	float var_G = (float)((int)src[1])/255.0f; //G from 0.0 to 255.0
	float var_B = (float)((int)src[2])/255.0f; //B from 0.0 to 255.0

	if (var_R > 0.04045) {
		var_R = pow(((var_R + 0.055) / 1.055), 2.4);
	} else {
		var_R = var_R / 12.92;
	}
	if (var_G > 0.04045) {
		var_G = pow(((var_G + 0.055) / 1.055), 2.4);
	} else {
		var_G = var_G / 12.92;
	}

	if (var_B > 0.04045) {
		var_B = pow(((var_B + 0.055) / 1.055), 2.4);
	} else {
		var_B = var_B / 12.92;
	}

	var_R = var_R * 100.0;
	var_G = var_G * 100.0;
	var_B = var_B * 100.0;

	//Observer. = 2.0°, Illuminant = D65
	float X = var_R * 0.4124 + var_G * 0.3576 + var_B * 0.1805;
	float Y = var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722;
	float Z = var_R * 0.0193 + var_G * 0.1192 + var_B * 0.9505;


	//<--------------------------- XYZ 2 LAB -------------------------->
	float ref_X = 95.047; //Observer= 2.0°, Illuminant= D65
	float ref_Y = 100.000;
	float ref_Z = 108.883;

	float var_X = X / ref_X;
	float var_Y = Y / ref_Y;
	float var_Z = Z / ref_Z;

	if (var_X > 0.008856) {
		var_X = pow(var_X, (1.0 / 3.0));
	} else {
		var_X = (7.787 * var_X) + (16.0 / 116.0);
	}
	if (var_Y > 0.008856) {
		var_Y = pow(var_Y, (1.0 / 3.0));
	} else {
		var_Y = (7.787 * var_Y) + (16.0 / 116.0);
	}
	if (var_Z > 0.008856) {
		var_Z = pow(var_Z, (1.0 / 3.0));
	} else {
		var_Z = (7.787 * var_Z) + (16.0 / 116.0);
	}
	
	if(var_Y>0.008856)
		dst[0] = ((116.0 * var_Y) - 16.0)/255.0;
	else
		dst[0] = 903.3*var_Y/255.0;
	dst[1] = (500.0 * (var_X - var_Y))/255.0+0.5;
	dst[2] = (200.0 * (var_Y - var_Z))/255.0+0.5;

}

void rgb2hsv(char src[], float dst[])
{
    float r = src[0] / 255.0f;
    float g = src[1] / 255.0f;
    float b = src[2] / 255.0f;
    
    //dst[3];  // float h, s, v; // h:0-360.0, s:0.0-1.0, v:0.0-1.0
 
    
 
    float maxhsv = max_f(r, g, b);
    float minhsv = min_f(r, g, b);
 
    dst[2] = maxhsv;
 
    if (maxhsv == 0.0f) {
        dst[1] = 0;
        dst[0] = 0;
    }
    else if (maxhsv - minhsv == 0.0f) {
        dst[1] = 0;
        dst[0] = 0;
    }
    else {
        dst[1] = (maxhsv - minhsv) / maxhsv;
 
        if (maxhsv == r) {
            dst[0] = 60 * ((g - b) / (maxhsv - minhsv)) + 0;
        }
        else if (maxhsv == g) {
            dst[0] = 60 * ((b - r) / (maxhsv - minhsv)) + 120;
        }
        else {
            dst[0] = 60 * ((r - g) / (maxhsv - minhsv)) + 240;
        }
    }
 
    if (dst[0] < 0) dst[0] += 360.0f;
    
   
 
	dst[0]=dst[0]/360.0f;
	dst[1]=dst[1];
	dst[2]=dst[2];
}

// so3.h 

extern VectorXf ax;
extern MatrixXf R_ret;	
extern float ang,c_ang,s_ang;

extern int k3, k1, k2;
extern int k;

extern VectorXf gam;
extern MatrixXf step;
extern MatrixXf N;

extern float angx, angy, angz;

extern MatrixXf beta1, Pr1, Pr, Ra, Rc;
extern VectorXf Veca, Vecc, error, dummyrgm;
extern float error_a, error_c, stepsize;

//<---------------------- Variables for Function quatrot ------------------------->//
extern VectorXf ax;
extern MatrixXf R_ret;	
extern float ang,c_ang,s_ang;

extern MatrixXf R;
extern MatrixXf landmark, pt_img, pt_reproject;
extern float pts[4][2];
extern VectorXf rgm;
