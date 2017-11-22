#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

int flags;
Mat R1, R2, P1, P2, Q;
vector<std::string> filenames_left; 
vector<std::string> filenames_right; 

//Create transformation and rectification maps
Mat cam1map1, cam1map2;
Mat cam2map1, cam2map2;

std::ifstream file_left;
std::ifstream file_right;
char string_left[100], string_right[100];

int main(int argc, char *argv[])
{

	file_left.open("/export/patraval/robo_car_loop2/pg_cam/left.txt"); 
	file_right.open("/export/patraval/robo_car_loop2/pg_cam/right.txt"); 

	std::string line; 
	if(!file_left) //Always test the file open.
	{
		std::cout<<"Error opening output file"<< std::endl;
		return -1;
	}
	while (std::getline(file_left, line))
	{
		filenames_left.push_back(line);
	}
	file_left.close();

	if(!file_right) //Always test the file open.
	{
		std::cout<<"Error opening output file"<<std::endl;
		return -1;
	}

	while (std::getline(file_right, line))
	{
		filenames_right.push_back(line);
	}

	file_right.close();

	float m1[9] = {1011.8324475746241, 0, 1010.2511327686972, 0, 1003.4956617374614, 360.8366532960085, 0, 0, 1};	
	float m2[9] = {1012.8417246566705, 0, 1011.1641484047722, 0, 1007.9043848900899, 428.2581510403487, 0, 0, 1};
	float d1[4] = {0.10320074632021066, 0.08712205462029374, -0.042172869308959034,0.08767489751000035};
	float d2[4] = {0.09872479486626964, 0.1010630830192847, -0.0651838838938228, 0.1005707298731867};

	// float m1[9] = {1010.7234313264403, 0, 1009.1829722092845, 0, 1002.9629088985278, 358.2714637354781, 0, 0, 1};	
	// float m2[9] = {1011.2044423890844, 0, 1009.7482878445264, 0, 1001.6375234436252, 427.7572918140741, 0, 0, 1};
	// float d1[4] = { 0.09591790561780944, 0.12373563338566723, -0.0975544797256392, 0.11613597508133179};
	// float d2[4] = {0.09155160663130679, 0.13854249300660224, -0.12766992970542612,   0.13681343069404775};



	Mat M1 = Mat(3, 3, CV_32F, m1);
	Mat M2 = Mat(3, 3, CV_32F, m2);
	Mat D1 = Mat(1, 4, CV_32F, d1);
	Mat D2 = Mat(1, 4, CV_32F, d2);

 	float ro[9] = {0.999780071930088, -0.020133464725992617, 0.005869528907105019, 0.020176665356444842, 0.9997690720508218, -0.007396265661047244,-0.005719261015041106, 0.007513066535174506, 0.999955420948692};
  	float tr[3] = {  -0.5012085599311499, -0.00974463773435044, 0.0037312131561487716};



 	// float ro[9] = {0.9998805919777298, -0.014374203502396918, 0.005673099676157448, 0.014425596660187882, 0.9998543112732692, -0.009124603512131731, -0.005541114261726946, 0.009205351809013641, 0.9999422771094391};
  // 	float tr[3] = { -0.5011739875266582,  -0.00853806222837021,  0.0024935075413970646};
	Mat RO = Mat(3,3, CV_32F, ro);
	Mat TR = Mat(3, 1 , CV_32F, tr);


   fisheye::stereoRectify(M1, D1, M2, D2,  Size(2048, 1136), RO, TR, R1,  R2,  P1,  P2,  Q, flags = CV_CALIB_ZERO_DISPARITY);
   fisheye::initUndistortRectifyMap(M1, D1, R1, P1, Size(2048, 1136) , CV_32F, cam1map1, cam1map2);
   fisheye::initUndistortRectifyMap(M2, D2, R2, P2, Size(2048, 1136) , CV_32F, cam2map1, cam2map2);
 
   Mat left_rect, right_rect; //Create matrices for storing rectified images
 

for(int i =0; i < filenames_left.size(); ++i)	
    {

		cout<<filenames_left[i]<<endl;
		cout<<filenames_right[i]<<endl;	

		Mat left = imread(filenames_left[i]);
		Mat right = imread(filenames_right[i]);

		if(!left.data)
		cerr << "Problem loading image!!!" << endl;	

		if(!right.data)
			cerr << "Problem loading image!!!" << endl;

		//Rectify and undistort images
		remap(left, left_rect, cam1map1, cam1map2, INTER_LINEAR);
		remap(right, right_rect, cam2map1, cam2map2, INTER_LINEAR);

		sprintf(string_left, "/export/patraval/robo_car_loop2/pg_cam/rect/left/%d.png", i );
		sprintf(string_right, "/export/patraval/robo_car_loop2/pg_cam/rect/right/%d.png", i );

		imwrite(string_left, left_rect);
		imwrite(string_right, right_rect);

	}
   return 0;
}
