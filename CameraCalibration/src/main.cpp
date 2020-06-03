#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
//#include "calibration.hpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	cv::Mat image;
	image = cv::imread("/home/tamarar/Desktop/Novo/Camera_calibration/image_radial_distortion/Pic_3.png", cv::IMREAD_COLOR);
	//cout << "image shape: " << image << endl;

	Vec2d h1 =1, h2 = 2, h3 = 3, h4 = 4, h5 =5, h6 = 6, h7 = 7, h8 = 8, h9 = 9;
	vector<Vec2d> H;
	H.push_back(h1);
	H.push_back(h2);
	H.push_back(h3);
	H.push_back(h4);
	H.push_back(h5);
	H.push_back(h6);
	H.push_back(h7);
	H.push_back(h8);
	H.push_back(h9);

	for (int i = 0; i < H.size(); i++){
		cout << H[i] << " ";
	}
	


	

	return 0;
}