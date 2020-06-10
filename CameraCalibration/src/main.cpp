#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include "../include/calibration.hpp"

using namespace std;
using namespace cv;

extern vector<Point2f> imagePoints;
extern vector<Point3f> objectPoints;

int main(int argc, char *argv[])
{
	
	//************************************************//

	vector<Point2f> corners = findchessboardCorners();

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		cout << imagePoints[i].x << " " << imagePoints[i].y << endl;
	}

	cout << "******************" << endl;

	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		cout << objectPoints[i].x << " " << objectPoints[i].y << " " << objectPoints[i].z << endl;;
	}

	cout << endl <<  "******************" << endl;

	Mat H = homographyDlt();
	cout << H << endl;
	
	


	

	//************************************************//
	//**********calibration with openCV***************//

	// Mat image = imread("/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_1.jpg");
	// Mat K, D;
	// vector<Mat> rvecs, tvecs;
	// calibrateCamera(objectPoints, imagePoints, image.size(), K, D, rvecs, tvecs);

	//cout << "Camera matrix, K = " << endl << K << endl;
	//cout << "Distortion coeff, dist = " << endl << D << endl;

	//************************************************//

	

	//************************************************//



	

	return 0;
}