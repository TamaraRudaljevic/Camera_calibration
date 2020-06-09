#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include "../include/calibration.hpp"

using namespace std;
using namespace cv;

extern vector<vector<Point2f>> imagePoints;
extern vector<vector<Point3f>> objectPoints;

int main(int argc, char *argv[])
{
	
	//************************************************//

	vector<Point2f> corners = findchessboardCorners();

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		for (unsigned j = 0; j < imagePoints[i].size(); j++)
		{
			cout << imagePoints[i][j];
		}
	}

	cout << "******************" << endl;

	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		for (unsigned j = 0; j < objectPoints[i].size(); j++)
		{
			cout << objectPoints[i][j];
		}
	}

	cout << endl <<  "******************" << endl;
	
	


	

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