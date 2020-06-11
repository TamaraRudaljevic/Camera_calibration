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
	
	//******************************************************************//
	//***********************CHESSBOARD CORNERS*************************//
	//******************************************************************//

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


	//******************************************************************//
	//*************************HOMOGRAPHY******************************//
	//*****************************************************************//

	//Mat H = homographyDlt();
	//cout << H << endl;
	vector<Mat> homographies;
	Mat H;

	// for (unsigned i = 0; i < objectPoints.size(); i++)
	// {
	// 	H = homographyDlt();
	// 	homographies.push_back(H);
	// }

	// for (unsigned i = 0; i < homographies.size(); i++)
	// {
	// 	cout << homographies[i] << endl;
	// }

	Mat h = homographyDlt();
	cout << "h = " << h << endl;
	//cout << "h = " << endl;

	// for (int i = 0; i < objectPoints.size(); i++)
	// {
	// 	H = homographyLeastSquares();
	// 	homographies.push_back(H);
	// }

	// for (unsigned i = 0; i < homographies.size(); i++)
	// {
	// 	cout << homographies[i] << endl;
	// }
	// H = homographyLeastSquares();
	// cout << "H = " << H << endl;
	
	

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