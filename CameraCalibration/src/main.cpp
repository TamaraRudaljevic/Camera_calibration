#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include "../include/calibration.hpp"
#include "../include/calib.hpp"

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

	cout << "*************************************" << endl;
	cout << "*************************************" << endl;
	cout << "*************************************" << endl;
	cout << "*************************************" << endl;
	vector<vector<Vec2f>> imagePointNorm = imageCorners();
	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		for (unsigned j = 0; j < imagePointNorm[i].size(); j++)
		{
			cout << imagePointNorm[i][j] << endl;
		}
	}

	cout << endl << "*****************************" << endl << endl;

	vector<Vec3f> modelPoints = obj(6, 9, 1.);

	for (unsigned i = 0; i < modelPoints.size(); i++)
	{
		//cout << modelPoints[i] << endl;
	}

	auto imagePointOrig = imagePointNorm;
	auto N = normalizeImagePoints(imagePointNorm, imagePointNorm.size(), imagePointNorm[0].size());
	cout << N << endl;
	auto N_inv = N.clone();
	invert(N_inv, N_inv);

	auto imagePointsCnt = imagePointNorm.size();

	vector<Mat> homography;

	for (unsigned i = 0; i < imagePointsCnt; i++)
	{
		homography[i] = homographyDlt(imagePointNorm[i], modelPoints);
		cout << "H[" << i << "] = " << homography[i] << endl;
	}




	//Mat H = homographyDlt(image)



	

	return 0;
}