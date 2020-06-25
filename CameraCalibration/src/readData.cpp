#include "../include/readData.hpp"
#include "../include/calibration.hpp"
#include "../include/homography.hpp"

Size patternsize(9,6);
string readPathImages = "/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/imagesCalib/2/";

bool readImages(vector<vector<Point2f>> &imagePoints, vector<vector<Point3f>> &modelPoints, int &width, int &height)
{
	cout << "Reading data..." << endl;
	vector<Point2f> corners;
	vector<Point3f> object;
	bool patternfound;
	int cnt = 0;
	
	for (int i = 1; i < 13; i++)
	{
		Mat image = imread(readPathImages + to_string(i) + ".png");
		width = image.size().width;
		height = image.size().height;
		patternfound = findChessboardCorners(image, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (patternfound)
		{
			cnt++;
			cout << "Corners found!" << endl;
			object = objectPoint(9, 6, 1.);
			imagePoints.push_back(corners);
			modelPoints.push_back(object);
		}
	}
	cout << cnt << " images read..." << endl;
	if (cnt != 12)
	{
		return false;
	}
	return true;
}

