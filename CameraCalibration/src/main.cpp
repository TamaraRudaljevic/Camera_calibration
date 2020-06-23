#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include "../include/calibration.hpp"
#include "../include/readData.hpp"

using namespace std;
using namespace cv;

// extern vector<Point2f> imagePoints;
// extern vector<Point3f> objectPoints;
string readPathZhang = "/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/zhang_data";
string readPathImages = "/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/imagesCalib/2/";
//string readPath = "/home/tamarar/Desktop/Novo/Camera_calibration/CamaearCalibration/ground_truth/";

void calibrateOpenCV(vector<vector<Point3d>> &modelPoints, vector<vector<Point2d>> &imagePointNorm, Mat &K, Mat &D)
{
	vector<Point2d> corners;
	// vector<Vec3f> objPoint = objectPoint(9, 6, 1.);
	vector<Point3d> object;
	bool patternfound;
	Size patternsize(9,6);
	
	for (int i = 1; i < 13; i++)
	{
		Mat image = imread(readPathImages + to_string(i) + ".png");
		patternfound = findChessboardCorners(image, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (patternfound)
		{
			//cout << "Corners found! " << "i = " << i << endl;
			object = objectPoint(9, 6, 1.);
			modelPoints.push_back(object);
			imagePointNorm.push_back(corners);
		}
	}

    Mat image = imread("/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/imagesCalib/2/1.png");
    vector<Mat> rvecs, tvecs;
    bool ret = calibrateCamera(modelPoints, imagePointNorm, image.size(), K, D, rvecs, tvecs, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
}

int main(int argc, char *argv[])
{
	cout << "MAIN" << endl;
	vector<vector<Point2d>> imagePointsNormZhang;
	vector<vector<Point2d>> imagePointNorm;
	vector<Point3d> modelPointsZhang;
	vector<Point3d> modelPoints;
	vector<vector<Point3d>> tmpModelPoints;
	int w = 0, h = 0, wZ = 0, hZ = 0;
	bool readZ = 0, read = 0;

	float modelSize = 3.;
	cout << "Reading data..." << endl;

	//readZ = readZhang(readPathZhang, imagePointsNormZhang, modelPointsZhang, wZ, hZ);
	readImages(readPathImages, imagePointNorm, tmpModelPoints, w, h);

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		for (unsigned j = 0; j < imagePointNorm.size(); j++)
		{
			//cout << imagePointNorm[i][j] << endl;
			//cout << "Number of images: " << imagePointNorm.size() << endl;
		}

		//cout << endl;
	}

	//cout << "Image points size = " << imagePointNorm[1].size() << endl;

	for (unsigned i = 0; i < tmpModelPoints.size(); i++)
	{
		//cout << "Model points = " << tmpModelPoints[i] << endl;
	}
	//read = readData(readPath, imagePointNorm, modelPoints, w, h, modelSize);
	cout << "size = " << tmpModelPoints.size() << endl;

	//****************************

	vector<Mat> H;
	//vector<Mat> HLastS;
	vector<Mat> HZhang(imagePointsNormZhang.size());
	Mat tmpH(3, 3, CV_64FC1);
	//Mat Hls(3, 3, CV_32FC1);
	//****************************

	// Size size = imagePointNorm.size();
	// h = size.height;
	// // w = size.width;
	// auto N = normalizeImagePoints(imagePointNorm, w, h);
	// cout << "w = " << w << ", h = " << h << endl;
	// cout << "N = " << N <<endl;
	// Mat N_inv = N.inv();
	// cout << "N_inv = " << N_inv << endl;

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		//H[i] = homographyDlt(imagePointNorm[i], modelPoints);
		tmpH = homographyDlt(imagePointNorm[i], tmpModelPoints[i]);
		//Hls = homographyLeastSquares(imagePointNorm[i], tmpModelPoints[i]);
		//optimize(imagePointsNormZhang[i], modelPointsZhang, HZhang[i], 1e-05);
		H.push_back(tmpH);
		//HLastS.push_back(Hls);
		//cout << "H = " << tmpH << endl;
	}

	// for (unsigned i = 0; i < H.size(); i++)
	// {
	// 	cout << "H[" << i << "] = " << H[i] << endl;
	// }

	Mat Kp = getIntrinsicParameters(H);
	// //Mat KpLs = getIntrinsicParameters(HLastS);
	cout << "Kp = " << Kp << endl;
	// Mat K;
	// K = intrinsicsDenormalize(Kp, N);
	// cout << "K = " << K << endl;
	// Mat rt;
	// vector<Mat> RT;


	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		// rt = getExtrinsicsParameters(Kp, H[i]);
		// RT.push_back(rt);
	}	

	// for (unsigned i = 0; i < RT.size(); i++)
	// {
	// 	//cout << "RT[" << i << "] = " << RT[i] << endl;
	// }

	// //****************CALIBRATE OPENCV****************//

	vector<vector<Point3d>> modelOpenCV;
	vector<vector<Point2d>> imageOpenCV;
	Mat KOpenCv(3, 3, CV_64FC1), DOpenCv;


	//calibrateOpenCV(modelOpenCV, imageOpenCV, KOpenCv, DOpenCv);

	//cout << "K OpenCv = " << KOpenCv << endl;
	


	return 0;
}


