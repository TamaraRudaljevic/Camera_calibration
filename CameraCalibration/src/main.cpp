#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include "../include/calibration.hpp"
#include "../include/readData.hpp"

using namespace std;
using namespace cv;

// extern vector<Point2f> imagePoints;
// extern vector<Point3f> objectPoints;
string readPathZhang = "/home/tamarar/Desktop/Novo/Camera_calibration/calib_data/zhang_data";
string readPath = "/home/tamarar/Desktop/calib/camera-calibration/calib_data/ground_truth/";

int main(int argc, char *argv[])
{
	cout << "MAIN" << endl;
	vector<vector<Vec2f>> imagePointsNormZhang;
	vector<vector<Vec2f>> imagePointNorm;
	vector<Vec3f> modelPointsZhang;
	vector<Vec3f> modelPoints;
	int w = 0, h = 0, wZ = 0, hZ = 0;
	bool readZ = 0, read = 0;

	float modelSize = 3.;
	cout << "Reading data..." << endl;

	readZ = readZhang(readPathZhang, imagePointsNormZhang, modelPointsZhang, wZ, hZ);
	read = readData(readPath, imagePointNorm, modelPoints, w, h, modelSize);
	cout << readZ << " " << read << endl;


	if(!read)
	{
		cout << "Error reading data..." << endl;
	}

	auto imagePoints = imagePointNorm;
	auto imagePointsZhang = imagePointsNormZhang;

	auto NZhang = normalizeImagePoints(imagePointsNormZhang, wZ, hZ);
	auto NZhangInv = NZhang.clone();
	invert(NZhangInv, NZhangInv);

	auto N = normalizeImagePoints(imagePointNorm, w, h);
	auto NInv = N.clone();
	invert(NInv, NInv);

	vector<Mat> H(imagePointNorm.size());
	vector<Mat> HZhang(imagePointNorm.size());

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		H[i] = homographyDlt(imagePointNorm[i], modelPoints);
		cout << "H[i]" << endl;
		HZhang[i] = homographyDlt(imagePointsNormZhang[i], modelPointsZhang);
	}

	Mat Kp = getIntrinsicParameters(H);
	Mat KZhanp = getIntrinsicParameters(HZhang);
	Mat tmp = Mat::zeros(3, 3, CV_64FC1);

	if (countNonZero(Kp) > 3)
	{
		cout << "Error calculating intrinsic parameters." << endl;
		return EXIT_FAILURE;
	}
	
	cout << "Intrinsics matrix K': " << endl << Kp << endl;

	auto K = intrinsicsDenormalize(Kp, N);

	cout << "Denormalize intrinsics matrix K: " << endl << K << endl;

	vector<Mat> rtMatrices;

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		Mat tmp = NInv*H[i];
		Mat rt = getExtrinsicsParameters(K, tmp);
		cout << "Extrinsic parameters: " << endl << rt << endl;
		rtMatrices.push_back(rt);
	}



	
	// //******************************************************************//
	// //***********************CHESSBOARD CORNERS*************************//
	// //******************************************************************//

	// vector<Point2f> corners = findchessboardCorners();

	// for (unsigned i = 0; i < imagePoints.size(); i++)
	// {
	// 	cout << imagePoints[i].x << " " << imagePoints[i].y << endl;
	// }

	// cout << "******************" << endl;

	// for (unsigned i = 0; i < objectPoints.size(); i++)
	// {
	// 	cout << objectPoints[i].x << " " << objectPoints[i].y << " " << objectPoints[i].z << endl;;
	// }

	// cout << endl <<  "******************" << endl;


	// //******************************************************************//
	// //*************************HOMOGRAPHY******************************//
	// //*****************************************************************//

	// //Mat H = homographyDlt();
	// //cout << H << endl;
	// vector<Mat> homographies;
	// Mat H;

	// // for (unsigned i = 0; i < objectPoints.size(); i++)
	// // {
	// // 	H = homographyDlt();
	// // 	homographies.push_back(H);
	// // }

	// // for (unsigned i = 0; i < homographies.size(); i++)
	// // {
	// // 	cout << homographies[i] << endl;
	// // }

	// Mat h = homographyDlt();
	// cout << "h = " << h << endl;
	// //cout << "h = " << endl;

	// // for (int i = 0; i < objectPoints.size(); i++)
	// // {
	// // 	H = homographyLeastSquares();
	// // 	homographies.push_back(H);
	// // }

	// // for (unsigned i = 0; i < homographies.size(); i++)
	// // {
	// // 	cout << homographies[i] << endl;
	// // }
	// // H = homographyLeastSquares();
	// // cout << "H = " << H << endl;
	
	

	// //************************************************//
	// //**********calibration with openCV***************//

	// // Mat image = imread("/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_1.jpg");
	// // Mat K, D;
	// // vector<Mat> rvecs, tvecs;
	// // calibrateCamera(objectPoints, imagePoints, image.size(), K, D, rvecs, tvecs);

	// //cout << "Camera matrix, K = " << endl << K << endl;
	// //cout << "Distortion coeff, dist = " << endl << D << endl;

	// //************************************************//

	// //#################################################//

	// //************************************************//

	// cout << "*************************************" << endl;
	// cout << "*************************************" << endl;
	// cout << "*************************************" << endl;
	// cout << "*************************************" << endl;
	// vector<vector<Vec2f>> imagePointNorm = imageCorners();
	// for (unsigned i = 0; i < imagePointNorm.size(); i++)
	// {
	// 	for (unsigned j = 0; j < imagePointNorm[i].size(); j++)
	// 	{
	// 		cout << imagePointNorm[i][j] << endl;
	// 	}
	// }

	// cout << endl << "*****************************" << endl << endl;

	// vector<Vec3f> modelPoints = obj(6, 9, 1.);

	// for (unsigned i = 0; i < modelPoints.size(); i++)
	// {
	// 	//cout << modelPoints[i] << endl;
	// }

	// auto imagePointOrig = imagePointNorm;
	// auto N = normalizeImagePoints(imagePointNorm, imagePointNorm.size(), imagePointNorm[0].size());
	// cout << N << endl;
	// auto N_inv = N.clone();
	// invert(N_inv, N_inv);

	// auto imagePointsCnt = imagePointNorm.size();

	// vector<Mat> homography;
	// Mat hi;

	// for (unsigned i = 0; i < imagePointsCnt; i++)
	// {
	// 	hi = homographyDlt(imagePointNorm[i], modelPoints);
	// 	homography.push_back(hi);
	// }

	// for (unsigned i = 0; i < homography.size(); i++)
	// {
	// 	cout << homography[i] << endl;
	// 	cout << "**********************" << endl;
	// }

	// //************************************************//	

	return 0;
}