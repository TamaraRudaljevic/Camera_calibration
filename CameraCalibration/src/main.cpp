#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include "../include/calibration.hpp"
#include "../include/readData.hpp"

using namespace std;
using namespace cv;

vector<vector<Point2f>> imagePointNorm;
vector<vector<Point3f>> modelTmp;
Mat Kp(3, 3, CV_32FC1); 
vector<Mat> H;
Mat tmpH(3, 3, CV_32FC1);

// extern vector<Point2f> imagePoints;
// extern vector<Point3f> objectPoints;
string readPathZhang = "/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/zhang_data";
string readPathImages = "/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/imagesCalib/2/";
//string readPath = "/home/tamarar/Desktop/Novo/Camera_calibration/CamaearCalibration/ground_truth/";

int main(int argc, char *argv[])
{
	cout << "MAIN" << endl;
	/*vector<vector<Point2f>> imagePointsNormZhang;
	vector<Point3f> modelPointsZhang;
	vector<Point3f> modelPoints;
	vector<vector<Point3f>> modelTmp;*/
	int w = 0, h = 0, wZ = 0, hZ = 0;
	bool readZ = 0, read = 0;

	float modelSize = 3.;
	cout << "Reading data..." << endl;

	//readZ = readZhang(readPathZhang, imagePointsNormZhang, modelPointsZhang, wZ, hZ);
	readImages(readPathImages, imagePointNorm, modelTmp);

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		for (unsigned j = 0; j < imagePointNorm.size(); j++)
		{
			//cout << imagePointNorm[i][j].x << " " << imagePointNorm[i][j].y << endl;
			//cout << "Number of images: " << imagePointNorm.size() << endl;
		}

		//cout << endl;
	}

	
	//read = readData(readPath, imagePointNorm, modelPoints, w, h, modelSize);

	
	// FILE* file = fopen("/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/file.txt", "w");
	// if (file == NULL)
	// {
	// 	cout << "File out not open.." << endl;
	// }

	// float* ptr;
	// for (unsigned i = 0; i < imagePointsNormZhang.size(); i++)
	// {
	// 	for (unsigned j = 0; j < imagePointsNormZhang.size(); j++)
	// 	{
	// 		Vec2f(ptr) = imagePointsNormZhang[i][j];
	// 	}
	// }
	// for (unsigned i = 0; i < imagePointsNormZhang.size(); i++)
	// {
	// 	for (unsigned j = 0; j < imagePointsNormZhang[i].size(); j++)
	// 	{
			
	// 		fwrite(ptr, sizeof(float), sizeof(ptr), file);
		
	// 	}
			
	// }
	// fclose(file);


	// if (readZ == false)
	// {
	// 	cout << "Error reading Zhang data..." << endl;
	// 	return EXIT_FAILURE;
	// }

	// if(!read)
	// {
	// 	cout << "Error reading data..." << endl;
	// }

	// auto imagePoints = imagePointNorm;
	// auto imagePointsZhang = imagePointsNormZhang;

	// //****************************

	// auto NZhang = normalizeImagePoints(imagePointsNormZhang, wZ, hZ);
	// auto NZhangInv = NZhang.clone();
	// invert(NZhangInv, NZhangInv);

	// auto N = normalizeImagePoints(imagePointNorm, w, h);
	// auto NInv = N.clone();
	// invert(NInv, NInv);

	//****************************

	
	//****************************

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		//H[i] = homographyDlt(imagePointNorm[i], modelPoints);
		homographyDlt(imagePointNorm[i], modelTmp[i], tmpH);
		//optimize(imagePointsNormZhang[i], modelPointsZhang, HZhang[i], 1e-05);
		H.push_back(tmpH);
		//cout << "H = " << tmpH << endl;
	}

	for (unsigned i = 0; i < H.size(); i++)
	{
		//cout << i << " " << H[i] << endl;
	}

	
	getIntrinsicParameters(H, Kp);
	cout << "Kp = " << Kp << endl;
	//Mat KZhangp = getIntrinsicParameters(HZhang);
	// Mat tmp = Mat::zeros(3, 3, CV_64FC1);

	// // if (countNonZero(Kp) > 3)
	// // {
	// // 	cout << "Error calculating intrinsic parameters." << endl;
	// // 	return EXIT_FAILURE;
	// // }
	
	//cout << "Intrinsics matrix K': " << endl << Kp << endl;

	// auto KZhang = intrinsicsDenormalize(KZhangp, NZhang);

	// cout << "Denormalize intrinsics matrix K: " << endl << KZhang << endl;

	// vector<Mat> rtMatrices;

	// for (unsigned i = 0; i < imagePointNorm.size(); i++)
	// {
	// 	Mat tmp = NZhangInv*HZhang[i];
	// 	Mat rt = getExtrinsicsParameters(KZhang, tmp);
	// 	cout << "Extrinsic parameters: " << endl << rt << endl;
	// 	rtMatrices.push_back(rt);
	// }



	
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