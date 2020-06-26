#include "../include/calibration.hpp"
#include "../include/readData.hpp"
#include "../include/homography.hpp"


//string readPath = "/home/tamarar/Desktop/Novo/Camera_calibration/CamaearCalibration/ground_truth/";
/*
void calibrateOpenCV(vector<vector<Point3f>> &modelPoints, vector<vector<Point2f>> &imagePointNorm, Mat &K, Mat &D)
{
	int w = 0, h = 0;
	readImages(readPathImages, imagePointNorm, modelPoints, w, h);

	for (unsigned i = 0; i < imagePointNorm.size(); i++)
	{
		cout << imagePointNorm[i] << endl;
	}
    Mat image = imread("/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/imagesCalib/2/1.png");
    vector<Mat> rvecs, tvecs;
    bool ret = calibrateCamera(modelPoints, imagePointNorm, image.size(), K, D, rvecs, tvecs, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
}
*/
int main(int argc, char *argv[])
{

	cout << "********************************" << endl;
	vector<vector<Point2f>> imagePoints;
	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>> imagePointProj;
	vector<vector<Point3f>> objectPointProj;
	int width = 0, height = 0;

	/*#######################################################
	*******************Reading images**********************
	#######################################################*/
	if (!readImages(imagePoints, objectPoints, width, height))
	{
		cout << "Error reading images..." << endl;
		cout << "No images..." << endl;
		return EXIT_FAILURE;
	}

	
	imagePointProj = imagePoints;
	objectPointProj = objectPoints;

	
	/*#######################################################
	*****************Homography calculation******************
	#######################################################*/
	int imagePointCnt = imagePoints.size();

	vector<Mat> H;
	Mat h = Mat(3, 3, CV_32FC1);

	for (int i = 0; i < imagePointCnt; i++)
	{
		h = homographyDlt(imagePoints[i], objectPoints[i]);
		H.push_back(h);
	}


	/*#######################################################
	**********************Camera matrix**********************
	#######################################################*/
	cout << "********************************" << endl;
	Mat K = getIntrinsicParameters(H);

	cout << "**Intrinsic parameters**" << endl << endl << "K = " << K << endl;


	
	/*#######################################################
	*******************Extrinsic parameters******************
	#######################################################*/
	Mat rt;
	vector<Mat> RT;

	vector<vector<Point2f>> imagePoitsProj(imagePointCnt);

	for (unsigned i = 0; i < H.size(); i++)
	{
		rt = getExtrinsicsParameters(K, H[i]);
		//cout << "rt = " << rt << endl;
		RT.push_back(rt);
	}


	/*#######################################################
	*******************Error calculation********************
	#######################################################*/
	cout << "********************************" << endl;
	cout << "***Reprojection error***" << endl << endl;

	float err;
	for (unsigned i = 0; i < H.size(); i++)
	{
		err = reprojError(imagePointProj[i], objectPointProj[i], H[i]);
		cout << "Image " << i << "., error - " << err << endl;
	}


	//lensDistortion(K, RT, imagePoints, objectPoints);


	// //****************CALIBRATE OPENCV****************//

	// vector<vector<Point3f>> modelOpenCV;
	// vector<vector<Point2f>> imageOpenCV;
	// Mat KOpenCv(3, 3, CV_32FC1), DOpenCv;


	// calibrateOpenCV(modelOpenCV, imageOpenCV, KOpenCv, DOpenCv);

	//cout << "K OpenCv = " << KOpenCv << endl;
	


	return 0;
}


