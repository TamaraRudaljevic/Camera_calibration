#include <stdio.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <inttypes.h>

using namespace std;
using namespace cv;


vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize);
vector<Point2f> findchessboardCorners();
Mat homographyDltSimEtimation(vector<Point2f>& vector);
void homographyDltNormalize(vector<Point2f>& point, Mat& S);
Mat homographyDlt();
int argmin(vector<float>& vector);
Mat getIntrinsicParameters(vector<Mat> H_r);
Mat extrinsicsCalculation(Mat& intrinsics, Mat& H_r);
vector<Mat> getExtrinsicsParameters(Mat& intrinsics, vector<Mat>& homographies);
