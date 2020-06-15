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


vector<Vec3f> objectPoint(unsigned row, unsigned col, float squarSize);
//vector<Point2f> findchessboardCorners();
Mat homographyDltSimEtimation(vector<Point2f>& vector);
void homographyDltNormalize(vector<Point2f>& point, Mat& S);
Mat homographyDlt(vector<Vec2f> &imagePoints, vector<Vec3f> &objectPoints);
Mat getIntrinsicParameters(vector<Mat> H_r);
//Mat extrinsicsCalculation(Mat& intrinsics, Mat& H_r);
//vector<Mat> getExtrinsicsParameters(Mat& intrinsics, vector<Mat>& homographies);
//Mat homographyLeastSquares();
Mat normalizeImagePoints(vector<vector<Vec2f>> &points, int w, int h);
Mat intrinsicsDenormalize(Mat &K, Mat &N);
Mat getExtrinsicsParameters(Mat &K, Mat &H);
