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
#include <opencv2/core/mat.hpp>

using namespace std;
using namespace cv;

Mat V_ij(const Mat H, int i, int j);
Mat getV(const vector<Mat> &H);
bool intrinsics(const Mat &B, float &u0, float &v0, float &lam, float &alpha, float &beta, float &gama);
Mat getIntrinsicParameters(const vector<Mat>& H_r);
Mat getExtrinsicsParameters(const Mat &K, const Mat &H);
float reprojError(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints, Mat &homography);