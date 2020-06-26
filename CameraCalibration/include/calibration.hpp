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
Mat distortion(vector<Point2f> &imagePoints, vector<vector<Point3f>> &objectPoints, Mat &K, vector<Mat> &RT);
//Mat distortion(vector<vector<Point2f>> &imagePoints, vector<vector<Point2f>> &imagePointsNorm, vector<vector<Point2f>> &imageProj, Mat &K);
float computeReprojectionError(const vector<vector<Point3f>> &objectPoints, const vector<vector<Point2f>> &imagePoints, const vector<Mat> &rvecs, const vector<Mat> &tvecs, const Mat &cameraMatrix, bool fisheye);
float computeReprojectionError(const vector<vector<Point3f>> &objectPoints, const vector<vector<Point2f>> &imagePoints, const vector<Mat> &rvecs, const vector<Mat> &tvecs, const Mat &cameraMatrix, const Mat &distCoef, vector<float> &perViewErrors, bool fisheye);
float calc_reprojection(const Mat &A, const Mat &K, const vector<Point3f> &model_pts, const vector<Point2f> &image_pts, vector<Point2f> &image_pts_proj, vector<float> &k);
vector<float> reproject_point(const vector<float> &world_ptn, const Mat &A, const Mat &K, const vector<float> &k);
float reprojError(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints, Mat &homography);
Mat lensDistortion(Mat &A, vector<Mat> &extrinsics, vector<vector<Point2f>> &imagePoints, vector<vector<Point3f>> &objectPoints);