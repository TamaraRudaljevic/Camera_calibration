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


vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize);
Mat homographyDltSimEtimationImagePoints(vector<Point2f>& vector);
Mat homographyDltSimEtimationObjectPoints(vector<Point3f>& vector);
void homographyDltNormalizeImagePoints(vector<Point2f>& point, Mat& S);
void homographyDltNormalizeObjectPoints(vector<Point3f>& point, Mat& S);
Mat homographyDlt(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints, Mat &H);
Mat V_ij(Mat &H, int i, int j);
void getV(vector<Mat> &H, Mat &V);
bool intrinsics(Mat &B, float &u0, float &v0, float &lam, float &alpha, float &beta, float &gama);
void getIntrinsicParameters(vector<Mat>& H_r, Mat &Kp);
Mat intrinsicsDenormalize(Mat &K, Mat &N);
Mat getExtrinsicsParameters(Mat &K, Mat &H);
Mat normalizeImagePoints(vector<vector<Point2f>> &points, int w, int h);

Mat imagePointNomalizationMatrix(vector<Point2f> &imagePoints);
Mat objectPointNomalizationMatrix(vector<Point3f> &objectPoints);
Point2f meanObjectPoint(vector<Point3f> &objectPoints);
Point2f meanImagePoint(vector<Point2f> &imagePoints);
Point2f varianceImagePoints(vector<Point2f> &imagePoints, float xMean, float yMean);
Point2f varianceObjectPoints(vector<Point3f> &objectPoints, float xMean, float yMean);
