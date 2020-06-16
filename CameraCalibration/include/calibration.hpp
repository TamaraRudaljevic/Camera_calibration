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


vector<Vec3f> objectPoint(unsigned row, unsigned col, float squarSize);
Mat homographyDltSimEtimationImagePoints(vector<Vec2f>& vector);
Mat homographyDltSimEtimationObjectPoints(vector<Vec3f>& vector);
void homographyDltNormalizeImagePoints(vector<Vec2f>& point, Mat& S);
void homographyDltNormalizeObjectPoints(vector<Vec3f>& point, Mat& S);
Mat homographyDlt(vector<Vec2f> &imagePoints, vector<Vec3f> &objectPoints);
vector<float> V_ij(Mat H, int i, int j);
Mat getV(vector<Mat> &H);
bool intrinsics(Mat &B, float &u0, float &v0, float &lam, float &alpha, float &beta, float &gama);
Mat getIntrinsicParameters(vector<Mat>& H_r);
Mat intrinsicsDenormalize(Mat &K, Mat &N);
Mat getExtrinsicsParameters(Mat &K, Mat &H);
Mat normalizeImagePoints(vector<vector<Vec2f>> &points, int w, int h);
