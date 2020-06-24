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
Mat homographyDlt(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints);
Mat V_ij(Mat H, int i, int j);
Mat getV(vector<Mat> &H);
bool intrinsics(Mat &B, float &u0, float &v0, float &lam, float &alpha, float &beta, float &gama);
Mat getIntrinsicParameters(vector<Mat>& H_r);
Mat intrinsicsDenormalize(Mat &K, Mat &N);
Mat getExtrinsicsParameters(Mat &K, Mat &H);
Mat normalizeImagePoints(vector<vector<Point2f>> &points, int w, int h);
Mat distortion(vector<vector<Point2f>> &imagePoints, vector<vector<Point2f>> &imagePointsNorm, vector<vector<Point2f>> &imageProj, Mat &K);

Mat imagePointNomalizationMatrix(vector<Point2f> &imagePoints, Mat &objInv);
Mat objectPointNomalizationMatrix(vector<Point3f> &objectPoints);
// Point2f meanObjectPoint(vector<Point3f> &objectPoints);
// Point2f meanImagePoint(vector<Point2f> &imagePoints);
// Point2f varianceImagePoints(vector<Point2f> &imagePoints, float xMean, float yMean);
// Point2f varianceObjectPoints(vector<Point3f> &objectPoints, float xMean, float yMean);


// Mat homographyLeastSquares(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints);
// void pack_ab(vector<Point2f> &src_pts, vector<Point3f> &tgt_pts, Mat &A, Mat &B);