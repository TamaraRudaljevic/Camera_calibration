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
Mat homographyDlt(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints);
Mat imagePointNomalizationMatrix(vector<Point2f> &imagePoints, Mat &imgInv);
Mat objectPointNomalizationMatrix(vector<Point3f> &objectPoints);
void normalizeImagePoints(vector<Point2f>& point, Mat& S);
void normalizeObjectPoints(vector<Point3f>& point, Mat& S);