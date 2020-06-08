#include <stdio.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;


vector<Point2f> objectPoint(unsigned row, unsigned col, float squarSize);
vector<Point2f> findchessboardCorners();
Matx33d normalizePoints(vector<vector<Point2f>>& corners);
Matx33d getIntrinsicParameters(vector<Matx33d> H);
Matx33d extrinsicCalculation(Matx33d intrinsic, Matx33d H);
vector<Matx34d> getExtrinsicsParameters(Matx33d intrinsic, vector<Matx33d> homographies);
