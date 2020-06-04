#include <stdio.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <string>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;


Matx33d normalizePoint(vector<vector<Point2f>> point, int width, int height);
vector<Point2f> findchessboardCorners();

