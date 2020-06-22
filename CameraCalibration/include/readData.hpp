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
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d_c.h>

using namespace std;
using namespace cv;


//bool readZhang(string readPathZhang, vector<vector<Point2f>> &imagePointsNorm, vector<Point3f> &modelPoints, int &w, int &h);
void readImages(string readPathImages, vector<vector<Point2f>> &imagePoints, vector<Point3f> &objectPoints, int &w, int &h);
// bool readData(string readPath, vector<vector<Vec2f>> &imagePointNorm, vector<Vec3f> &modelPoints, int &w, int &h, float modelSize);
// vector<string> split_string(string s, string del);
// vector<vector<Vec2f>> readPattern(string readPattern, int &w, int &h);