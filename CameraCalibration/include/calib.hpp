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

Mat homographyDltEstimationImg(vector<Vec2f>& imgPoints);
Mat homographyDltEstimationObj(vector<Vec3f>& objPoints);
void homographyDltNormalizeImg(vector<Vec2f>& imgPoints, Mat& S);
void homographyDltNormalizeObj(vector<Vec3f>& objPoints, Mat& S);
Mat homographyDlt(vector<Vec2f>& src_pts, vector<Vec3f>& tgt_pts);
vector<Vec3f> obj(unsigned row, unsigned col, float squarSize);
vector<vector<Vec2f>> imageCorners();
Mat normalizeImagePoints(vector<vector<Vec2f>> &points, unsigned w, unsigned h);
