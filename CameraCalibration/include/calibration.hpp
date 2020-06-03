#include <stdio.h>
#include <vector>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;

#define SQUARE_SIZE = 1.0

vector<Vec3d> objectPoints(unsigned rows, unsigned cols);
Mat normalizePoint(vector<vector<Vec2d>> chessboardCorrespondences, unsigned w, unsigned h);
Mat denormalizeIntrinsics(const Mat &K, const Mat &N);
Mat findIntrisicsParameters(const vector<Mat> &H);
Mat findExtrinsics(const Mat &K, const Mat &H);
Mat homographyDlt(vector<Vec2d> &chessboardCorrespondencesNormalize);

