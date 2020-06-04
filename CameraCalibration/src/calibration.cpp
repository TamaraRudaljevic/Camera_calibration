#include "../include/calibration.hpp"

#define SQUARE_SIZE = 1.0
string readpath = "/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_";
Size patternsize(9,6);
vector<vector<Point2f>> imagePoints;
vector<vector<Point3f>> objectPoints;


vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize)
{
	vector<Point3f> objectPoint;

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			objectPoint.push_back(Point3f((float)j*squarSize, (float)i*squarSize, 0));
		}
	}

	return objectPoint;
}


vector<Point2f> findchessboardCorners()
{
	vector<Point2f> corners;
	vector<Point3f> objPoint = objectPoint(9, 6, 1.);
	bool patternfound;
	
	for (int i = 1; i < 14; i++)
	{
		Mat image = imread(readpath + to_string(i) + ".jpg");
		patternfound = findChessboardCorners(image, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (patternfound)
		{
			cout << "Corners found!" << endl;
			imagePoints.push_back(corners);
			objectPoints.push_back(objPoint);
		}
	}

	return corners;
}

Matx33d normalize(vector<vector<Vec2d>> &point, int width, int height)
{
	/*float s_x = 2. / width;
	float s_y = 2. / height;
	float x0 = width / 2.;
	float y0 = height / 2.;

	for (int i = 0; i < point.size(); i++)
	{
		for (int j = 0; j < point[i].size(); j++)
		{
			point[i][j][0] = s_x * (point[i][j][0] - x0);
			point[i][j][1] = s_y * (point[i][j][1] - x0);
		}
	}*/

	Point2f sum1;
	Point2f sum2;

	for (int i = 0; i < point.size(); i++)
	{
		sum1 += point[i][0];
		sum2 += point[i][1];
		//points1.push_back(point[i][0]);
		//points2.push_back(point[i][1]);
	} 

	Point2f avg_x = sum1 / float(point.size());
	Point2f avg_y = sum2 / float(point.size());

	//Point2f s_x = sqrt((point[0]));

	

}


