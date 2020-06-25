#include "../include/homography.hpp"


vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize)
{
	vector<Point3f> objectPoint;
	objectPoint.reserve(row*col);

	for (unsigned i = 0; i < col; i++)
	{
		for (unsigned j = 0; j < row; j++)
		{
			objectPoint.push_back({static_cast<float>(j*squarSize), static_cast<float>(i*squarSize), 1.});
		}
	}

	return objectPoint;
}

void normalizeImagePoints(vector<Point2f>& point, Mat& S)
{
	Mat x = Mat::zeros(3, 1, CV_32FC1), xp = Mat::zeros(3, 1, CV_32FC1);
	Mat newPoint(3, point.size(), CV_32FC1);
	for(unsigned i = 0; i < point.size(); i++)
	{
		x.at<float>(0, 0) = point[i].x;
		x.at<float>(1, 0) = point[i].y;
		x.at<float>(2, 0) = 1.;

		xp = S * x;

		point[i].x = xp.at<float>(0, 0) / xp.at<float>(2, 0);
		point[i].y = xp.at<float>(1, 0) / xp.at<float>(2, 0);
	}
}

void normalizeObjectPoints(vector<Point3f>& point, Mat& S)
{
	Mat x = Mat::zeros(3, 1, CV_32FC1), xp = Mat::zeros(3, 1, CV_32FC1);
	for(unsigned i = 0; i < point.size(); i++)
	{
		x.at<float>(0, 0) = point[i].x;
		x.at<float>(1, 0) = point[i].y;
		x.at<float>(2, 0) = point[i].z;

		xp = S * x;

		point[i].x = xp.at<float>(0, 0) / xp.at<float>(2, 0);
		point[i].y = xp.at<float>(1, 0) / xp.at<float>(2, 0);
		point[i].z = 1;
	}
}

Mat homographyDlt(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints)
{
	Mat img(3, 3, CV_32FC1), obj(3, 3, CV_32FC1);
	Mat imgInv = Mat::eye(3, 3, CV_32FC1);
	Mat A = Mat::zeros(2*imagePoints.size(), 9, CV_32FC1);
	Mat H(3, 3, CV_32FC1);
	img = imagePointNomalizationMatrix(imagePoints, imgInv);
	obj = objectPointNomalizationMatrix(objectPoints);

	normalizeImagePoints(imagePoints, img);
	normalizeObjectPoints(objectPoints, obj);

	for (unsigned i = 0; i < imagePoints.size(); i++)
		{
			A.at<float>(i*2 + 0, 0) = -1*objectPoints[i].x;
			A.at<float>(i*2 + 0, 1) = -1*objectPoints[i].y;
			A.at<float>(i*2 + 0, 2) = -1.;
			A.at<float>(i*2 + 0, 3) = 0.;
			A.at<float>(i*2 + 0, 4) = 0.;
			A.at<float>(i*2 + 0, 5) = 0.;
			A.at<float>(i*2 + 0, 6) = objectPoints[i].x * imagePoints[i].x;
			A.at<float>(i*2 + 0, 7) = objectPoints[i].y * imagePoints[i].x;
			A.at<float>(i*2 + 0, 8) = imagePoints[i].x;

			A.at<float>(i*2 + 1, 0) = 0.;
			A.at<float>(i*2 + 1, 1) = 0.;
			A.at<float>(i*2 + 1, 2) = 0.;
			A.at<float>(i*2 + 1, 3) = -1*objectPoints[i].x;
			A.at<float>(i*2 + 1, 4) = -1*objectPoints[i].y;
			A.at<float>(i*2 + 1, 5) = -1.;
			A.at<float>(i*2 + 1, 6) = objectPoints[i].x * imagePoints[i].y;
			A.at<float>(i*2 + 1, 7) = objectPoints[i].y * imagePoints[i].y;
			A.at<float>(i*2 + 1, 8) = imagePoints[i].y;
			
	}
	Mat S, U, VT;
	
	SVDecomp(A, U, S, VT);
	Mat Vtransp;
	Vtransp = VT.t();
	
	Mat h(3, 3, CV_32FC1);

	h.at<float>(0, 0) = Vtransp.at<float>(0, 8);
	h.at<float>(0, 1) = Vtransp.at<float>(1, 8);
	h.at<float>(0, 2) = Vtransp.at<float>(2, 8);
	h.at<float>(1, 0) = Vtransp.at<float>(3, 8);
	h.at<float>(1, 1) = Vtransp.at<float>(4, 8);
	h.at<float>(1, 2) = Vtransp.at<float>(5, 8);
	h.at<float>(2, 0) = Vtransp.at<float>(6, 8);
	h.at<float>(2, 1) = Vtransp.at<float>(7, 8);
	h.at<float>(2, 2) = Vtransp.at<float>(8, 8);

	h = imgInv * h * img;

	return h;
}

Mat imagePointNomalizationMatrix(vector<Point2f> &imagePoints, Mat &imgInv)
{
	Mat matrix = Mat::eye(3, 3, CV_32FC1);
	Point2f center(0, 0);
	Mat S;
	for (auto veec : imagePoints)
	{
		center.x += veec.x;
		center.y += veec.y;
	}

	center.x /= imagePoints.size(); // x mean
	center.y /= imagePoints.size(); // y mean

	Point2f sum(0, 0);


	for (auto veec : imagePoints)
	{
		sum.x += (veec.x - center.x) * (veec.x - center.x);
		sum.y += (veec.y - center.y) * (veec.y - center.y);
	}

	Point2f var(0, 0);
	var.x = sum.x / imagePoints.size(); 
	var.y = sum.y / imagePoints.size(); 
	

	// standard deviation square root of variance
	Point2f sd(0, 0);
	sd.x = sqrt(var.x);
	sd.y = sqrt(var.y);

	float s_x = sqrt(2. / var.x);
	float s_y = sqrt(2. / var.y);

	
	matrix.row(0).col(0) = s_x;
	matrix.row(1).col(1) = s_y;
	matrix.row(0).col(2) = -s_x*center.x;
	matrix.row(1).col(2) = -s_y*center.y;

	imgInv.row(0).col(0) = 1. / s_x;
	imgInv.row(1).col(1) = 1. / s_y;
	imgInv.row(0).col(2) = center.x;
	imgInv.row(1).col(2) = center.y;
	return matrix;

}

Mat objectPointNomalizationMatrix(vector<Point3f> &objectPoints)
{
	Mat matrix = Mat::eye(3, 3, CV_32FC1);
	Point3f center(0, 0, 0);
	Mat S;
	for (auto veec : objectPoints)
	{
		center.x += veec.x;
		center.y += veec.y;
	}

	center.x /= objectPoints.size(); // x maen
	center.y /= objectPoints.size(); // y mean

	Point2f sum(0, 0);

	for (auto veec :objectPoints)
	{
		sum.x += (veec.x - center.x) * (veec.x - center.x);
		sum.y += (veec.y - center.y) * (veec.y - center.y);
	}

	Point2f var(0, 0);
	var.x = sum.x / objectPoints.size(); 
	var.y = sum.y / objectPoints.size(); 

	float s_x = sqrt(2. / var.x);
	float s_y = sqrt(2. / var.y);	
	
	matrix.row(0).col(0) = s_x;
	matrix.row(1).col(1) = s_y;
	matrix.row(0).col(2) = -s_x*center.x;
	matrix.row(1).col(2) = -s_y*center.y;

	
	return matrix;
}
