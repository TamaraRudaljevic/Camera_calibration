#include "../include/calibration.hpp"

#define SQUARE_SIZE = 1.0
string readpath = "/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_";
Size patternsize(9,6);
vector<vector<Point2f>> imagePoints;
vector<vector<Point2f>> objectPoints;
vector<vector<double>> tmp;
Mat n_x, n_u = Mat(3, 3, CV_64FC1);
Mat n_x_inv, n_u_inv = Mat(3, 3, CV_64FC1);


// Getting object point (world coord)
// vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize)
// {
// 	vector<Point3f> objectPoint;

// 	for (int i = 0; i < row; i++)
// 	{
// 		for (int j = 0; j < col; j++)
// 		{
// 			objectPoint.push_back(Point3f((float)j*squarSize, (float)i*squarSize, 0));
// 		}
// 	}

// 	return objectPoint;
// }

vector<Point2f> objectPoint(unsigned row, unsigned col, float squarSize)
{
	vector<Point2f> objectPoint;

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			objectPoint.push_back(Point2f((float)j*squarSize, (float)i*squarSize));
		}
	}

	return objectPoint;
}

// Detecting chessboard corners
vector<Point2f> findchessboardCorners()
{
	vector<Point2f> corners;
	vector<Point2f> objPoint = objectPoint(9, 6, 1.);
	bool patternfound;
	
	for (int i = 1; i < 14; i++)
	{
		Mat image = imread(readpath + to_string(i) + ".jpg");
		patternfound = findChessboardCorners(image, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (patternfound)
		{
			//cout << "Corners found!" << endl;
			imagePoints.push_back(corners);
			objectPoints.push_back(objPoint);
		}
	}

	return corners;
}

//****************************************************//

Mat homographyDltSimEtimation(vector<vector<Point2f>>& vector)
{
	float xMean, yMean, xDev, yDev, xVar, yVar, s_x, s_y;
	meanStdDev(vector[0], Scalar(xMean), Scalar(xDev));
	meanStdDev(vector[1], Scalar(yMean), Scalar(yDev));
	xVar = xDev*xDev;
	yVar = yDev*yDev;

	sqrt(2./xVar, Scalar(s_x));
	sqrt(2./yVar, Scalar(s_y));


	n_u.row(0).col(0) = s_x;
	n_u.row(0).col(1) = 0;
	n_u.row(0).col(1) = -s_x*xMean;
	n_u.row(0).col(1) = 0;
	n_u.row(0).col(1) = s_y;
	n_u.row(0).col(1) = -s_y*yMean;
	n_u.row(0).col(1) = 0;
	n_u.row(0).col(1) = 0;
	n_u.row(0).col(1) = 1;

	n_u_inv.row(0).col(0) = 1./s_x;
	n_u_inv.row(0).col(1) = 0;
	n_u_inv.row(0).col(1) = xMean;
	n_u_inv.row(0).col(1) = 0;
	n_u_inv.row(0).col(1) = 1./s_y;
	n_u_inv.row(0).col(1) = yMean;
	n_u_inv.row(0).col(1) = 0;
	n_u_inv.row(0).col(1) = 0;
	n_u_inv.row(0).col(1) = 1;
	

}

void homographyDltNormalize(vector<vector<Point2f>>& point, Mat& S)
{
	Mat x(3, 1, CV_32FC1), xp(3, 1, CV_32FC1);
	for(int i = 0; i < point.size(); i++)
	{
		Point2f(x.row(0).col(0)) = point[i][0];
		Point2f(x.row(1).col(0)) = point[i][1];
		x.row(2).col(0) = 1;
		xp = S.cross(x);
		point[i][0] = xp.at<Point2f>(0, 0) / xp.at<float>(2, 0);
		point[i][1] = xp.at<Point2f>(1, 0) / xp.at<float>(2, 0);
	}
}

Mat homographyDlt()
{
	Mat img, obj, invTgtS;
	Mat A = Mat::zeros(2*imagePoints.size(), 9, CV_64FC1);
	Mat H(3, 3, CV_64FC1);
	img = homographyDltSimEtimation(imagePoints);
	obj = homographyDltSimEtimation(objectPoints);


	invTgtS = obj.clone();
	invert(invTgtS, invTgtS);
	homographyDltNormalize(imagePoints, img);
	homographyDltNormalize(objectPoints, obj);

	for (int i = 0; i < imagePoints.size(); i++)
	{
		A.at<Point2f>(i*2 + 0, 0) = -1*imagePoints[i][0];
		A.at<Point2f>(i*2 + 0, 1) = -1*imagePoints[i][1];
		A.at<Point2f>(i*2 + 0, 2) = Point2f(-1);
		A.at<Point2f>(i*2 + 0, 3) = Point2f(0);
		A.at<Point2f>(i*2 + 0, 4) = Point2f(0);
		A.at<Point2f>(i*2 + 0, 5) = Point2f(0);
		A.at<Point2f>(i*2 + 0, 6) = imagePoints[i][0] * objectPoints[i][0];
		A.at<Point2f>(i*2 + 0, 7) = imagePoints[i][1] * objectPoints[i][0];
		A.at<Point2f>(i*2 + 0, 8) = objectPoints[i][0];

		A.at<Point2f>(i*2 + 1, 0) = Point2f(0);
		A.at<Point2f>(i*2 + 1, 1) = Point2f(0);
		A.at<Point2f>(i*2 + 1, 2) = Point2f(0);
		A.at<Point2f>(i*2 + 1, 3) = -1*imagePoints[i][0];
		A.at<Point2f>(i*2 + 1, 4) = -1*imagePoints[i][1];
		A.at<Point2f>(i*2 + 1, 5) = Point2f(-1);
		A.at<Point2f>(i*2 + 1, 6) = imagePoints[i][0] * objectPoints[i][1];
		A.at<Point2f>(i*2 + 1, 7) = imagePoints[i][1] * objectPoints[i][1];
		A.at<Point2f>(i*2 + 1, 8) = objectPoints[i][1];
	}

	Mat w, vt;
	vector<float> u;

	SVDecomp(A, w, u, vt);
	

	
}


//****************************************************//
// Matx33d normalizePoints()
// {
// 	int len = imagePoints.size() + objectPoints.size();
// 	vector<vector<Point>> normalizedHomImp, normalizedHomObjp;
// 	vector<Point> tmp;

// 	for (int i = 0; i < imagePoints.size(); i++)
// 	{
// 		tmp.clear();
// 		tmp.push_back(imagePoints[i][0]);
// 		tmp.push_back(imagePoints[i][1]);
// 		tmp.push_back(Point(1));
// 		normalizedHomImp.push_back(tmp);
// 	}


// 	for (int i = 0; i < objectPoints.size(); i++)
// 	{
// 		tmp.clear();
// 		tmp.push_back(objectPoints[i][0]);
// 		tmp.push_back(objectPoints[i][1]);
// 		tmp.push_back(Point(1));
// 		normalizedHomObjp.push_back(tmp);
// 	}

// 	for (int i = 0; i < len; i++)
// 	{
// 		for (int j = 0; j < normalizedHomObjp[0].size(); j++)
// 		{
// 			Mat n_o, src3;
// 			gemm(n_x, normalizedHomObjp[i], 1, src3, 0, n_o, 0);
// 			normalizedHomObjp[i] = n_o/n_o[-1];
// 		}
// 	}
// }

// void getNormalizationMatrixImagePoints(vector<vector<Point2f>>& vector)
// {
// 	float xMean, yMean, xDev, yDev, xVar, yVar, s_x, s_y;
// 	meanStdDev(vector[0], Scalar(xMean), Scalar(xDev));
// 	meanStdDev(vector[1], Scalar(yMean), Scalar(yDev));
// 	xVar = xDev*xDev;
// 	yVar = yDev*yDev;

// 	sqrt(2./xVar, Scalar(s_x));
// 	sqrt(2./yVar, Scalar(s_y));


// 	n_u.row(0).col(0) = s_x;
// 	n_u.row(0).col(1) = 0;
// 	n_u.row(0).col(1) = -s_x*xMean;
// 	n_u.row(0).col(1) = 0;
// 	n_u.row(0).col(1) = s_y;
// 	n_u.row(0).col(1) = -s_y*yMean;
// 	n_u.row(0).col(1) = 0;
// 	n_u.row(0).col(1) = 0;
// 	n_u.row(0).col(1) = 1;

// 	n_u_inv.row(0).col(0) = 1./s_x;
// 	n_u_inv.row(0).col(1) = 0;
// 	n_u_inv.row(0).col(1) = xMean;
// 	n_u_inv.row(0).col(1) = 0;
// 	n_u_inv.row(0).col(1) = 1./s_y;
// 	n_u_inv.row(0).col(1) = yMean;
// 	n_u_inv.row(0).col(1) = 0;
// 	n_u_inv.row(0).col(1) = 0;
// 	n_u_inv.row(0).col(1) = 1;
	 
// }

// void getNormalizationMatrixObjectPoints(vector<vector<Point2f>>& vector)
// {
// 	float xMean, yMean, xDev, yDev, xVar, yVar, s_x, s_y;
// 	meanStdDev(vector[0], Scalar(xMean), Scalar(xDev));
// 	meanStdDev(vector[1], Scalar(yMean), Scalar(yDev));
// 	xVar = xDev*xDev;
// 	yVar = yDev*yDev;

// 	sqrt(2./xVar, Scalar(s_x));
// 	sqrt(2./yVar, Scalar(s_y));


// 	n_x.row(0).col(0) = s_x;
// 	n_x.row(0).col(1) = 0;
// 	n_x.row(0).col(1) = -s_x*xMean;
// 	n_x.row(0).col(1) = 0;
// 	n_x.row(0).col(1) = s_y;
// 	n_x.row(0).col(1) = -s_y*yMean;
// 	n_x.row(0).col(1) = 0;
// 	n_x.row(0).col(1) = 0;
// 	n_x.row(0).col(1) = 1;

// 	n_x_inv.row(0).col(0) = 1./s_x;
// 	n_x_inv.row(0).col(1) = 0;
// 	n_x_inv.row(0).col(1) = xMean;
// 	n_x_inv.row(0).col(1) = 0;
// 	n_x_inv.row(0).col(1) = 1./s_y;
// 	n_x_inv.row(0).col(1) = yMean;
// 	n_x_inv.row(0).col(1) = 0;
// 	n_x_inv.row(0).col(1) = 0;
// 	n_x_inv.row(0).col(1) = 1;
	 
// }
//****************************************************//

	// Matx33d M;

	// double x_mean, y_mean, dev1, dev2;
	// meanStdDev(corners[0], Scalar(x_mean),  Scalar(dev1));
	// meanStdDev(corners[1], Scalar(y_mean),  Scalar(dev2));
	// double s_x, s_y;
	// sqrt(Scalar(2./dev1), Scalar(s_x));
	// sqrt(Scalar(2./dev2), Scalar(s_y));

	
	// M(0, 0) = s_x;
	// M(0, 1) = 0;
	// M(0, 2) = -s_x*x_mean;
	// M(1, 0) = 0;
	// M(1, 1) = s_y;
	// M(1, 2) = -s_y*y_mean;
	// M(2, 0) = 0;
	// M(2, 1) = 0;
	// M(2, 2) = 1;

	// return M;
	


//*****************************INTRINSIC***********************//

vector<float> V_ij(int i, int j, Matx33d H)
{
	vector<float> v;
	v.push_back(H(0, i)*H(0, j));
	v.push_back(H(0, i)*H(1, j) + H(1, i)*H(0, j));
	v.push_back(H(1, i)*H(1, j));
	v.push_back(H(2, i)*H(0, j)*H(0, i)*H(2, j));
	v.push_back(H(2, i)*H(1, j) + H(1, i)*H(2, j));
	v.push_back(H(2, i)*H(2, j));
	return v;
}

// Mat computeViewBasedHomography(vector<vector<Point2f>> normalizeImagePoints, vector<vector<Point3f>> normalizeObjectPoints, int len)
// {
// 	Mat M(2*len, 9, CV_64FC2);

// 	for (int i = 0; i < len; i++)
// 	{
// 		Point3f X, Y;
// 		Point2f u, v;
// 		X = normalizeObjectPoints[i][0];
// 		Y = normalizeObjectPoints[i][1];
// 		u = normalizeImagePoints[i][0];
// 		v = normalizeImagePoints[i][1];
// 		vector<float> row1, row2;
// 		row1.push_back(-X);
// 		row1.push_back(-Y);
// 		row1.push_back(-1);
// 		row1.push_back(0);
// 		row1.push_back(0);
// 		row1.push_back(0);
// 		row1.push_back(X*u);
// 		row1.push_back(Y*u);
// 		row1.push_back(u);

// 		row2.push_back(0);
// 		row2.push_back(0);
// 		row2.push_back(0);
// 		row2.push_back(-X);
// 		row2.push_back(-Y);
// 		row2.push_back(-1);
// 		row2.push_back(X*v);
// 		row2.push_back(Y*v);
// 		row2.push_back(v);

// 		//M(2*i, 0) = row1;
// 	}
// }

// Matx33d getIntrinsicParameters(vector<Matx33d> H_r)
// {
// 	int M = H_r.size();
// 	Mat v = Mat(2*M, 6, CV_64F);

// 	for (int i = 0; i < M; i++)
// 	{
// 		Matx33d H = H_r[i];
// 		v(2*i, 0) = V_ij(0, 1, H);
// 		subtract(V_ij(0, 0, H), V_ij(1, 1, H), v(2*i, 1));
// 	}

// 	Mat s, u, vt;
// 	SVDecomp(v, s, u, vt);


// }

//*********************HOMOGRAPHY*************************//


// vector<Matx33d> arrayHomographies(vector<vector<Point2f>>& imagePoints, vector<vector<Point3f>>& objectPoints)
// {
// 	vector<Matx33d> refinedHomographies;

// 	cv::Matx33d H;

// 	// 0. Prepare data;
// 	cv::Matx33d srcS, tgtS, invTgtS;
// 	cv::Matx33d A;

// 	// 1. Perform normalization;
// 	srcS = homography_dlt_sim_estimation<2>(imagePoints);
// 	tgtS = homography_dlt_sim_estimation<3>(objectPoints);

// 	auto src_n = imagePoints; // source normalized points
// 	auto tgt_n = objectPoints; // target normalized points

// 	invTgtS = tgtS.clone();
// 	invert(invTgtS);

// 	homography_dlt_normalize<2>(src_n, srcS);
// 	homography_dlt_normalize<3>(tgt_n, tgtS);

// 	// 2. Pack matrix A;
// 	for (unsigned i = 0; i < imagePoints.size(); ++i) {
// 		Point2f(A(i * 2 + 0, 0)) = -1 * src_n[i][0];
// 		Point2f(A(i * 2 + 0, 1)) = -1 * src_n[i][1];
// 		A(i * 2 + 0, 2) = -1;
// 		Point3f(A(i * 2 + 0, 6)) = tgt_n[i][0] * src_n[i][0];
// 		A(i * 2 + 0, 7) = tgt_n[i][0] * src_n[i][1];
// 		Point3f(A(i * 2 + 0, 8)) = tgt_n[i][0];

// 		A(i * 2 + 1, 3) = -1 * src_n[i][0];
// 		A(i * 2 + 1, 4) = -1 * src_n[i][1];
// 		A(i * 2 + 1, 5) = -1;
// 		A(i * 2 + 1, 6) = tgt_n[i][1] * src_n[i][0];
// 		A(i * 2 + 1, 7) = tgt_n[i][1] * src_n[i][1];
// 		A(i * 2 + 1, 8) = tgt_n[i][1];
// 	}

// 	// 3. solve nullspace of A for H;
// 	cv::null_solve(A, H);

// 	H.reshape(3, 3);

// 	// 4. denormalize the homography.
// 	H = invTgtS * H * srcS;

// 	return H;
	
// }

// //template<size_t _size>
// void homographyDltNormalize2f(vector<vector<Point2f> > &features, const Matx33d &S) {
// 	//ASSERT(S && S.rows() == 3 && S.cols() == 3);
// 	Matx31f x(3, 1), xp(3, 1);
// 	for (unsigned i = 0; i < features.size(); ++i) {
// 		Point2f(x(0, 0)) = features[i][0];
// 		Point2f(x(1, 0)) = features[i][1];
// 		x(2, 0) = 1.;
// 		//cross(S, x, xp);
// 		features[i][0] = Point2f(xp(0, 0) / xp(2, 0));
// 		features[i][1] = Point2f(xp(1, 0) / xp(2, 0));
// 	}
// }

// void homographyDltNormalize3f(vector<vector<Point3f> > &features, const Matx33d &S) {
// 	//ASSERT(S && S.rows() == 3 && S.cols() == 3);
// 	Matx31f x(3, 1), xp(3, 1);
// 	for (unsigned i = 0; i < features.size(); ++i) {
// 		Point3f(x(0, 0)) = features[i][0];
// 		Point3f(x(1, 0)) = features[i][1];
// 		Point3f(x(2, 0)) = features[i][2];
// 		//cross(S, x, xp);
// 		features[i][0] = Point3f(xp(0, 0) / xp(2, 0));
// 		features[i][1] = Point3f(xp(1, 0) / xp(2, 0));
// 		features[i][2] = Point3f(1.);
// 	}
// }


// Matx33d homography_dlt_sim_estimation(const vector<vector<Point2f> > &features) {
// 	Mat transform = Mat::eye(3, 3, CV_64F);
	

// 	size_t centroid(0, 0);
// 	Mat S;

// 	for (auto feat : features) {
// 		centroid += feat;
// 	}
// 	centroid /= features.size();

// 	Point2f sum_dist = 0;

// 	for (auto feat : features) {
// 		sum_dist+= centroid.distance(feat);
// 	}
// 	centroid *= -1;

// 	Point2f scale_v = std::sqrt(2.) / (sum_dist / features.size());

// 	transform(0, 0) = scale_v;
// 	transform(1, 1) = scale_v;
// 	transform(0, 2) = centroid[0];
// 	transform(1, 2) = centroid[1];

// 	return transform;
// }

//*****************************EXTRINSICS***********************//

