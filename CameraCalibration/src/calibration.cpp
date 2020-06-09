#include "../include/calibration.hpp"

#define SQUARE_SIZE = 1.0
string readpath = "/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_";
Size patternsize(9,6);
vector<Point2f> imagePoints;
vector<Point3f> objectPoints;
vector<vector<double>> tmp;
Mat n_x, n_u = Mat(3, 3, CV_64FC1);
Mat n_x_inv, n_u_inv = Mat(3, 3, CV_64FC1);
Mat K;
vector<Mat> extrinsic;


//Getting object point (world coord)
vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize)
{
	vector<Point3f> objectPoint;

	for (unsigned i = 0; i < row; i++)
	{
		for (unsigned j = 0; j < col; j++)
		{
			objectPoint.push_back(Point3f((float)j*squarSize, (float)i*squarSize, 0));
		}
	}

	return objectPoint;
}

// vector<Point2f> objectPoint(unsigned row, unsigned col, float squarSize)
// {
// 	vector<Point2f> objectPoint;

// 	for (unsigned i = 0; i < row; i++)
// 	{
// 		for (unsigned j = 0; j < col; j++)
// 		{
// 			objectPoint.push_back(Point2f((float)j*squarSize, (float)i*squarSize));
// 		}
// 	}

// 	return objectPoint;
// }

// Detecting chessboard corners
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
			imagePoints = corners;
			objectPoints = objPoint;
		}
	}

	return corners;
}

// //****************************************************//

Mat homographyDltSimEtimationImagePoints(vector<Point2f>& vector)
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
	n_u.row(0).col(2) = -s_x*xMean;
	n_u.row(1).col(0) = 0;
	n_u.row(1).col(1) = s_y;
	n_u.row(1).col(2) = -s_y*yMean;
	n_u.row(2).col(0) = 0;
	n_u.row(2).col(1) = 0;
	n_u.row(2).col(2) = 1;

	return n_u;

	// n_u_inv.row(0).col(0) = 1./s_x;
	// n_u_inv.row(0).col(1) = 0;
	// n_u_inv.row(0).col(1) = xMean;
	// n_u_inv.row(0).col(1) = 0;
	// n_u_inv.row(0).col(1) = 1./s_y;
	// n_u_inv.row(0).col(1) = yMean;
	// n_u_inv.row(0).col(1) = 0;
	// n_u_inv.row(0).col(1) = 0;
	// n_u_inv.row(0).col(1) = 1;
	

}

Mat homographyDltSimEtimationObjectPoints(vector<Point3f>& vector)
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
	n_u.row(0).col(2) = -s_x*xMean;
	n_u.row(1).col(0) = 0;
	n_u.row(1).col(1) = s_y;
	n_u.row(1).col(2) = -s_y*yMean;
	n_u.row(2).col(0) = 0;
	n_u.row(2).col(1) = 0;
	n_u.row(2).col(2) = 1;

	return n_u;
}

void homographyDltNormalizeImagePoints(vector<Point2f>& point, Mat& S)
{
	Mat x(3, 1, CV_32FC1), xp(3, 1, CV_32FC1);
	for(int i = 0; i < point.size(); i++)
	{
		x.row(0).col(0) = point[i].x;
		x.row(1).col(0) = point[i].y;
		x.row(2).col(0) = 1;
		xp = S.cross(x);
		point[i].x = xp.at<float>(0, 0) / xp.at<float>(2, 0);
		point[i].y = xp.at<float>(1, 0) / xp.at<float>(2, 0);
	}
}

void homographyDltNormalizeObjectPoints(vector<Point3f>& point, Mat& S)
{
	Mat x(3, 1, CV_32FC1), xp(3, 1, CV_32FC1);
	for(int i = 0; i < point.size(); i++)
	{
		x.row(0).col(0) = point[i].x;
		x.row(1).col(0) = point[i].y;
		x.row(2).col(0) = point[i].z;
		xp = S.cross(x);
		point[i].x = xp.at<float>(0, 0) / xp.at<float>(2, 0);
		point[i].y = xp.at<float>(1, 0) / xp.at<float>(2, 0);
		point[i].z = 1;
	}
}

Mat homographyDlt()
{
	Mat img, obj, invTgtS;
	Mat A = Mat::zeros(2*imagePoints.size(), 9, CV_64FC1);
	Mat H;
	img = homographyDltSimEtimationImagePoints(imagePoints);
	obj = homographyDltSimEtimationObjectPoints(objectPoints);

	auto src_n = imagePoints;
	auto tgt_n = objectPoints; 

	invTgtS = obj.clone();
	invert(invTgtS, invTgtS);

	homographyDltNormalizeImagePoints(src_n, img);
	homographyDltNormalizeObjectPoints(tgt_n, obj);

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		A.at<float>(i*2 + 0, 0) = -1*src_n[i].x;
		A.at<float>(i*2 + 0, 1) = -1*src_n[i].y;
		A.at<float>(i*2 + 0, 2) = -1;
		A.at<float>(i*2 + 0, 3) = 0;
		A.at<float>(i*2 + 0, 4) = 0;
		A.at<float>(i*2 + 0, 5) = 0;
		A.at<float>(i*2 + 0, 6) = src_n[i].x * tgt_n[i].x;
		A.at<float>(i*2 + 0, 7) = src_n[i].y * tgt_n[i].x;
		A.at<float>(i*2 + 0, 8) = tgt_n[i].x;

		A.at<float>(i*2 + 1, 0) = 0;
		A.at<float>(i*2 + 1, 1) = 0;
		A.at<float>(i*2 + 1, 2) = 0;
		A.at<float>(i*2 + 1, 3) = -1*src_n[i].x;
		A.at<float>(i*2 + 1, 4) = -1*src_n[i].y;
		A.at<float>(i*2 + 1, 5) = -1;
		A.at<float>(i*2 + 1, 6) = src_n[i].x * tgt_n[i].y;
		A.at<float>(i*2 + 1, 7) = src_n[i].y * tgt_n[i].y;
		A.at<float>(i*2 + 1, 8) = tgt_n[i].y;
		
	}

	Mat w;
	vector<Mat> vt;
	vector<float> u;

	SVDecomp(A, w, u, vt);
	
	Mat h_norm = vt[argmin(u)];
	h_norm.reshape(3, 3);

	H = invTgtS * h_norm * img;
	return H;

	
}

int argmin(vector<float>& vector)
{
	std::vector<float>::iterator iterator = std::min_element(vector.begin(), vector.end());
	int min = iterator[0];
	for (unsigned i = 0; i < vector.size(); i++)
	{
		if(vector[i] == min)
			return i;
	}
	return -1;
}

// //************************CAMERA MATRIX****************************//

// vector<float> V_ij(int i, int j, Matx33d H)
// {
// 	vector<float> v;
// 	v.push_back(H(0, i)*H(0, j));
// 	v.push_back(H(0, i)*H(1, j) + H(1, i)*H(0, j));
// 	v.push_back(H(1, i)*H(1, j));
// 	v.push_back(H(2, i)*H(0, j)*H(0, i)*H(2, j));
// 	v.push_back(H(2, i)*H(1, j) + H(1, i)*H(2, j));
// 	v.push_back(H(2, i)*H(2, j));
// 	return v;
// }

// Mat getIntrinsicParameters(vector<Mat>& H_r)
// {
// 	int M = H_r.size();
// 	Mat H, V = Mat(2*M, 6, CV_64FC1), u;
// 	vector<Mat> vt;
// 	vector<float> s;

// 	for (int i = 0; i < M; i++)
// 	{
// 		vector<float> h_r_1 = V.row(i*2);
// 		vector<float> h_r_2 = V.row(i*2 + 1);

// 		auto v12 = V_ij(0, 1, H_r[i]);
// 		auto v11 = V_ij(0, 0, H_r[i]);
// 		auto v22 = V_ij(1, 1, H_r[i]);
// 		vector<float> v11_v22;
// 		subtract(v11, v22, v11_v22);

// 		for (int i = 0; i < v12.size(); i++)
// 		{
// 			V.row(i*2).col(i) = v12[i];
// 		}

// 		for (int i = 0; i < v11_v22.size(); i++)
// 		{
// 			V.row(i*2 + 1).col(i) = v11_v22[i]; 
// 		}
// 	}

// 	SVDecomp(V, u, s, vt);
// 	vector<float> b = vt[argmin(s)];

// 	float w = b[0] * b[2] * b[5] - b[1]*b[1] * b[5] - b[0] * b[4]*b[4] + 2 * b[1] * b[3] * b[4] - b[2] * b[3]*b[3];
// 	float d = b[0] * b[2] - b[1]*b[1];

// 	float alpha, beta, gama, tmp, uc, vc;
// 	sqrt(Scalar(w / (d * b[0])), Scalar(alpha));
// 	sqrt(Scalar(w / d*d*b[0]), Scalar(beta));
// 	sqrt(Scalar(w / (d*d*b[0])), Scalar(tmp));
// 	gama = tmp * b[1];
// 	uc = (b[1]*b[4] - b[2]*b[3]) / d;
// 	vc = (b[1]*b[3] - b[0]*b[4]) / d;

// 	Mat K = Mat(3, 3, CV_64FC1);
// 	K.row(0).col(0) = alpha;
// 	K.row(0).col(0) = gama;
// 	K.row(0).col(0) = uc;
// 	K.row(0).col(0) = 0;
// 	K.row(0).col(0) = beta;
// 	K.row(0).col(0) = vc;
// 	K.row(0).col(0) = 0;
// 	K.row(0).col(0) = 0;
// 	K.row(0).col(0) = 1;

// 	return K;

// }

// Mat extrinsicsCalculation(Mat& intrinsic, Mat& H_r)
// {
// 	Mat homography = H_r.reshape(3, 3);
// 	Mat intrinsicInv; 
// 	invert(intrinsic, intrinsicInv);

// 	vector<float> h1, h2, h3;
// 	h1 = homography.col(0);
// 	h2 = homography.col(1);
// 	h3 = homography.col(2);

// 	float lam_r1, lam_r2, Lam_r3 r3;
// 	vector<float> r1, r2, t;
// 	lam_r1 = 1 / normalize(dot(intrinsicInv, h1));
// 	lam_r2 = 1 / normalize(dot(intrinsicInv, h2));
// 	lam_r3 = (lam_r1 + lam_r2) / 2;
// 	r1 = lam_r1 * dot(intrinsicInv, h1);
// 	r2 = lam_r2 * dot(intrinsicInv, h2);
// 	r3 = r1.cross(r2);
// 	t = transpose((Lam_r3 * dot(intrinsicInv , h3)));
	
// 	Mat rt;

// 	rt.row(0).col(0) = r1[0];
// 	rt.row(0).col(1) = r2[0];
// 	rt.row(0).col(2) = r3[0];
// 	rt.row(0).col(3) = t[0];
// 	rt.row(1).col(0) = r1[1];
// 	rt.row(1).col(1) = r2[1];
// 	rt.row(1).col(2) = r3[1];
// 	rt.row(1).col(3) = t[1];
// 	rt.row(2).col(0) = r1[2];
// 	rt.row(2).col(1) = r2[2];
// 	rt.row(2).col(2) = r3[2];
// 	rt.row(2).col(3) = t[2];

// 	return rt;
	
// }


// // Calculating dot product from matrices A and B
// vector<float> dot(Mat& A, Mat& B)
// {
// 	vector<float> dot;

// 	for (int i = 0; i < A.size(); i++)
// 	{
// 		dot[i] = 0;
// 		for (int j = 0; j < B.size(); j++)
// 		{
// 			dot[i] += A.row(i).col(i) * B.row(i).col(i);
// 		}
// 	}

// 	return dot;
// }

// vector<Mat> getExtrinsicsParameters(Mat& intrinsic, vector<Mat>& homographies)
// {
// 	vector<Mat> extrinsics;
// 	Mat rt;

// 	for (int i = 0; i < homographies.size(); i++)
// 	{
// 		rt = extrinsicsCalculation(intrinsic, homographies[i]);
// 		extrinsics.push_back(rt);
// 	}

// 	return extrinsics;
// }


// void calibration()
// {
	

	
// }



// //****************************************************//
// // Matx33d normalizePoints()
// // {
// // 	int len = imagePoints.size() + objectPoints.size();
// // 	vector<vector<Point>> normalizedHomImp, normalizedHomObjp;
// // 	vector<Point> tmp;

// // 	for (int i = 0; i < imagePoints.size(); i++)
// // 	{
// // 		tmp.clear();
// // 		tmp.push_back(imagePoints[i][0]);
// // 		tmp.push_back(imagePoints[i][1]);
// // 		tmp.push_back(Point(1));
// // 		normalizedHomImp.push_back(tmp);
// // 	}


// // 	for (int i = 0; i < objectPoints.size(); i++)
// // 	{
// // 		tmp.clear();
// // 		tmp.push_back(objectPoints[i][0]);
// // 		tmp.push_back(objectPoints[i][1]);
// // 		tmp.push_back(Point(1));
// // 		normalizedHomObjp.push_back(tmp);
// // 	}

// // 	for (int i = 0; i < len; i++)
// // 	{
// // 		for (int j = 0; j < normalizedHomObjp[0].size(); j++)
// // 		{
// // 			Mat n_o, src3;
// // 			gemm(n_x, normalizedHomObjp[i], 1, src3, 0, n_o, 0);
// // 			normalizedHomObjp[i] = n_o/n_o[-1];
// // 		}
// // 	}
// // }

// // void getNormalizationMatrixImagePoints(vector<vector<Point2f>>& vector)
// // {
// // 	float xMean, yMean, xDev, yDev, xVar, yVar, s_x, s_y;
// // 	meanStdDev(vector[0], Scalar(xMean), Scalar(xDev));
// // 	meanStdDev(vector[1], Scalar(yMean), Scalar(yDev));
// // 	xVar = xDev*xDev;
// // 	yVar = yDev*yDev;

// // 	sqrt(2./xVar, Scalar(s_x));
// // 	sqrt(2./yVar, Scalar(s_y));


// // 	n_u.row(0).col(0) = s_x;
// // 	n_u.row(0).col(1) = 0;
// // 	n_u.row(0).col(1) = -s_x*xMean;
// // 	n_u.row(0).col(1) = 0;
// // 	n_u.row(0).col(1) = s_y;
// // 	n_u.row(0).col(1) = -s_y*yMean;
// // 	n_u.row(0).col(1) = 0;
// // 	n_u.row(0).col(1) = 0;
// // 	n_u.row(0).col(1) = 1;

// // 	n_u_inv.row(0).col(0) = 1./s_x;
// // 	n_u_inv.row(0).col(1) = 0;
// // 	n_u_inv.row(0).col(1) = xMean;
// // 	n_u_inv.row(0).col(1) = 0;
// // 	n_u_inv.row(0).col(1) = 1./s_y;
// // 	n_u_inv.row(0).col(1) = yMean;
// // 	n_u_inv.row(0).col(1) = 0;
// // 	n_u_inv.row(0).col(1) = 0;
// // 	n_u_inv.row(0).col(1) = 1;
	 
// // }

// // void getNormalizationMatrixObjectPoints(vector<vector<Point2f>>& vector)
// // {
// // 	float xMean, yMean, xDev, yDev, xVar, yVar, s_x, s_y;
// // 	meanStdDev(vector[0], Scalar(xMean), Scalar(xDev));
// // 	meanStdDev(vector[1], Scalar(yMean), Scalar(yDev));
// // 	xVar = xDev*xDev;
// // 	yVar = yDev*yDev;

// // 	sqrt(2./xVar, Scalar(s_x));
// // 	sqrt(2./yVar, Scalar(s_y));


// // 	n_x.row(0).col(0) = s_x;
// // 	n_x.row(0).col(1) = 0;
// // 	n_x.row(0).col(1) = -s_x*xMean;
// // 	n_x.row(0).col(1) = 0;
// // 	n_x.row(0).col(1) = s_y;
// // 	n_x.row(0).col(1) = -s_y*yMean;
// // 	n_x.row(0).col(1) = 0;
// // 	n_x.row(0).col(1) = 0;
// // 	n_x.row(0).col(1) = 1;

// // 	n_x_inv.row(0).col(0) = 1./s_x;
// // 	n_x_inv.row(0).col(1) = 0;
// // 	n_x_inv.row(0).col(1) = xMean;
// // 	n_x_inv.row(0).col(1) = 0;
// // 	n_x_inv.row(0).col(1) = 1./s_y;
// // 	n_x_inv.row(0).col(1) = yMean;
// // 	n_x_inv.row(0).col(1) = 0;
// // 	n_x_inv.row(0).col(1) = 0;
// // 	n_x_inv.row(0).col(1) = 1;
	 
// // }
// //****************************************************//

// 	// Matx33d M;

// 	// double x_mean, y_mean, dev1, dev2;
// 	// meanStdDev(corners[0], Scalar(x_mean),  Scalar(dev1));
// 	// meanStdDev(corners[1], Scalar(y_mean),  Scalar(dev2));
// 	// double s_x, s_y;
// 	// sqrt(Scalar(2./dev1), Scalar(s_x));
// 	// sqrt(Scalar(2./dev2), Scalar(s_y));

	
// 	// M(0, 0) = s_x;
// 	// M(0, 1) = 0;
// 	// M(0, 2) = -s_x*x_mean;
// 	// M(1, 0) = 0;
// 	// M(1, 1) = s_y;
// 	// M(1, 2) = -s_y*y_mean;
// 	// M(2, 0) = 0;
// 	// M(2, 1) = 0;
// 	// M(2, 2) = 1;

// 	// return M;
	


// //*****************************INTRINSIC***********************//



// // Mat computeViewBasedHomography(vector<vector<Point2f>> normalizeImagePoints, vector<vector<Point3f>> normalizeObjectPoints, int len)
// // {
// // 	Mat M(2*len, 9, CV_64FC2);

// // 	for (int i = 0; i < len; i++)
// // 	{
// // 		Point3f X, Y;
// // 		Point2f u, v;
// // 		X = normalizeObjectPoints[i][0];
// // 		Y = normalizeObjectPoints[i][1];
// // 		u = normalizeImagePoints[i][0];
// // 		v = normalizeImagePoints[i][1];
// // 		vector<float> row1, row2;
// // 		row1.push_back(-X);
// // 		row1.push_back(-Y);
// // 		row1.push_back(-1);
// // 		row1.push_back(0);
// // 		row1.push_back(0);
// // 		row1.push_back(0);
// // 		row1.push_back(X*u);
// // 		row1.push_back(Y*u);
// // 		row1.push_back(u);

// // 		row2.push_back(0);
// // 		row2.push_back(0);
// // 		row2.push_back(0);
// // 		row2.push_back(-X);
// // 		row2.push_back(-Y);
// // 		row2.push_back(-1);
// // 		row2.push_back(X*v);
// // 		row2.push_back(Y*v);
// // 		row2.push_back(v);

// // 		//M(2*i, 0) = row1;
// // 	}
// // }

// // Matx33d getIntrinsicParameters(vector<Matx33d> H_r)
// // {
// // 	int M = H_r.size();
// // 	Mat v = Mat(2*M, 6, CV_64F);

// // 	for (int i = 0; i < M; i++)
// // 	{
// // 		Matx33d H = H_r[i];
// // 		v(2*i, 0) = V_ij(0, 1, H);
// // 		subtract(V_ij(0, 0, H), V_ij(1, 1, H), v(2*i, 1));
// // 	}

// // 	Mat s, u, vt;
// // 	SVDecomp(v, s, u, vt);


// // }

// //*********************HOMOGRAPHY*************************//


// // vector<Matx33d> arrayHomographies(vector<vector<Point2f>>& imagePoints, vector<vector<Point3f>>& objectPoints)
// // {
// // 	vector<Matx33d> refinedHomographies;

// // 	cv::Matx33d H;

// // 	// 0. Prepare data;
// // 	cv::Matx33d srcS, tgtS, invTgtS;
// // 	cv::Matx33d A;

// // 	// 1. Perform normalization;
// // 	srcS = homography_dlt_sim_estimation<2>(imagePoints);
// // 	tgtS = homography_dlt_sim_estimation<3>(objectPoints);

// // 	auto src_n = imagePoints; // source normalized points
// // 	auto tgt_n = objectPoints; // target normalized points

// // 	invTgtS = tgtS.clone();
// // 	invert(invTgtS);

// // 	homography_dlt_normalize<2>(src_n, srcS);
// // 	homography_dlt_normalize<3>(tgt_n, tgtS);

// // 	// 2. Pack matrix A;
// // 	for (unsigned i = 0; i < imagePoints.size(); ++i) {
// // 		Point2f(A(i * 2 + 0, 0)) = -1 * src_n[i][0];
// // 		Point2f(A(i * 2 + 0, 1)) = -1 * src_n[i][1];
// // 		A(i * 2 + 0, 2) = -1;
// // 		Point3f(A(i * 2 + 0, 6)) = tgt_n[i][0] * src_n[i][0];
// // 		A(i * 2 + 0, 7) = tgt_n[i][0] * src_n[i][1];
// // 		Point3f(A(i * 2 + 0, 8)) = tgt_n[i][0];

// // 		A(i * 2 + 1, 3) = -1 * src_n[i][0];
// // 		A(i * 2 + 1, 4) = -1 * src_n[i][1];
// // 		A(i * 2 + 1, 5) = -1;
// // 		A(i * 2 + 1, 6) = tgt_n[i][1] * src_n[i][0];
// // 		A(i * 2 + 1, 7) = tgt_n[i][1] * src_n[i][1];
// // 		A(i * 2 + 1, 8) = tgt_n[i][1];
// // 	}

// // 	// 3. solve nullspace of A for H;
// // 	cv::null_solve(A, H);

// // 	H.reshape(3, 3);

// // 	// 4. denormalize the homography.
// // 	H = invTgtS * H * srcS;

// // 	return H;
	
// // }

// // //template<size_t _size>
// // void homographyDltNormalize2f(vector<vector<Point2f> > &features, const Matx33d &S) {
// // 	//ASSERT(S && S.rows() == 3 && S.cols() == 3);
// // 	Matx31f x(3, 1), xp(3, 1);
// // 	for (unsigned i = 0; i < features.size(); ++i) {
// // 		Point2f(x(0, 0)) = features[i][0];
// // 		Point2f(x(1, 0)) = features[i][1];
// // 		x(2, 0) = 1.;
// // 		//cross(S, x, xp);
// // 		features[i][0] = Point2f(xp(0, 0) / xp(2, 0));
// // 		features[i][1] = Point2f(xp(1, 0) / xp(2, 0));
// // 	}
// // }

// // void homographyDltNormalize3f(vector<vector<Point3f> > &features, const Matx33d &S) {
// // 	//ASSERT(S && S.rows() == 3 && S.cols() == 3);
// // 	Matx31f x(3, 1), xp(3, 1);
// // 	for (unsigned i = 0; i < features.size(); ++i) {
// // 		Point3f(x(0, 0)) = features[i][0];
// // 		Point3f(x(1, 0)) = features[i][1];
// // 		Point3f(x(2, 0)) = features[i][2];
// // 		//cross(S, x, xp);
// // 		features[i][0] = Point3f(xp(0, 0) / xp(2, 0));
// // 		features[i][1] = Point3f(xp(1, 0) / xp(2, 0));
// // 		features[i][2] = Point3f(1.);
// // 	}
// // }


// // Matx33d homography_dlt_sim_estimation(const vector<vector<Point2f> > &features) {
// // 	Mat transform = Mat::eye(3, 3, CV_64F);
	

// // 	size_t centroid(0, 0);
// // 	Mat S;

// // 	for (auto feat : features) {
// // 		centroid += feat;
// // 	}
// // 	centroid /= features.size();

// // 	Point2f sum_dist = 0;

// // 	for (auto feat : features) {
// // 		sum_dist+= centroid.distance(feat);
// // 	}
// // 	centroid *= -1;

// // 	Point2f scale_v = std::sqrt(2.) / (sum_dist / features.size());

// // 	transform(0, 0) = scale_v;
// // 	transform(1, 1) = scale_v;
// // 	transform(0, 2) = centroid[0];
// // 	transform(1, 2) = centroid[1];

// // 	return transform;
// // }

// //*****************************EXTRINSICS***********************//

