#include "../include/calibration.hpp"

Mat V = Mat::zeros(2*12, 6, CV_32FC1);

//#define SQUARE_SIZE = 1.0
// string readpath = "/home/tamarar/Desktop/novo/Camera_calibration/calibration/newCalibrationImages/Pic_";
// // vector<Point2f> imagePoints;
// vector<Point3f> objectPoints;
// vector<vector<double>> tmp;
// Mat nImg, nObj;
// Mat K;
// vector<Mat> extrinsic;


//Getting object point (world coord)
vector<Point3f> objectPoint(unsigned row, unsigned col, float squarSize)
{
	vector<Point3f> objectPoint;
	objectPoint.reserve(row*col);

	for (unsigned i = 0; i < row; i++)
	{
		for (unsigned j = 0; j < col; j++)
		{
			//objectPoint.push_back(Point3f((float)j*squarSize, (float)i*squarSize, 1));
			objectPoint.push_back({static_cast<float>(j*squarSize), static_cast<float>(i*squarSize), 1.});
		}
	}

	return objectPoint;
}

Mat normalizeImagePoints(vector<vector<Point2f>> &points, int w, int h)
{


	float sx = 2. / w;
	float sy = 2. / h;
	float x0 = w / 2.;
	float y0 = h / 2.;

	for (unsigned i = 0; i < points.size(); i++)
	{
		for (unsigned j = 0; j < points[i].size(); j++)
		{
			points[i][j].x = sx * (points[i][j].x - x0);
			points[i][j].y = sx * (points[i][j].y - x0);
		}
	}

	Mat N = Mat(3, 3, CV_32FC1);
	N.at<float>(0, 0) = sx;
	N.at<float>(0, 1) = 0.;
	N.at<float>(0, 2) = -sx * x0;
	N.at<float>(1, 0) = 0.;
	N.at<float>(1, 1) = sy;
	N.at<float>(1, 2) = -sy * y0;
	N.at<float>(2, 0) = 0.;
	N.at<float>(2, 1) = 0.;
	N.at<float>(2, 2) = 1.;
	return N;
}


Mat homographyDltSimEtimationImagePoints(vector<Point2f>& vector)
{
	//cout << "ESTIMATION IMAGE POINTS" << endl;
	Mat matrix = Mat::eye(3, 3, CV_32FC1);
	Point2f center(0, 0);
	Mat S;
	for (auto veec : vector)
	{
		center += veec;
	}

	center.x /= vector.size(); // x mean
	center.y /= vector.size(); // y mean

	float sum = 0;


	for (auto veec : vector)
	{
		double tmp = norm(veec - center);
		sum += tmp;
	}
	center *= -1;

	float scale_b = sqrt(2.) / (sum / vector.size());
	
	matrix.row(0).col(0) = scale_b;
	matrix.row(1).col(1) = scale_b;
	matrix.row(0).col(2) = center.x;
	matrix.row(1).col(2) = center.y;
	return matrix;

}


Mat homographyDltSimEtimationObjectPoints(vector<Point3f>& vector)
{
	//cout << "ESTIMATION OBJECT POINTS" << endl;
	Mat matrix = Mat::eye(3, 3, CV_32FC1);
	Point3f center(0, 0, 0);
	Mat S;
	for (auto veec : vector)
	{
		center += veec;
	}

	center.x /= vector.size(); // x maen
	center.y /= vector.size(); // y mean

	float sum = 0;

	for (auto veec :vector)
	{
		double tmp = norm(veec - center);
		sum += tmp;
	}
	center *= -1;

	float scale = sqrt(2.) / (sum / vector.size());
	
	matrix.row(0).col(0) = scale;
	matrix.row(1).col(1) = scale;
	matrix.row(0).col(2) = center.x;
	matrix.row(1).col(2) = center.y;
	return matrix;
	
}

void homographyDltNormalizeImagePoints(vector<Point2f>& point, Mat& S)
{
	//cout << "NORMALIZE IMAGE POINTS" << endl;
	Mat x = Mat::zeros(3, 1, CV_32FC1), xp = Mat::zeros(3, 1, CV_32FC1);
	//cout << "S = " << S << endl;
	for(unsigned i = 0; i < point.size(); i++)
	{
		x.at<float>(0, 0) = point[i].x;
		x.at<float>(1, 0) = point[i].y;
		x.at<float>(2, 0) = 1.;
		//cout << "S size = " << S.size() << endl;
		//xp = S.cross(x);
		xp = S * x;
		//cout << "xp = " << xp << endl;
		
		//cout << "S = " << S << endl;
		// float s = 0;
		// s = x.at<float>(0, 0) * S.at<float>(0, 0);
		// s += x.at<float>(1, 0) * S.at<float>(0, 1);
		// s += x.at<float>(2, 0) * S.at<float>(0, 2);
		// xp.at<float>(0, 0) = s;

		// s = 0;
		// s = x.at<float>(0, 0) * S.at<float>(1, 0);
		// s += x.at<float>(1, 0) * S.at<float>(1, 1);
		// s += x.at<float>(2, 0) * S.at<float>(1, 2);
		// xp.at<float>(1, 0) = s;

		// s = 0;
		// s = x.at<float>(0, 0) * S.at<float>(2, 0);
		// s += x.at<float>(1, 0) * S.at<float>(2, 1);
		// s += x.at<float>(2, 0) * S.at<float>(2, 2);
		// xp.at<float>(2, 0) = s;
		 //cout << xp << endl;

		//TREBA
		// xp.at<float>(0, 0) = x.dot(S.row(0));
		// xp.at<float>(1, 0) = x.dot(S.row(1));
		// xp.at<float>(2, 0) = x.dot(S.row(2));
		point[i].x = xp.at<float>(0, 0) / xp.at<float>(2, 0);
		//cout << "point[i].x = " << point[i].x << endl;
		point[i].y = xp.at<float>(1, 0) / xp.at<float>(2, 0);
		//cout << "point[i].y = " << point[i].y << endl;
		//point[i].z = 1;
	}
}

void homographyDltNormalizeObjectPoints(vector<Point3f>& point, Mat& S)
{
	//cout << "NORMALIZE OBJECT POINTS" << endl;
	Mat x = Mat::zeros(3, 1, CV_32FC1), xp = Mat::zeros(3, 1, CV_32FC1);
	for(unsigned i = 0; i < point.size(); i++)
	{
		x.at<float>(0, 0) = point[i].x;
		x.at<float>(1, 0) = point[i].y;
		x.at<float>(2, 0) = point[i].z;

		//cout << "S size = " << S.size() << endl;
		//multiply(S, x, xp);
		//xp = S.cross(x);
		//cout << "S = " << S << endl;
		xp = S * x;
		//cout << "xp = " << xp << endl;
		// float s = 0;
		// s = x.at<float>(0, 0) * S.at<float>(0, 0);
		// s += x.at<float>(1, 0) * S.at<float>(0, 1);
		// s += x.at<float>(2, 0) * S.at<float>(0, 2);
		// xp.at<float>(0, 0) = s;

		// s = 0;
		// s = x.at<float>(0, 0) * S.at<float>(1, 0);
		// s += x.at<float>(1, 0) * S.at<float>(1, 1);
		// s += x.at<float>(2, 0) * S.at<float>(1, 2);
		// xp.at<float>(0, 0) = s;

		// s = 0;
		// s = x.at<float>(0, 0) * S.at<float>(2, 0);
		// s += x.at<float>(1, 0) * S.at<float>(2, 1);
		// s += x.at<float>(2, 0) * S.at<float>(2, 2);
		// xp.at<float>(0, 0) = s;

		//TREBA
		// xp.at<float>(0, 0) = x.dot(S.row(0));
		// xp.at<float>(1, 0) = x.dot(S.row(1));
		// xp.at<float>(2, 0) = x.dot(S.row(2));
		point[i].x = xp.at<float>(0, 0) / xp.at<float>(2, 0);
		point[i].y = xp.at<float>(1, 0) / xp.at<float>(2, 0);
		point[i].z = 1;
	}
}
Mat homographyDlt(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints, Mat &H)
{
	//cout << "DLT" << endl;
	Mat img(3, 3, CV_32FC1), obj(3, 3, CV_32FC1), invTgtS(3, 3, CV_32FC1);
	Mat A = Mat::zeros(2*imagePoints.size(), 9, CV_32FC1);
	//Mat H(3, 3, CV_32FC1);
	//img = homographyDltSimEtimationImagePoints(imagePoints);
	img = imagePointNomalizationMatrix(imagePoints);
	//cout << "img = " << img << endl;
	//cout << "********************" << endl;
	//obj = homographyDltSimEtimationObjectPoints(objectPoints);
	obj = objectPointNomalizationMatrix(objectPoints);
	//cout << img << endl;
	//cout << "********************" << endl;



	//auto src_n = imagePoints;
	//cout << "scr_n = " << src_n << endl;
	//auto tgt_n = objectPoints; 

	//invTgtS = obj.clone();
	//cout << "invTgtS = " << invTgtS << endl;
	invert(obj, invTgtS);
	//cout << invTgtS << endl;
	//cout << "Invert invTgts = " << invTgtS << endl;

	homographyDltNormalizeImagePoints(imagePoints, img);
	
	// for (unsigned i = 0; i < imagePoints.size(); i++)
	// {
	// 	//cout << imagePoints[i] << endl;
	// }

	homographyDltNormalizeObjectPoints(objectPoints, obj);

	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		//cout << objectPoints[i] << endl;
	}

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		A.at<float>(i*2 + 0, 0) = -1*imagePoints[i].x;
		A.at<float>(i*2 + 0, 1) = -1*imagePoints[i].y;
		A.at<float>(i*2 + 0, 2) = -1.;
		A.at<float>(i*2 + 0, 3) = 0.;
		A.at<float>(i*2 + 0, 4) = 0.;
		A.at<float>(i*2 + 0, 5) = 0.;
		A.at<float>(i*2 + 0, 6) = imagePoints[i].x * objectPoints[i].x;
		A.at<float>(i*2 + 0, 7) = imagePoints[i].y * objectPoints[i].y;
		A.at<float>(i*2 + 0, 8) = objectPoints[i].x;

		A.at<float>(i*2 + 1, 0) = 0.;
		A.at<float>(i*2 + 1, 1) = 0.;
		A.at<float>(i*2 + 1, 2) = 0.;
		A.at<float>(i*2 + 1, 3) = -1*imagePoints[i].x;
		A.at<float>(i*2 + 1, 4) = -1*imagePoints[i].y;
		A.at<float>(i*2 + 1, 5) = -1.;
		A.at<float>(i*2 + 1, 6) = imagePoints[i].x * objectPoints[i].y;
		A.at<float>(i*2 + 1, 7) = imagePoints[i].y * objectPoints[i].y;
		A.at<float>(i*2 + 1, 8) = objectPoints[i].y;
		
	}

	//cout << "A = " << A << endl;

	Mat S, U, VT;
	//Mat B = Mat::zeros(1, 1, CV_64FC1);
	//solve(A, H, B);
	//vector<float> U;

	SVDecomp(A, U, S, VT);
	//cout << "Vt = " << VT <<  endl;
	//cout << "####################3" << endl;
	
	//cout << "U size = " << U.size() << endl;
	//cout << "S size = " << S.size() << endl;
	//cout << "S height = " << S.size().height << endl;
	//cout << "S width = " << S.size().width << endl;
	//cout << "S = " << S << endl;
	//cout << "VT size = " << VT.size() << endl;
	//int minArg = minarg(S);
	//cout << "minarg = " << minArg << endl;
	//cout << "vt[8] = " << VT.row(8) << endl;
	//cout << "Vt transpose = " << VT <<  endl;
	//cout << "####################3" << endl;
	Mat Vtransp;
	//cout << Vtransp << endl;
	//transpose(VT, Vtransp);
	// cout << "Vt transpose = " << Vtransp <<  endl;
	// cout << "####################3" << endl;
	
	
	auto HTmp = VT.col(VT.cols - 1);
	Mat h(3, 3, CV_32FC1);
	//cout << "HTmp = " << HTmp << endl;
	
	// h.at<float>(0, 0) = HTmp[0];
	// h.at<float>(0, 1) = HTmp[1];
	// h.at<float>(0, 2) = HTmp[2];
	// h.at<float>(1, 0) = HTmp[3];
	// h.at<float>(1, 1) = HTmp[4];
	// h.at<float>(1, 2) = HTmp[5];
	// h.at<float>(2, 0) = HTmp[6];
	// h.at<float>(2, 1) = HTmp[7];
	// h.at<float>(2, 2) = HTmp[8];

	//H = Mat(VT.row(8));
	//H = H.reshape(3, 3);
	//cout << "H = " << H << endl;
	

	
	
	// cout << "VT = " << VT << endl;
	// cout << "**************" << endl;
	// cout << "VT[8] = " << VT.row(8) << endl;
	//solve(A, H, 0, DECOMP_SVD);

	
	//Mat h_norm = VT.col(-1)
	//h_norm.reshape(3, 3);
	//Mat src3 = Mat::zeros(1, 1, CV_64FC1);
	// Mat h(3, 3, CV_64FC1);
	h.at<float>(0, 0) = VT.at<float>(0, 8);
	h.at<float>(0, 1) = VT.at<float>(1, 8);
	h.at<float>(0, 2) = VT.at<float>(2, 8);
	h.at<float>(1, 0) = VT.at<float>(3, 8);
	h.at<float>(1, 1) = VT.at<float>(4, 8);
	h.at<float>(1, 2) = VT.at<float>(5, 8);
	h.at<float>(2, 0) = VT.at<float>(6, 8);
	h.at<float>(2, 1) = VT.at<float>(7, 8);
	h.at<float>(2, 2) = VT.at<float>(8, 8);
	//cout << h << endl;
	//cout << VT.row(8) << endl;

	//cout << "invTgts size = " << invTgtS.size() << endl;
	//cout << "img size = " << img.size() << endl;
	//cout << "H size = " << H.size() << endl;
	//h.dot(invTgtS);
	h = invTgtS * h * img;
	//cout << "h size novo = " << h.size() << endl;
	//h.dot(img);
	//gemm(invTgtS, H, 1.0, noArray(), 0.0, H); //* img;
	//gemm(H, img, 1.0, noArray(), 0.0, H);

}

//##############CAMERA MATRIX##################//
//int i = 0;

Mat V_ij(Mat &H, int i, int j)
{
	//cout << ++i << " " << H << endl;
	Mat v(1, 6, CV_32FC1);	
	// v.push_back(H.at<float>(0, i) * H.at<float>(0, j)); 
	// v.push_back(H.at<float>(0, i) * H.at<float>(1, j) + H.at<float>(1, i) * H.at<float>(0, j));
	// v.push_back(H.at<float>(1, i) * H.at<float>(1, j));
	// v.push_back(H.at<float>(2, i) * H.at<float>(0, j) + H.at<float>(0, i) * H.at<float>(2, j));
	// v.push_back(H.at<float>(2, i) * H.at<float>(1, j) + H.at<float>(1, i) * H.at<float>(2, j));
	// v.push_back(H.at<float>(2, i) * H.at<float>(2, j));

	v.at<float>(0, 0) = H.at<float>(0, i) * H.at<float>(0, j); 
	v.at<float>(0, 1) = H.at<float>(0, i) * H.at<float>(1, j) + H.at<float>(1, i) * H.at<float>(0, j);
	v.at<float>(0, 2) = H.at<float>(1, i) * H.at<float>(1, j);
	v.at<float>(0, 3) = H.at<float>(2, i) * H.at<float>(0, j) + H.at<float>(0, i) * H.at<float>(2, j);
	v.at<float>(0, 4) = H.at<float>(2, i) * H.at<float>(1, j) + H.at<float>(1, i) * H.at<float>(2, j);
	v.at<float>(0, 5) = H.at<float>(2, i) * H.at<float>(2, j);
	return v;
}

void getV(vector<Mat> &H, Mat &V)
{
	//Mat V(2*H.size(), 6, CV_32FC1);

	for (unsigned i = 0; i < H.size(); i++)
	{
		// auto h1 = V.row(i*2);
		// auto h2 = V.row(i*2 + 1);

		auto v12 = V_ij(H[i], 0, 1);
		auto v11 = V_ij(H[i], 0, 0);
		auto v22 = V_ij(H[i], 1, 1);
		//auto v11_v22 = v11 - v22;
		Mat v11_v22; 
		subtract(v11, v22, v11_v22);

		//V.row(i*2).push_back(v12);
		//V.row(i*2 + 1).push_back(v11_v22);

		V.at<float>(i*2, 0) = v12.at<float>(0, 0);
		V.at<float>(i*2, 1) = v12.at<float>(0, 1);
		V.at<float>(i*2, 2) = v12.at<float>(0, 2);
		V.at<float>(i*2, 3) = v12.at<float>(0, 3);
		V.at<float>(i*2, 4) = v12.at<float>(0, 4);
		V.at<float>(i*2, 5) = v12.at<float>(0, 5);
	

		V.at<float>(i*2 + 1, 0) = v11_v22.at<float>(0, 0);
		V.at<float>(i*2 + 1, 1) = v11_v22.at<float>(0, 1);
		V.at<float>(i*2 + 1, 2) = v11_v22.at<float>(0, 2);
		V.at<float>(i*2 + 1, 3) = v11_v22.at<float>(0, 3);
		V.at<float>(i*2 + 1, 4) = v11_v22.at<float>(0, 4);
		V.at<float>(i*2 + 1, 5) = v11_v22.at<float>(0, 5);
	}

	cout << "V from func = " << V << endl;
	
}

bool intrinsics(Mat &B, float &u0, float &v0, float &lam, float &alpha, float &beta, float &gama)
{
	auto d = B.at<float>(0, 0) * B.at<float>(2, 2) - B.at<float>(0, 1) * B.at<float>(0, 1);

	if (fabs(d) < 1e-8)
	{
		cout << "d < 1e-8" << endl;
		return false;
	}

	v0 = (B.at<float>(0, 1)*B.at<float>(0, 2) - B.at<float>(0, 0)*B.at<float>(1, 2)) / (B.at<float>(0, 0)*B.at<float>(1, 1) - B.at<float>(0, 1)*B.at<float>(0, 1));
	lam = B.at<float>(2, 2) - (B.at<float>(0, 1)*B.at<float>(0, 1) + v0*(B.at<float>(0, 1)*B.at<float>(0, 2) - B.at<float>(0, 0)*B.at<float>(1, 2))) / B.at<float>(0, 0);
	auto l = (lam / B.at<float>(0, 0));
	if (l < .0)
	{
		cout << "l < 0: " << l << endl;
		return false;
	}
	alpha = sqrt(l);
	auto b = (lam*B.at<float>(0, 0))/(B.at<float>(0, 0)*B.at<float>(1, 1) - B.at<float>(0, 1)*B.at<float>(0, 1));
	if (b < .0)
	{
		cout << "b < 0: " << b << endl;
		return false;
	}
	beta = sqrt(b);
	gama = (-1*B.at<float>(0, 1)*(alpha*alpha)*beta)/lam;
	u0 = (gama*v0)/alpha - (B.at<float>(0, 2)*(alpha*alpha))/lam;
	return true;
}

void getIntrinsicParameters(vector<Mat>& H_r, Mat &Kp)
{
	getV(H_r, V);
	//cout << "V = " << endl;
	Mat U, S, Vt;
	SVDecomp(V, U, S, Vt);

	transpose(Vt, Vt);
	auto b = Vt.col(Vt.cols - 1);
	//cout << "b = " << b << endl;

	Mat B = Mat::zeros(3, 3, CV_32FC1);
	cout << "b = " << b << endl << endl;
	// B.at<float>(0, 0) = b[0];
	// B.at<float>(0, 1) = b[1];
	// B.at<float>(0, 2) = b[3];
	// B.at<float>(1, 0) = b[1];
	// B.at<float>(1, 1) = b[2];
	// B.at<float>(1, 2) = b[4];
	// B.at<float>(2, 0) = b[3];
	// B.at<float>(2, 1) = b[4];
	// B.at<float>(2, 2) = b[5];


	float u0, v0, alpha, beta, gama, lam;

	if (intrinsics(B, u0, v0, lam, alpha, beta, gama))
	{
		Mat K(3, 3, CV_32FC1);
		K.at<float>(0, 0) = alpha;
		K.at<float>(0, 1) = gama;
		K.at<float>(0, 2) = u0;
		K.at<float>(1, 0) = 0.;
		K.at<float>(1, 1) = beta;
		K.at<float>(1, 2) = v0;
		K.at<float>(2, 0) = 0.;
		K.at<float>(2, 1) = 0.;
		K.at<float>(2, 2) = 1.;
	}
	else
	{
		cout << "Error calculating K..." << endl;
	}
}

Mat intrinsicsDenormalize(Mat &K, Mat &N)
{
	auto N_inv = N.clone();
	invert(N_inv, N_inv);
	auto M = N_inv*K;
	return M;
}


Mat getExtrinsicsParameters(Mat &K, Mat &H)
{
	auto KInv = K.clone();
	invert(KInv, KInv);

	Mat rt(3, 4, CV_32FC1);

	auto h1 = H.col(0);
	auto h2 = H.col(1);
	auto h3 = H.col(2);

	auto r1 = KInv * h1;
	auto r2 = KInv * h2;

	float lam1 = 1. / norm(r1);
	float lam2 = 1. / norm(r2);
	float lam3 = (lam1 + lam2) / 2.;

	r1 *= lam1;
	r2 *= lam2;

	auto r3 = r1.cross(r2);

	auto t = (KInv * h3) * lam3;

	Mat R(3, 3, CV_32FC1);
	Mat U, S, Vt;
	SVDecomp(R, U, S, Vt);
	R = U * Vt;

	auto tmp1 = R.col(0);
	auto tmp2 = R.col(1);
	auto tmp3 = R.col(2);


	// rt.at<float>(0, 0) = r1[0];
	// rt.at<float>(0, 1) = r2[0];
	// rt.at<float>(0, 2) = r3[0];
	// rt.at<float>(0, 3) = t[0];
	// rt.at<float>(1, 0) = r1[1];
	// rt.at<float>(1, 1) = r2[1];
	// rt.at<float>(1, 2) = r3[1];
	// rt.at<float>(1, 3) = t[1];
	// rt.at<float>(2, 0) = r1[2];
	// rt.at<float>(2, 1) = r2[2];
	// rt.at<float>(2, 2) = r3[2];
	// rt.at<float>(2, 2) = t[2];

	return rt;
}


/*
vector<float> distortion(vector<vector<Vec2f>> &imagePoints, vector<vector<Vec2f>> &imagePointsNorm, vector<vector<Vec2f>> &imageProj, Mat &K)
{
	vector<float> k = {0., 0., 0., 0., 0., 0., 0., 0.};
	float u0, v0, ui_uo, vi_vo, xy;

	float num = imagePoints.front().size();

	Mat D(imagePoints.size()*num*2, 2, CV_64FC1);
	Mat d(imagePoints.size()*num*2, 1, CV_64FC1);

	u0 = K.at<float>(0, 2);
	v0 = K.at<float>(1, 2);

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		for (unsigned j = 0; j < num; j++)
		{
			xy = sum((imagePointsNorm[i][j] * imagePointsNorm[i][j]));
			ui_uo = imageProj[i][j][0] - u0;
			vi_vo = imageProj[i][j][1] - v0;

			D.at<float>(i*2, 0) = ui_uo*xy;
			D.at<float>(i*2, 1) = ui_uo*xy*xy;

			D.at<float>(i*2+1, 0) = vi_vo*xy;
			D.at<float>(i*2+1, 1) = vi_vo*xy*xy;

			d.at<float>(i*2, 0) = imagePoints[i][j][0] - imageProj[i][j][0];
			d.at<float>(i*2+1, 0) = imagePoints[i][j][1] - imageProj[i][j][1];
		}
	}

	//Mat K = Mat(imagePoints.size()*4, imagePoints.size(), CV_64FC1);
	//solve(D, K, d, DECOMP_SVD);

	// k[0] = K.data()[0];
	// k[1] = K.data()[1];

	return k;
}*/




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

// // Detecting chessboard corners
// vector<Point2f> findchessboardCorners()
// {
// 	vector<Point2f> corners;
// 	vector<Point3f> objPoint = objectPoint(9, 6, 1.);
// 	bool patternfound;
	
// 	for (int i = 1; i < 14; i++)
// 	{
// 		Mat image = imread(readpath + to_string(i) + ".jpg");
// 		patternfound = findChessboardCorners(image, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
// 		if (patternfound)
// 		{
// 			cout << "Corners found!" << endl;
// 			imagePoints.insert(imagePoints.end(), corners.begin(), corners.end());
// 			objectPoints.insert(objectPoints.end(), objPoint.begin(), objPoint.end());
// 			//imagePoints.push_back(corners);
// 			//objectPoints.push_back(objPoint);
// 		}
// 	}

// 	return corners;
// }

// //**********************HOMOGRAPHY******************************//

// //####################################################################//
// Point2f meanImagePoints(vector<Point2f>& vector)
// {
// 	float meanX, meanY = 0;
// 	Point2f mean;
// 	for (unsigned i = 0; i < vector.size(); i++)
// 	{
// 		meanX += vector[i].x;
// 		meanY += vector[i].y;
// 	}

// 	meanX /= vector.size();
// 	meanY /= vector.size();
// 	mean.x = meanX;
// 	mean.y = meanY;
// 	return mean;
// }

// Point2f varianceImagePoints(vector<Point2f>& vector, float xMean, float yMean)
// {
// 	Point2f var;
// 	for (unsigned i = 0; i < vector.size(); i++)
// 	{
// 		var.x += (vector[i].x - xMean) * (vector[i].x - xMean);
// 		var.y += (vector[i].y - yMean) * (vector[i].y - yMean);
// 	}

// 	var.x /= vector.size();
// 	var.x /= vector.size();
// 	return var;
// }

// Point3f varianceObjectPoints(vector<Point3f>& vector, float xMean, float yMean)
// {
// 	Point3f var;
// 	for (unsigned i = 0; i < vector.size(); i++)
// 	{
// 		var.x += (vector[i].x - xMean) * (vector[i].x - xMean);
// 		var.y += (vector[i].y - yMean) * (vector[i].y - yMean);
// 	}

// 	var.x /= vector.size();
// 	var.x /= vector.size();
// 	return var;
// }

// Point3f meanObjectPoints(vector<Point3f>& vector)
// {
// 	float meanX, meanY = 0;
// 	Point3f mean;
// 	for (unsigned i = 0; i < vector.size(); i++)
// 	{
// 		meanX += vector[i].x;
// 		meanY += vector[i].y;
// 	}

// 	meanX /= vector.size();
// 	meanY /= vector.size();
// 	mean.x = meanX;
// 	mean.y = meanY;
// 	return mean;
// }

// //####################################################################//


// int minarg(Mat& S)
// {
// 	int minarg = 0;
// 	float min = S.at<float>(0, 0);
// 	for (int i = 0; i < S.size().height; i++)
// 	{
// 		for (int j = 0; j < S.size().width; j++)
// 		{
// 			if (min > S.at<float>(i, j))
// 			{
// 				min = S.at<float>(i, j);
// 				minarg = i + j;
// 			}
// 		}
// 	}
// 	return minarg;
// }


// Mat homographyLeastSquares()
// {
// 	Mat A, B, H;
// 	A = Mat::zeros(imagePoints.size() * 2, 8, CV_64FC1);
// 	B.create(imagePoints.size() * 2, 1, CV_64FC1);
// 	// for (int i = 0; i < B.size().height; i++)
// 	// {
// 	// 	for (int j = 0; j < B.size().width; j++)
// 	// 	{
// 	// 		B.at<float>(i, j) = 0;
// 	// 	}
// 	// }

// 	// populate matrices with data.
// 	for (unsigned i = 0; i < imagePoints.size(); i++) {

// 		auto &src = imagePoints[i];
// 		auto &tgt = imagePoints[i];

// 		B.at<Point3f>(i * 2, 0) = objectPoints[0];
// 		B.at<Point3f>(i * 2 + 1, 0) = objectPoints[1];

// 		A.at<float>(i * 2, 0) = src.x;
// 		A.at<float>(i * 2, 1) = src.y;
// 		A.at<float>(i * 2, 2) = 1;
// 		A.at<float>(i * 2 + 1, 3) = src.x;
// 		A.at<float>(i * 2 + 1, 4) = src.y;
// 		A.at<float>(i * 2 + 1, 5) = 1;

// 		A.at<float>(i * 2, 6) = -1 * src.x * tgt.x;
// 		A.at<float>(i * 2, 7) = -1 * src.y * tgt.x;
// 		A.at<float>(i * 2 + 1, 6) = -1 * src.x * tgt.y;
// 		A.at<float>(i * 2 + 1, 7) = -1 * src.y * tgt.y;
// 	}
// 	Mat _H(8, 1, CV_64FC1);
// 	Mat At;
// 	transpose(A, At);
// 	solve(At*A, At*B, _H);
// 	cout << _H << endl;

// 	H.create(3,3, CV_64FC1);
// 	H.at<float>(0, 0) = _H.at<float>(0, 0);
// 	H.at<float>(0, 1) = _H.at<float>(1, 0);
// 	H.at<float>(0, 2) = _H.at<float>(2, 0);
// 	H.at<float>(1, 0) = _H.at<float>(3, 0);
// 	H.at<float>(1, 1) = _H.at<float>(4, 0);
// 	H.at<float>(1, 2) = _H.at<float>(5, 0);
// 	H.at<float>(2, 0) = _H.at<float>(6, 0);
// 	H.at<float>(2, 1) = _H.at<float>(7, 0);
// 	H.at<float>(2, 2) = _H.at<float>(8, 0);
// 	return H;

	
// }





// // int argmin(vector<float>& vector)
// // {
// // 	std::vector<float>::iterator iterator = std::min_element(vector.begin(), vector.end());
// // 	int min = iterator[0];
// // 	for (unsigned i = 0; i < vector.size(); i++)
// // 	{
// // 		if(vector[i] == min)
// // 			return i;
// // 	}
// // 	return -1;
// // }

// //##################################################################//


// Mat normalizePointsMatrix()
// {
// 	vector<float> xCoord, yCoord;
// 	for (unsigned i = 0; i < imagePoints.size(); i++)
// 	{
// 		xCoord.push_back(imagePoints[i].x);
// 		yCoord.push_back(imagePoints[i].y);
// 	}

// 	float xMean, yMean, xDev, yDev, xVar, yVar;
// 	meanStdDev(xCoord, Scalar(xMean), Scalar(xDev));
// 	meanStdDev(yCoord, Scalar(yMean), Scalar(yDev));
// 	xVar = xDev*xDev;
// 	yVar = yDev*yDev;

// 	float s_x, s_y;
// 	s_x = sqrt(2/xVar);
// 	s_y = sqrt(2/yVar);

// 	Mat n;
// 	n.at<float>(0, 0) = s_x;
// 	n.at<float>(0, 1) = 0;
// 	n.at<float>(0, 2) = -s_x*xMean;
// 	n.at<float>(1, 0) = 0;
// 	n.at<float>(1, 1) = s_y;
// 	n.at<float>(1, 2) = -s_y*yMean;
// 	n.at<float>(2, 0) = 0;
// 	n.at<float>(2, 1) = 0;
// 	n.at<float>(2, 2) = 1;

// 	return n;

// }



// //##################################################################//

// // //************************CAMERA MATRIX****************************//


// Mat extrinsicsCalculation(Mat& intrinsic, Mat& H_r)
// {
// 	auto intrinsicInv = intrinsic.clone();
// 	invert(intrinsicInv, intrinsicInv);
// 	Mat rt(3, 4, CV_64FC1);

// 	auto h1 = H_r.col(0);
// 	auto h2 = H_r.col(1);
// 	auto h3 = H_r.col(2);

// 	auto r1 = intrinsicInv * h1;
// 	auto r2 = intrinsicInv * h2;

// 	float lam1 = 1. / cv::norm(r1);
// 	float lam2 = 1. / cv::norm(r2);
// 	float lam3 = (lam1 + lam2) / 2.;

// 	r1 *= lam1;
// 	r2 *= lam2;
// 	auto r3 = r1.cross(r2);

// 	auto t = (intrinsicInv * h3) * lam3;

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

	

// 	// Mat homography = H_r.reshape(3, 3);
// 	// auto intrinsicInv = intrinsic.clone(); 
// 	// invert(intrinsicInv, intrinsicInv);

// 	// vector<float> h1, h2, h3;
// 	// h1 = homography.col(0);
// 	// h2 = homography.col(1);
// 	// h3 = homography.col(2);

// 	// float lam_r1, lam_r2, lam_r3;
// 	// vector<float> r1, r2, r3, t;
// 	// normalize(intrinsicInv.Mat::dot(h1), intrinsicInv);
// 	// lam_r1 = 1. / intrinsicInv;
// 	// normalize(intrinsicInv.Mat::dot(h2), intrinsicInv);
// 	// lam_r2 = 1. / intrinsicInv;
// 	// lam_r3 = (lam_r1 + lam_r2) / 2;
// 	// r1 = lam_r1 * intrinsicInv.Mat::dot(h1);
// 	// r2 = lam_r2 * intrinsicInv.Mat::dot(h2);
// 	// r3 = r1.cross(r2);
// 	// t = transpose(lam_r3 * intrinsicInv.Mat::dot(h3));
	
// 	// Mat rt;

// 	// rt.row(0).col(0) = r1[0];
// 	// rt.row(0).col(1) = r2[0];
// 	// rt.row(0).col(2) = r3[0];
// 	// rt.row(0).col(3) = t[0];
// 	// rt.row(1).col(0) = r1[1];
// 	// rt.row(1).col(1) = r2[1];
// 	// rt.row(1).col(2) = r3[1];
// 	// rt.row(1).col(3) = t[1];
// 	// rt.row(2).col(0) = r1[2];
// 	// rt.row(2).col(1) = r2[2];
// 	// rt.row(2).col(2) = r3[2];
// 	// rt.row(2).col(3) = t[2];

// 	// return rt;
	
// }


// Calculating dot product from matrices A and B
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

// 	for (unsigned i = 0; i < homographies.size(); i++)
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


//////////////////////// Normalization again ///////////////////////////////////
Mat imagePointNomalizationMatrix(vector<Point2f> &imagePoints)
{
	// float xMean = 0, yMean = 0, xDev = 0, yDev = 0, xVar = 0, yVar = 0;
	// //meanStdDev(imagePoints[0], Scalar(xMean), Scalar(xDev));
	// //meanStdDev(imagePoints[1], Scalar(yMean), Scalar(yDev));

	// Vec2f mean = meanImagePoint(imagePoints);
	// xMean = mean[0];
	// yMean = mean[1];

	// Vec2f var = varianceImagePoints(imagePoints, xMean, yMean);
	// xVar = var[0];
	// yVar = var[1];

	// //cout << "xMean = " << xMean << endl << "yMean = " << yMean << endl << "xVar = " << xVar << endl << "yVar = " << yVar << endl;

	// float s_x, s_y;
	// //float tmpx = sqrt(xVar);
	// //float tmpy = sqrt(yVar);
	// s_x = sqrt(2. / xVar*xVar);
	// s_y = sqrt(2. / yVar*xVar);

	// Mat n(3, 3, CV_64FC1);
	// n.at<float>(0, 0) = s_x;
	// n.at<float>(0, 1) = 0.;
	// n.at<float>(0, 2) = -s_x*xMean;
	// n.at<float>(1, 0) = 0.;
	// n.at<float>(1, 1) = s_y;
	// n.at<float>(1, 2) = -s_y*yMean;
	// n.at<float>(2, 0) = 0.;
	// n.at<float>(2, 1) = 0.;
	// n.at<float>(2, 2) = 1;

	// return n;

	//**************************************]

	// vector<float> imagePointX;
	// vector<float> imagePointY;

	// for (auto img : imagePoints)
	// {
	// 	imagePointX.push_back(img.x);
	// 	imagePointY.push_back(img.y);
	// }

	// float xMean, xDev, yMean, yDev;
	// Mat tmp = Mat::eye(3, 3, CV_64FC1);
	// meanStdDev(imagePointX, Scalar(xMean), Scalar(xDev));
	// meanStdDev(imagePointY, Scalar(yMean), Scalar(yDev));
	
	// float s_xTmp, s_yTmp;

	// s_xTmp = sqrt(2 / (xDev*xDev));
	// s_yTmp = sqrt(2. / (yDev*yDev));

	// tmp.row(0).col(0) = s_xTmp;
	// tmp.row(1).col(1) = s_yTmp;
	// tmp.row(0).col(2) = -s_xTmp*xMean;
	// tmp.row(1).col(2) = -s_yTmp*yMean;

	//**************************************

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
		//double tmp = norm(veec - center);
		//sum += tmp;
		sum.x += (veec.x - center.x) * (veec.x - center.x);
		sum.y += (veec.y - center.y) * (veec.y - center.y);
	}
	//center *= -1;

	//float scale_b = sqrt(2.) / (sum / vector.size());

	Point2f var(0, 0);
	var.x = sum.x / imagePoints.size(); 
	var.y = sum.y / imagePoints.size(); 

	Point2f sd(0, 0);
	sd.x = sqrt(var.x);
	sd.y = sqrt(var.y);

	float s_x = sqrt(2. / var.x);
	float s_y = sqrt(2. / var.y);

	
	matrix.row(0).col(0) = s_x;
	matrix.row(1).col(1) = s_y;
	matrix.row(0).col(2) = -s_x*center.x;
	matrix.row(1).col(2) = -s_y*center.y;
	return matrix;

}

Mat objectPointNomalizationMatrix(vector<Point3f> &objectPoints)
{
	// float xMean = 0, yMean = 0, xDev = 0, yDev = 0, xVar = 0, yVar = 0;
	// //meanStdDev(objectPoints[0], Scalar(xMean), Scalar(xDev));
	// //meanStdDev(objectPoints[1], Scalar(yMean), Scalar(yDev));

	// Vec2f mean = meanObjectPoint(objectPoints);
	// xMean = mean[0];
	// //cout << xMean << endl;
	// yMean = mean[1];

	// Vec2f var = varianceObjectPoints(objectPoints, xMean, yMean);
	// xVar = var[0];
	// yVar = var[1];

	

	// float s_x, s_y;
	// //float tmpx = sqrt(xVar);
	// //float tmpy = sqrt(yVar);
	// s_x = sqrt(2. / xVar*xVar);
	// s_y = sqrt(2. / yVar*yVar);

	// Mat n(3, 3, CV_64FC1);
	// n.at<float>(0, 0) = s_x;
	// n.at<float>(0, 1) = 0.;
	// n.at<float>(0, 2) = -s_x*xMean;
	// n.at<float>(1, 0) = 0.;
	// n.at<float>(1, 1) = s_y;
	// n.at<float>(1, 2) = -s_y*yMean;
	// n.at<float>(2, 0) = 0.;
	// n.at<float>(2, 1) = 0.;
	// n.at<float>(2, 2) = 1;

	// return n;

	//***************************************

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
	//center *= -1;

	Point2f var(0, 0);
	var.x = sum.x / objectPoints.size(); 
	var.y = sum.y / objectPoints.size(); 

	float s_x = sqrt(2. / var.x);
	float s_y = sqrt(2. / var.y);


	//float scale = sqrt(2.) / (sum / objectPoints.size());
	
	
	matrix.row(0).col(0) = s_x;
	matrix.row(1).col(1) = s_y;
	matrix.row(0).col(2) = -s_x*center.x;
	matrix.row(1).col(2) = -s_y*center.y;
	return matrix;
}



Point2f meanImagePoint(vector<Point2f> &imagePoints)
{
	float xMean = 0, yMean = 0;
	Point2f mean;

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		xMean += imagePoints[i].x;
		yMean += imagePoints[i].y;
	}

	xMean /= imagePoints.size();
	yMean /= imagePoints.size();
	mean.x = xMean;
	mean.y = yMean;
	return mean;

}

Point2f meanObjectPoint(vector<Point3f> &objectPoints)
{
	float xMean = 0, yMean = 0;
	Point2f mean;

	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		xMean += objectPoints[i].x;
		yMean += objectPoints[i].y;
	}

	xMean /= objectPoints.size();
	yMean /= objectPoints.size();
	mean.x = xMean;
	mean.y = yMean;
	return mean;
}

Point2f varianceImagePoints(vector<Point2f> &imagePoints, float xMean, float yMean)
{
	Point2f var(0, 0);
	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		var.x += (imagePoints[i].x - xMean) * (imagePoints[i].x - xMean);
		var.y += (imagePoints[i].y - yMean) * (imagePoints[i].y - yMean);
	}

	var.x /= imagePoints.size();
	var.y /= imagePoints.size();
	return var;
}

Point2f varianceObjectPoints(vector<Point3f> &objectPoints, float xMean, float yMean)
{
	Point2f var(0, 0);
	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		var.x += (objectPoints[i].x - xMean) * (objectPoints[i].x - xMean);
		var.y += (objectPoints[i].y - yMean) * (objectPoints[i].y - yMean);
	}

	var.x /= objectPoints.size();
	var.y /= objectPoints.size();
	return var;
}

// Mat homographyEstimation(vector<Vec2f> &imagePoints, vector<Vec3f> &objectPoints)
// {
// 	Mat imageNormalization(3, 3, CV_64FC1), objectNomalization(3, 3, CV_64FC1);
// 	imageNormalization = homographyDltSimEtimationImagePoints(imagePoints);
// 	objectNomalization = homographyDltSimEtimationObjectPoints(objectPoints);

// 	Mat A = Mat::zeros(2*imagePoints.size(), 9, CV_64FC1);

// 	for (unsigned i = 0; i < imagePoints.size(); i++)
// 	{
// 		vector<float> tmp1;
// 		tmp1.push_back(imagePoints[i[0]);
// 		tmp1.push_back(imagePoints[i][1]);
// 		tmp1.push_back(1.);

// 		vector<float> tmp2;
// 		tmp1.push_back(objectPoints[i][0]);
// 		tmp1.push_back(objectPoints[i][1]);
// 		tmp1.push_back(1.);

// 		vector<float> tmp3;

// 		imageNormalization.dot(tmp1);
// 	}

// }

	

	 



