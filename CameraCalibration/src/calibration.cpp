#include "../include/calibration.hpp"


//Getting object point (world coord)
vector<Point3d> objectPoint(unsigned row, unsigned col, double squarSize)
{
	vector<Point3d> objectPoint;
	objectPoint.reserve(row*col);

	for (unsigned i = 0; i < col; i++)
	{
		for (unsigned j = 0; j < row; j++)
		{
			objectPoint.push_back({static_cast<double>(j*squarSize), static_cast<double>(i*squarSize), 1.});
		}
	}

	return objectPoint;
}

Mat normalizeImagePoints(vector<vector<Point2d>> &points, int w, int h)
{
	double sx = 2. / w;
	double sy = 2. / h;
	double x0 = w / 2.;
	double y0 = h / 2.;

	for (unsigned i = 0; i < points.size(); i++)
	{
		for (unsigned j = 0; j < points[i].size(); j++)
		{
			points[i][j].x = sx * (points[i][j].x - x0);
			points[i][j].y = sx * (points[i][j].y - x0);
		}
	}

	Mat N = Mat(3, 3, CV_64FC1);
	N.at<double>(0, 0) = sx;
	N.at<double>(0, 1) = 0.;
	N.at<double>(0, 2) = -sx * x0;
	N.at<double>(1, 0) = 0.;
	N.at<double>(1, 1) = sy;
	N.at<double>(1, 2) = -sy * y0;
	N.at<double>(2, 0) = 0.;
	N.at<double>(2, 1) = 0.;
	N.at<double>(2, 2) = 1.;
	return N;
}


Mat homographyDltSimEtimationImagePoints(vector<Point2d>& vector)
{
	//cout << "ESTIMATION IMAGE POINTS" << endl;
	Mat matrix = Mat::eye(3, 3, CV_64FC1);
	Point2d center(0, 0);
	Mat S;
	for (auto veec : vector)
	{
		center += veec;
	}

	center.x /= vector.size(); // x mean
	center.y /= vector.size(); // y mean

	double sum = 0;

	for (auto veec : vector)
	{
		double tmp = norm(veec - center);
		sum += tmp;
		

	}

	center *= -1;

	double scale = sqrt(2.) / (sum / vector.size());

	
	
	matrix.row(0).col(0) = scale;
	matrix.row(1).col(1) = scale;
	matrix.row(0).col(2) = center.x;
	matrix.row(1).col(2) = center.y;
	return matrix;

}


Mat homographyDltSimEtimationObjectPoints(vector<Point3d>& vector)
{
	//cout << "ESTIMATION OBJECT POINTS" << endl;
	Mat matrix = Mat::eye(3, 3, CV_64FC1);
	Point3d center(0, 0, 0);
	Mat S;
	for (auto veec : vector)
	{
		center += veec;
	}

	center.x /= vector.size(); // x maen
	center.y /= vector.size(); // y mean

	double sum = 0;

	for (auto veec :vector)
	{
		double tmp = norm(veec - center);
		sum += tmp;
	}
	center *= -1;

	double scale = sqrt(2.) / (sum / vector.size());
	
	matrix.row(0).col(0) = scale;
	matrix.row(1).col(1) = scale;
	matrix.row(0).col(2) = center.x;
	matrix.row(1).col(2) = center.y;
	return matrix;
	
}

void homographyDltNormalizeImagePoints(vector<Point2d>& point, Mat& S)
{
	//cout << "NORMALIZE IMAGE POINTS" << endl;
	Mat x = Mat::zeros(3, 1, CV_64FC1), xp = Mat::zeros(3, 1, CV_64FC1);
	Mat newPoint(3, point.size(), CV_64FC1);
	for(unsigned i = 0; i < point.size(); i++)
	{
		x.at<double>(0, 0) = point[i].x;
		x.at<double>(1, 0) = point[i].y;
		x.at<double>(2, 0) = 1.;
		xp = S * x;

		//TREBA
		// xp.at<float>(0, 0) = x.dot(S.row(0));
		// xp.at<float>(1, 0) = x.dot(S.row(1));
		// xp.at<float>(2, 0) = x.dot(S.row(2));
		point[i].x = xp.at<double>(0, 0) / xp.at<double>(2, 0);
		point[i].y = xp.at<double>(1, 0) / xp.at<double>(2, 0);
		//point[i].z = 1;
	}
}

void homographyDltNormalizeObjectPoints(vector<Point3d>& point, Mat& S)
{
	//cout << "NORMALIZE OBJECT POINTS" << endl;
	Mat x = Mat::zeros(3, 1, CV_64FC1), xp = Mat::zeros(3, 1, CV_64FC1);
	for(unsigned i = 0; i < point.size(); i++)
	{
		x.at<double>(0, 0) = point[i].x;
		x.at<double>(1, 0) = point[i].y;
		x.at<double>(2, 0) = point[i].z;

		xp = S * x;

		//TREBA
		// xp.at<float>(0, 0) = x.dot(S.row(0));
		// xp.at<float>(1, 0) = x.dot(S.row(1));
		// xp.at<float>(2, 0) = x.dot(S.row(2));
		point[i].x = xp.at<double>(0, 0) / xp.at<double>(2, 0);
		point[i].y = xp.at<double>(1, 0) / xp.at<double>(2, 0);
		point[i].z = 1;
	}
}
Mat homographyDlt(vector<Point2d> &imagePoints, vector<Point3d> &objectPoints)
{
	//cout << "DLT" << endl;
	Mat img(3, 3, CV_64FC1), obj(3, 3, CV_64FC1), invTgtS(3, 3, CV_64FC1);
	Mat A = Mat::zeros(2*imagePoints.size(), 9, CV_64FC1);
	Mat H(3, 3, CV_64FC1);
	//img = homographyDltSimEtimationImagePoints(imagePoints);
	img = imagePointNomalizationMatrix(imagePoints);
	//cout << "img = " << img << endl;
	//cout << "********************" << endl;
	//obj = homographyDltSimEtimationObjectPoints(objectPoints);
	obj = objectPointNomalizationMatrix(objectPoints);
	//cout << "obj = " << obj << endl;
	//cout << "********************" << endl;

	invert(obj, invTgtS);

	homographyDltNormalizeImagePoints(imagePoints, img);
	homographyDltNormalizeObjectPoints(objectPoints, obj);

	//cout << "***********" << endl;
	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		//cout << endl << imagePoints[i];
	}

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		A.at<double>(i*2 + 0, 0) = -1*imagePoints[i].x;
		A.at<double>(i*2 + 0, 1) = -1*imagePoints[i].y;
		A.at<double>(i*2 + 0, 2) = -1.;
		A.at<double>(i*2 + 0, 3) = 0.;
		A.at<double>(i*2 + 0, 4) = 0.;
		A.at<double>(i*2 + 0, 5) = 0.;
		A.at<double>(i*2 + 0, 6) = imagePoints[i].x * objectPoints[i].x;
		A.at<double>(i*2 + 0, 7) = imagePoints[i].y * objectPoints[i].x;
		A.at<double>(i*2 + 0, 8) = objectPoints[i].x;

		A.at<double>(i*2 + 1, 0) = 0.;
		A.at<double>(i*2 + 1, 1) = 0.;
		A.at<double>(i*2 + 1, 2) = 0.;
		A.at<double>(i*2 + 1, 3) = -1*imagePoints[i].x;
		A.at<double>(i*2 + 1, 4) = -1*imagePoints[i].y;
		A.at<double>(i*2 + 1, 5) = -1.;
		A.at<double>(i*2 + 1, 6) = imagePoints[i].x * objectPoints[i].y;
		A.at<double>(i*2 + 1, 7) = imagePoints[i].y * objectPoints[i].y;
		A.at<double>(i*2 + 1, 8) = objectPoints[i].y;
		
	}

	//cout << "A = " << A.row(0) << endl;

	// Prvobitno
	//##############################################
	// Mat S, U, VT;
	
	// SVDecomp(A, U, S, VT);
	
	// cout << VT << endl;
	
	// //cout << "####################3" << endl;
	
	// Mat Vtransp = VT.t();
	// //cout << "Vt = " << Vtransp.row(8) <<  endl;
	// // cout << "Vt transpose = " << Vtransp <<  endl;
	// // cout << "####################3" << endl;
	
	
	// auto HTmp = Vtransp.col(Vtransp.cols - 1);
	// Mat h(3, 3, CV_64FC1);
	// //cout << "HTmp = " << HTmp << endl;

	// h.at<double>(0, 0) = VT.at<double>(0, 8);
	// h.at<double>(0, 1) = VT.at<double>(1, 8);
	// h.at<double>(0, 2) = VT.at<double>(2, 8);
	// h.at<double>(1, 0) = VT.at<double>(3, 8);
	// h.at<double>(1, 1) = VT.at<double>(4, 8);
	// h.at<double>(1, 2) = VT.at<double>(5, 8);
	// h.at<double>(2, 0) = VT.at<double>(6, 8);
	// h.at<double>(2, 1) = VT.at<double>(7, 8);
	// h.at<double>(2, 2) = VT.at<double>(8, 8);
	//########################################################
	//cout << h << endl;
	//cout << VT.row(8) << endl;

	if (imagePoints.size() == 4)
	{
		for (unsigned i = 0; i < 9; i++)
		{
			A.at<double>(2*imagePoints.size(), i) = 0;
		}
	}

	Mat w, u, vt;
	SVD::compute(A, w, u, vt);

	double smallestSV = w.at<double>(0, 0);
	unsigned int indexSmallestSV = 0;
	for (unsigned i = 0; i < w.rows; i++)
	{
		if (w.at<double>(i, 0) < smallestSV)
		{
			smallestSV = w.at<double>(i, 0);
			indexSmallestSV = i;
		}
	}

	Mat h = vt.row(indexSmallestSV);
	//cout << vt << endl;

	if (h.at<double>(0, 8) < 0)
	{
		h *= -1;
	}

	Mat hom(3, 3, CV_64FC1);
	for (unsigned i = 0; i < 3; i++)
	{
		for (unsigned j = 0; j < 3; j++)
		{
			hom.at<double>(i, j) = h.at<double>(0, 3*i+j);
		}
	}

	//cout << "H = " << hom << endl;

	return hom;
	

	//h = invTgtS * h * img;
	//cout << h << endl;
	//return h;

	
}

//##############CAMERA MATRIX##################//

Mat V_ij(Mat H, int i, int j)
{	
	Mat v(1, 6, CV_64FC1);

	v.at<double>(0) = H.at<double>(0, i) * H.at<double>(0, j); 
	v.at<double>(1) = H.at<double>(0, i) * H.at<double>(1, j) + H.at<double>(1, i) * H.at<double>(0, j);
	v.at<double>(2) = H.at<double>(1, i) * H.at<double>(1, j);
	v.at<double>(3) = H.at<double>(2, i) * H.at<double>(0, j) + H.at<double>(0, i) * H.at<double>(2, j);
	v.at<double>(4) = H.at<double>(2, i) * H.at<double>(1, j) + H.at<double>(1, i) * H.at<double>(2, j);
	v.at<double>(5) = H.at<double>(2, i) * H.at<double>(2, j);
	return v;
}

Mat getV(vector<Mat> &H)
{
	Mat V(2*H.size(), 6, CV_64FC1);

	for (unsigned i = 0; i < H.size(); i++)
	{
		auto v12 = V_ij(H[i], 0, 1);
		auto v11 = V_ij(H[i], 0, 0);
		auto v22 = V_ij(H[i], 1, 1);
		Mat v11_v22(1, 6, CV_64FC1); 
		subtract(v11, v22, v11_v22);

		V.at<double>(i*2, 0) = v12.at<double>(0);
		V.at<double>(i*2, 1) = v12.at<double>(1);
		V.at<double>(i*2, 2) = v12.at<double>(2);
		V.at<double>(i*2, 3) = v12.at<double>(3);
		V.at<double>(i*2, 4) = v12.at<double>(4);
		V.at<double>(i*2, 5) = v12.at<double>(5);


		V.at<double>(i*2 + 1, 0) = v11_v22.at<double>(0);
		V.at<double>(i*2 + 1, 1) = v11_v22.at<double>(1);
		V.at<double>(i*2 + 1, 2) = v11_v22.at<double>(2);
		V.at<double>(i*2 + 1, 3) = v11_v22.at<double>(3);
		V.at<double>(i*2 + 1, 4) = v11_v22.at<double>(4);
		V.at<double>(i*2 + 1, 5) = v11_v22.at<double>(5);
	}

	return V;
	
}

//******************Old calculation******************//
/*
bool intrinsics(Mat &B, double &u0, double &v0, double &lam, double &alpha, double &beta, double &gama)
{
	auto d = B.at<double>(0, 0) * B.at<double>(2, 2) - B.at<double>(0, 1) * B.at<double>(0, 1);

	if (fabs(d) < 1e-8)
	{
		cout << "d < 1e-8" << endl;
		return false;
	}

	v0 = (B.at<double>(0, 1)*B.at<double>(0, 2) - B.at<double>(0, 0)*B.at<double>(1, 2)) / (B.at<double>(0, 0)*B.at<double>(1, 1) - B.at<double>(0, 1)*B.at<double>(0, 1));
	lam = B.at<double>(2, 2) - (B.at<double>(0, 1)*B.at<double>(0, 1) + v0*(B.at<double>(0, 1)*B.at<double>(0, 2) - B.at<double>(0, 0)*B.at<double>(1, 2))) / B.at<double>(0, 0);
	auto l = (lam / B.at<double>(0, 0));
	if (l < .0)
	{
		cout << "l < 0: " << l << endl;
		return false;
	}
	alpha = sqrt(l);
	auto b = (lam*B.at<float>(0, 0))/(B.at<float>(0, 0)*B.at<float>(1, 1) - B.at<float>(0, 1)*B.at<float>(0, 1));
	//auto b = (lam / B.at<double>(0, 0));
	if (b < .0)
	{
		cout << "b < 0: " << b << endl;
		return false;
	}
	beta = sqrt(b);
	gama = (-1*B.at<double>(0, 1)*(alpha*alpha)*beta)/lam;
	u0 = (gama*v0)/beta - (B.at<double>(0, 2)*(alpha*alpha))/lam;
	return true;
}
*/
//******************New calculation******************//

bool intrinsics(Mat &B, double &u0, double &v0, double &lam, double &alpha, double &beta, double &gama)
{

	double w = B.at<double>(0, 0)*B.at<double>(1, 1)*B.at<double>(2, 2) - B.at<double>(0, 1)*B.at<double>(0, 1)*B.at<double>(2, 2) - B.at<double>(0, 0)*B.at<double>(1, 2)*B.at<double>(1, 2) + 2*B.at<double>(0, 1)*B.at<double>(0, 2)*B.at<double>(1, 2) - B.at<double>(1, 1)*B.at<double>(0, 2)*B.at<double>(0, 2);
	double d = B.at<double>(0, 0)*B.at<double>(1, 1) - B.at<double>(0, 1)*B.at<double>(0, 1);

	if (d < 0.)
	{
		d = 0.01;
	}

	alpha = sqrt(w / (d * B.at<double>(0, 0)));
	beta = sqrt(w / (d*d*B.at<double>(0, 0)));
	gama = sqrt(w / (d*d*B.at<double>(0, 0))) *B.at<double>(1);
	u0 = (B.at<double>(0, 1) * B.at<double>(1, 2) - B.at<double>(1, 1) * B.at<double>(0, 2)) / d;
	v0 = (B.at<double>(0, 1) * B.at<double>(0, 2) - B.at<double>(0, 0) * B.at<double>(1, 2)) / d;
	return true;

}

Mat getIntrinsicParameters(vector<Mat>& H_r)
{
	Mat V = getV(H_r);
	//cout << "V = " << V << endl;
	Mat U, S, Vt;
	SVDecomp(V, U, S, Vt);
	//cout << "Vt = " << Vt << endl;

	Mat VtTransposed = Vt.t();
	//cout << "Vt transposed = " << VtTransposed << endl;
	Mat b = VtTransposed.col(VtTransposed.cols - 1);
	cout << "b = " << b << endl;

	Mat B = Mat::zeros(3, 3, CV_64FC1);
	//cout << "b = " << b << endl << endl;
	B.at<double>(0, 0) = b.at<double>(0); // row 1
	B.at<double>(0, 1) = b.at<double>(1);
	B.at<double>(0, 2) = b.at<double>(3);
	B.at<double>(1, 0) = b.at<double>(1);
	B.at<double>(1, 1) = b.at<double>(2);
	B.at<double>(1, 2) = b.at<double>(4);
	B.at<double>(2, 0) = b.at<double>(3);
	B.at<double>(2, 1) = b.at<double>(4);
	B.at<double>(2, 2) = b.at<double>(5);

	cout << "B = " << B << endl;


	double u0, v0, alpha, beta, gama, lam;

	if (intrinsics(B, u0, v0, lam, alpha, beta, gama))
	{
		Mat K(3, 3, CV_64FC1);
		K.at<double>(0, 0) = alpha;
		K.at<double>(0, 1) = gama;
		K.at<double>(0, 2) = u0;
		K.at<double>(1, 0) = 0.;
		K.at<double>(1, 1) = beta;
		K.at<double>(1, 2) = v0;
		K.at<double>(2, 0) = 0.;
		K.at<double>(2, 1) = 0.;
		K.at<double>(2, 2) = 1.;
		return K;
	}
	else
	{
		cout << "Error calculating K..." << endl;
		return Mat();
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
	Mat KInv(3, 3, CV_64FC1);
	KInv = K.inv();
	//cout << KInv << endl;
	
	Mat rt(3, 4, CV_64FC1);

	Mat h1 = H.col(0);
	Mat h2 = H.col(1);
	Mat h3 = H.col(2);

	Mat r1 = KInv * h1;
	Mat r2 = KInv * h2;
	
	double lam1 = 1. / norm(r1);
	double lam2 = 1. / norm(r2);
	double lam3 = (lam1 + lam2) / 2.;

	r1 *= lam1;
	r2 *= lam2;

	Mat r3 = r1.cross(r2);
	//cout << "r3 = " << r3 << endl;

	Mat t = (KInv * h3) * lam3;

	// Mat R(3, 3, CV_32FC1);
	// Mat U, S, Vt;
	// SVDecomp(R, U, S, Vt);
	// R = U * Vt;

	// auto tmp1 = R.col(0);
	// auto tmp2 = R.col(1);
	// auto tmp3 = R.col(2);


	rt.at<double>(0, 0) = r1.at<double>(0);
	rt.at<double>(0, 1) = r2.at<double>(0);
	rt.at<double>(0, 2) = r3.at<double>(0);
	rt.at<double>(0, 3) = t.at<double>(0);
	rt.at<double>(1, 0) = r1.at<double>(1);
	rt.at<double>(1, 1) = r2.at<double>(1);
	rt.at<double>(1, 2) = r3.at<double>(1);
	rt.at<double>(1, 3) = t.at<double>(1);
	rt.at<double>(2, 0) = r1.at<double>(2);
	rt.at<double>(2, 1) = r2.at<double>(2);
	rt.at<double>(2, 2) = r3.at<double>(2);
	rt.at<double>(2, 2) = t.at<double>(2);

	return rt;
}
/*
Mat distortion(vector<vector<Point2f>> &imagePoints, vector<vector<Point2f>> &imagePointsNorm, vector<vector<Point2f>> &imageProj, Mat &K)
{
	Mat k(0, 8, CV_64FC1);
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
			//xy = sum((imagePointsNorm[i][j] * imagePointsNorm[i][j]));
			ui_uo = imageProj[i][j].x - u0;
			vi_vo = imageProj[i][j].y - v0;

			D.at<float>(i*2, 0) = ui_uo*xy;
			D.at<float>(i*2, 1) = ui_uo*xy*xy;

			D.at<float>(i*2+1, 0) = vi_vo*xy;
			D.at<float>(i*2+1, 1) = vi_vo*xy*xy;

			d.at<float>(i*2, 0) = imagePoints[i][j].x - imageProj[i][j].x;
			d.at<float>(i*2+1, 0) = imagePoints[i][j].y - imageProj[i][j].y;
		}
	}

	//Mat K = Mat(imagePoints.size()*4, imagePoints.size(), CV_64FC1);
	//solve(D, K, d, DECOMP_SVD);

	// k[0] = K.data()[0];
	// k[1] = K.data()[1];

	return k;
}
*/

//////////////////////// Normalization again ///////////////////////////////////
Mat imagePointNomalizationMatrix(vector<Point2d> &imagePoints)
{
	Mat matrix = Mat::eye(3, 3, CV_64FC1);
	Point2d center(0, 0);
	Mat S;
	for (auto veec : imagePoints)
	{
		center.x += veec.x;
		center.y += veec.y;
	}

	center.x /= imagePoints.size(); // x mean
	center.y /= imagePoints.size(); // y mean

	Point2d sum(0, 0);


	for (auto veec : imagePoints)
	{
		//double tmp = norm(veec - center);
		//sum += tmp;
		sum.x += (veec.x - center.x) * (veec.x - center.x);
		sum.y += (veec.y - center.y) * (veec.y - center.y);
	}
	//center *= -1;

	//float scale_b = sqrt(2.) / (sum / vector.size());

	Point2d var(0, 0);
	var.x = sum.x / imagePoints.size(); 
	var.y = sum.y / imagePoints.size(); 
	

	// standard deviation square root of variance
	Point2d sd(0, 0);
	sd.x = sqrt(var.x);
	sd.y = sqrt(var.y);

	double s_x = sqrt(2. / var.x);
	double s_y = sqrt(2. / var.y);

	
	matrix.row(0).col(0) = s_x;
	matrix.row(1).col(1) = s_y;
	matrix.row(0).col(2) = -s_x*center.x;
	matrix.row(1).col(2) = -s_y*center.y;
	return matrix;

}

Mat objectPointNomalizationMatrix(vector<Point3d> &objectPoints)
{
	Mat matrix = Mat::eye(3, 3, CV_64FC1);
	Point3d center(0, 0, 0);
	Mat S;
	for (auto veec : objectPoints)
	{
		center.x += veec.x;
		center.y += veec.y;
	}

	center.x /= objectPoints.size(); // x maen
	center.y /= objectPoints.size(); // y mean

	Point2d sum(0, 0);

	for (auto veec :objectPoints)
	{
		sum.x += (veec.x - center.x) * (veec.x - center.x);
		sum.y += (veec.y - center.y) * (veec.y - center.y);
	}
	//center *= -1;

	Point2d var(0, 0);
	var.x = sum.x / objectPoints.size(); 
	var.y = sum.y / objectPoints.size(); 

	double s_x = sqrt(2. / var.x);
	double s_y = sqrt(2. / var.y);


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
/*
Mat homographyLeastSquares(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints)
{
	Mat A, B, H;
	Mat h(3, 3, CV_32FC1);
	pack_ab(imagePoints, objectPoints, A, B);

	Mat _H(8, 1, CV_32FC1);

	Mat At = A.t();

	solve(At*A, At*B, _H);

	// H.create(1, 9);
	// copy(_H.begin(), _H.end(), H.begin());

	h.at<float>(0, 0) = _H.at<float>(0, 0);
	h.at<float>(0, 1) = _H.at<float>(0, 1);
	h.at<float>(0, 2) = _H.at<float>(0, 2);
	h.at<float>(1, 0) = _H.at<float>(0, 3);
	h.at<float>(1, 1) = _H.at<float>(0, 4);
	h.at<float>(1, 2) = _H.at<float>(0, 5);
	h.at<float>(2, 0) = _H.at<float>(0, 6);
	h.at<float>(2, 1) = _H.at<float>(0, 7);
	h.at<float>(2, 2) = _H.at<float>(0, 8);

	return h;
	
	
}

void pack_ab(vector<Point2f> &src_pts, vector<Point3f> &tgt_pts, Mat &A, Mat &B) {

	// construct matrices
	A = Mat::zeros(src_pts.size() * 2, 8, CV_32FC1);
	B.create(src_pts.size() * 2, 1, CV_32FC1);

	// populate matrices with data.
	for (unsigned i = 0; i < src_pts.size(); i++) {

		auto &src = src_pts[i];
		auto &tgt = tgt_pts[i];

		B.at<float>(i * 2, 0) = tgt.x;
		B.at<float>(i * 2 + 1, 0) = tgt.y;

		A.at<float>(i * 2, 0) = src.x;
		A.at<float>(i * 2, 1) = src.y;
		A.at<float>(i * 2, 2) = 1;
		A.at<float>(i * 2 + 1, 3) = src.x;
		A.at<float>(i * 2 + 1, 4) = src.y;
		A.at<float>(i * 2 + 1, 5) = 1;

		A.at<float>(i * 2, 6) = -1 * src.x * tgt.x;
		A.at<float>(i * 2, 7) = -1 * src.y * tgt.x;
		A.at<float>(i * 2 + 1, 6) = -1 * src.x * tgt.y;
		A.at<float>(i * 2 + 1, 7) = -1 * src.y * tgt.y;
	}
}
	
*/
	 



