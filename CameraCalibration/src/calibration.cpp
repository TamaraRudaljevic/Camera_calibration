#include "../include/calibration.hpp"

Mat V_ij(const Mat H, int i, int j)
{	
	Mat v(1, 6, CV_32FC1);

	v.at<float>(0) = H.at<float>(0, i) * H.at<float>(0, j); 
	v.at<float>(1) = H.at<float>(0, i) * H.at<float>(1, j) + H.at<float>(1, i) * H.at<float>(0, j);
	v.at<float>(2) = H.at<float>(1, i) * H.at<float>(1, j);
	v.at<float>(3) = H.at<float>(2, i) * H.at<float>(0, j) + H.at<float>(0, i) * H.at<float>(2, j);
	v.at<float>(4) = H.at<float>(2, i) * H.at<float>(1, j) + H.at<float>(1, i) * H.at<float>(2, j);
	v.at<float>(5) = H.at<float>(2, i) * H.at<float>(2, j);
	return v;
}

Mat getV(const vector<Mat> &H)
{
	Mat V(2*H.size(), 6, CV_32FC1);

	for (unsigned i = 0; i < H.size(); i++)
	{
		auto v12 = V_ij(H[i], 0, 1);
		auto v11 = V_ij(H[i], 0, 0);
		auto v22 = V_ij(H[i], 1, 1);
		Mat v11_v22(1, 6, CV_32FC1); 
		subtract(v11, v22, v11_v22);

		V.at<float>(i*2, 0) = v12.at<float>(0);
		V.at<float>(i*2, 1) = v12.at<float>(1);
		V.at<float>(i*2, 2) = v12.at<float>(2);
		V.at<float>(i*2, 3) = v12.at<float>(3);
		V.at<float>(i*2, 4) = v12.at<float>(4);
		V.at<float>(i*2, 5) = v12.at<float>(5);


		V.at<float>(i*2 + 1, 0) = v11_v22.at<float>(0);
		V.at<float>(i*2 + 1, 1) = v11_v22.at<float>(1);
		V.at<float>(i*2 + 1, 2) = v11_v22.at<float>(2);
		V.at<float>(i*2 + 1, 3) = v11_v22.at<float>(3);
		V.at<float>(i*2 + 1, 4) = v11_v22.at<float>(4);
		V.at<float>(i*2 + 1, 5) = v11_v22.at<float>(5);
	}

	return V;
	
}

bool intrinsics(const Mat &B, float &u0, float &v0, float &lam, float &alpha, float &beta, float &gama)
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
	//auto b = (lam / B.at<double>(0, 0));
	if (b < .0)
	{
		cout << "b < 0: " << b << endl;
		return false;
	}
	beta = sqrt(b);
	gama = (-1*B.at<float>(0, 1)*(alpha*alpha)*beta)/lam;
	u0 = (gama*v0)/beta - (B.at<float>(0, 2)*(alpha*alpha))/lam;
	return true;
}

Mat getIntrinsicParameters(const vector<Mat>& H_r)
{
	Mat V = getV(H_r);
	Mat U, S, Vt;
	SVDecomp(V, U, S, Vt);

	Mat VtTransposed = Vt.t();
	Mat b = VtTransposed.col(VtTransposed.cols - 1);

	Mat B = Mat::zeros(3, 3, CV_32FC1);
	B.at<float>(0, 0) = b.at<float>(0); // row 1
	B.at<float>(0, 1) = b.at<float>(1);
	B.at<float>(0, 2) = b.at<float>(3);
	B.at<float>(1, 0) = b.at<float>(1);
	B.at<float>(1, 1) = b.at<float>(2);
	B.at<float>(1, 2) = b.at<float>(4);
	B.at<float>(2, 0) = b.at<float>(3);
	B.at<float>(2, 1) = b.at<float>(4);
	B.at<float>(2, 2) = b.at<float>(5);

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
		return K;
	}
	else
	{
		cout << "Error calculating K..." << endl;
		return Mat();
	}
}

Mat getExtrinsicsParameters(const Mat &K, const Mat &H)
{
	Mat KInv(3, 3, CV_32FC1);
	KInv = K.inv();
	
	Mat rt(3, 4, CV_32FC1);

	Mat h1 = H.col(0);
	Mat h2 = H.col(1);
	Mat h3 = H.col(2);

	Mat r1 = KInv * h1;
	Mat r2 = KInv * h2;
	
	float lam1 = 1. / norm(r1);
	float lam2 = 1. / norm(r2);
	float lam3 = (lam1 + lam2) / 2.;

	r1 *= lam1;
	r2 *= lam2;

	Mat r3 = r1.cross(r2);

	Mat t = (KInv * h3) * lam3;

	Mat R(3, 4, CV_32FC1);
	R.at<float>(0, 0) = r1.at<float>(0);
	R.at<float>(0, 1) = r2.at<float>(0);
	R.at<float>(0, 2) = r3.at<float>(0);
	R.at<float>(1, 0) = r1.at<float>(1);
	R.at<float>(1, 1) = r2.at<float>(1);
	R.at<float>(1, 2) = r3.at<float>(1);
	R.at<float>(2, 0) = r1.at<float>(2);
	R.at<float>(2, 1) = r2.at<float>(2);
	R.at<float>(2, 2) = r3.at<float>(2);
	R.at<float>(0, 3) = t.at<float>(0);
	R.at<float>(1, 3) = t.at<float>(1);
	R.at<float>(2, 2) = t.at<float>(2);
	return R;
}

float reprojError(vector<Point2f> &imagePoints, vector<Point3f> &objectPoints, Mat &homography)
{
	float err = 0;
	float sumX = 0, sumY;
	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		Mat obj = Mat::zeros(3, 1, CV_32FC1);
		obj.at<float>(0, 0) = objectPoints[i].x;
		obj.at<float>(1, 0) = objectPoints[i].y;
		obj.at<float>(2, 0) = objectPoints[i].z;
		Mat t = Mat::zeros(1, 3, CV_32FC1);
		//t = homography * obj;
		t.at<float>(0, 0) = homography.at<float>(0, 0) * obj.at<float>(0) + homography.at<float>(0, 1) * obj.at<float>(1) + homography.at<float>(0, 2) * obj.at<float>(2);
		t.at<float>(0, 1) = homography.at<float>(1, 0) * obj.at<float>(0) + homography.at<float>(1, 1) * obj.at<float>(1) + homography.at<float>(1, 2) * obj.at<float>(2);
		t.at<float>(0, 2) = homography.at<float>(2, 0) * obj.at<float>(0) + homography.at<float>(2, 1) * obj.at<float>(1) + homography.at<float>(2, 2) * obj.at<float>(2);
		
		t.at<float>(0, 0) = t.at<float>(0, 0) / t.at<float>(0, 2);
		t.at<float>(0, 1) = t.at<float>(0, 1) / t.at<float>(0, 2);
		t.at<float>(0, 2) = t.at<float>(0, 2) / t.at<float>(0, 2);
		//cout << t << endl;
		sumX = imagePoints[i].x - t.at<float>(0, 0);
		sumY = imagePoints[i].y - t.at<float>(0, 1);
		sumX = abs(sumX);
		sumY = abs(sumY);
		err += sumX + sumY;
	}
	err = sqrt(err / imagePoints.size()) / 100.0;
	return err;
}

Mat distortion(vector<vector<Point2f>>& imagePoints, vector<vector<Point3f>>& objectPoints, Mat& A)
{
	float uc, vc;
	uc = A.at<float>(0, 2);
	vc = A.at<float>(1, 2);

	Mat D = Mat::zeros(imagePoints.size()*objectPoints.size(), imagePoints.size()*objectPoints.size(), CV_32FC1);
	Mat d = Mat::zeros(imagePoints.size()*objectPoints.size(), imagePoints.size()*objectPoints.size(), CV_32FC1);

	Mat k(2, 1, CV_32FC1);

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		for (unsigned j = 0; j < objectPoints.size(); j++)
		{
			float r = 0; 
			r = sqrt((imagePoints[i][j].x - uc)*(imagePoints[i][j].x - uc) + (imagePoints[i][j].y - vc)*(imagePoints[i][j].y - vc));

			D.at<float>(i*2, 0) = (imagePoints[i][j].x - uc)*r*r;
			D.at<float>(i*2, 1) = (imagePoints[i][j].x - uc)*r*r*r*r;

			D.at<float>(i*2+1, 0) = (imagePoints[i][j].y - vc)*r*r;
			D.at<float>(i*2+1, 1) = (imagePoints[i][j].y - vc)*r*r*r*r;

			d.at<float>(i*2, 0) = imagePoints[i][j].x - objectPoints[i][j].x;
			d.at<float>(i*2+1, 0) = imagePoints[i][j].y - objectPoints[i][j].y;

		}
	}

	//cout << "D = " << d << endl;
	
	solve(D, k, d);
	//k = d * D.inv();
	return k;
}