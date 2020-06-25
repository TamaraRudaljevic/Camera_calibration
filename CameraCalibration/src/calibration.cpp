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

	Mat R(3, 3, CV_32FC1);
	R.at<float>(0, 0) = r1.at<float>(0);
	R.at<float>(0, 1) = r2.at<float>(0);
	R.at<float>(0, 2) = r3.at<float>(0);
	R.at<float>(1, 0) = r1.at<float>(1);
	R.at<float>(1, 1) = r2.at<float>(1);
	R.at<float>(1, 2) = r3.at<float>(1);
	R.at<float>(2, 0) = r1.at<float>(2);
	R.at<float>(2, 1) = r2.at<float>(2);
	R.at<float>(2, 2) = r3.at<float>(2);


	Mat U, S, Vt;
	SVD::compute(R, U, S, Vt);
	R.col(0) = U * Vt.row(0);
	R.col(1) = U * Vt.row(1);
	R.col(2) = U * Vt.row(2);

	Mat tmp1 = R.col(0);
	Mat tmp2 = R.col(1);
	Mat tmp3 = R.col(2);

	rt.at<float>(0, 0) = tmp1.at<float>(0);
	rt.at<float>(0, 1) = tmp2.at<float>(0);
	rt.at<float>(0, 2) = tmp3.at<float>(0);
	rt.at<float>(0, 3) = t.at<float>(0);	
	rt.at<float>(1, 0) = tmp1.at<float>(1);
	rt.at<float>(1, 1) = tmp2.at<float>(1);
	rt.at<float>(1, 2) = tmp3.at<float>(1);
	rt.at<float>(1, 3) = t.at<float>(1);
	rt.at<float>(2, 0) = tmp1.at<float>(2);
	rt.at<float>(2, 1) = tmp2.at<float>(2);
	rt.at<float>(2, 2) = tmp3.at<float>(2);
	rt.at<float>(2, 2) = t.at<float>(2);

	return rt;
}
/*
float computeReprojectionError(const vector<vector<Point3f>> &objectPoints, const vector<vector<Point2f>> &imagePoints, 
	const vector<Mat> &rvecs, const vector<Mat> &tvecs, 
	const Mat &cameraMatrix, const Mat &distCoef, 
	vector<float> &perViewErrors, bool fisheye)
{
	vector<Point2f> imagePoints2;
	float totalPoints = 0;
	float totalError = 0;
	float err;
	perViewErrors.resize(objectPoints.size());

	for (unsigned i = 0; i < objectPoints.size(); i++)
	{
		if (fisheye)
		{
			fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix, distCoef);
		} else 
		{
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoef, imagePoints2);
		}
		err = norm(imagePoints[i], imagePoints2, NORM_L2);

		unsigned n = objectPoints.size();
		perViewErrors[i] = (float)sqrt(err*err / n);
		totalError += err*err;
		totalError += n;
	}
	return sqrt(totalError / totalPoints);
}
*/

/*
float calc_reprojection(const Mat &A, const Mat &K, const vector<Point3f> &model_pts, const vector<Point2f> &image_pts, vector<Point2f> &image_pts_proj, vector<float> &k) 
{

	auto m = model_pts.size();

	image_pts_proj = vector<Point2f>(m);

	vector<float> model(4);

	float err = 0.;
	for(unsigned i = 0; i < m; i++) {

		model[0] = model_pts[i].x;
		model[1] = model_pts[i].y;
		model[2] = 0.0;
		model[3] = 1.0;

		auto proj_ptn = reproject_point(model, A, K, k);
		image_pts_proj[i].x = proj_ptn[0];
		image_pts_proj[i].y = proj_ptn[1];

		// calculate projection error
		auto x_d = image_pts[i].x - proj_ptn[0];
		auto y_d = image_pts[i].x - proj_ptn[1];

		x_d*=x_d;
		y_d*=y_d;

		err += sqrt(x_d + y_d);
	}

	return err / m;
}


vector<float> reproject_point(const vector<float> &world_ptn, const Mat &A, const Mat &K, const vector<float> &k) 
{

	// auto proj_ptn = K * world_ptn; 
	// proj_ptn /= proj_ptn[2]; 
	vector<float> proj_ptn(3);
	proj_ptn[0] = K.at<float>(0, 0) * world_ptn[0];
	proj_ptn[1] = K.at<float>(1, 0) * world_ptn[1];
	proj_ptn[2] = K.at<float>(2, 0) * world_ptn[2];

	proj_ptn[0] = proj_ptn[0] / proj_ptn[2];
	proj_ptn[1] = proj_ptn[1] / proj_ptn[2];
	proj_ptn[2] = proj_ptn[2] / proj_ptn[2];


	if (k.size() == 2) {
		float r2 = proj_ptn[0]*proj_ptn[0] + proj_ptn[1]*proj_ptn[1] + 1; 
		float d_r = (1 + k[0]*r2 + k[1]*(r2*r2)); // radial distortion
		proj_ptn[0] = proj_ptn[0]*d_r;
		proj_ptn[1] = proj_ptn[1]*d_r;
	} else if (k.size() == 4) {
		float r2 = proj_ptn[0]*proj_ptn[0] + proj_ptn[1]*proj_ptn[1] + 1; 
		float d_r = (1 + k[0]*r2 + k[1]*(r2*r2)); // radial distortion
		float d_t = 2 * k[2]*proj_ptn[0]*proj_ptn[1] + k[3]*(r2 + 2*(proj_ptn[0]*proj_ptn[0])); // tan distortion
		proj_ptn[0] = proj_ptn[0]*d_r+ d_t;
		proj_ptn[1] = proj_ptn[1]*d_r+ d_t;
	} else if (k.size() == 8) {
		float r2 = proj_ptn[0]*proj_ptn[0] + proj_ptn[1]*proj_ptn[1] + 1; 
		float r3 = proj_ptn[0]*proj_ptn[0]*proj_ptn[0] + proj_ptn[1]*proj_ptn[1]*proj_ptn[1] + 1; 
		float k_u = 1 + k[0]*r2 + k[1]*(r2*r2) + k[2]*(r3*r3);
		float k_d = 1 + k[3]*r2 + k[4]*(r2*r2) + k[5]*(r3*r3);
		float d_r = (k_d) ? k_u / k_d : 0.; // radial distortion
		float d_t = 2 * k[2]*proj_ptn[0]*proj_ptn[1] + k[3]*(r2 + 2*(proj_ptn[0]*proj_ptn[0])); // tan distortion
		proj_ptn[0] = proj_ptn[0]*d_r+ d_t;
		proj_ptn[1] = proj_ptn[1]*d_r+ d_t;
	}

	//auto pp_vec = A * vector{proj_ptn[0], proj_ptn[1], proj_ptn[2]};
	vector<float> tmp1;

	tmp1[0] = A.at<float>(0, 0) * proj_ptn[0];
	tmp1[1] = A.at<float>(1, 0) * proj_ptn[1];
	tmp1[1] = A.at<float>(2, 0) * proj_ptn[2];

	vector<float> tmp;
	tmp.push_back(tmp1[0] / tmp1[2]);
	tmp.push_back( tmp1[1] / tmp1[2]);

	return tmp;
}*/
/*
Mat distortion(vector<vector<Point2f>> &imagePoints, vector<vector<Point2f>> &imagePointsNorm, vector<vector<Point2f>> &imageProj, Mat &A)
{
	Mat k(1, 8, CV_64FC1);
	float u0, v0, ui_uo, vi_vo, xy;

	float num = imagePoints.front().size();

	Mat D(imagePoints.size()*num*2, 2, CV_64FC1);
	Mat d(imagePoints.size()*num*2, 1, CV_64FC1);

	u0 = A.at<float>(0, 2);
	v0 = A.at<float>(1, 2);

	for (unsigned i = 0; i < imagePoints.size(); i++)
	{
		for (unsigned j = 0; j < num; j++)
		{ 
			xy = imagePoints[i][j].x * imagePoints[i][j].y + imagePoints[i][j].y * imagePoints[i][j].y;
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

	Mat K = Mat(imagePoints.size()*4, imagePoints.size(), CV_64FC1);
	solve(D, K, d);

	k.at<float>(0) = K.at<float>(0);
	k.at<float>(1) = K.at<float>(1);

	return k;
}
*/

Mat distortion(vector<Point2f> &imagePoints, vector<vector<Point3f>> &objectPoints, Mat &K, vector<Mat> &RT)
{
	float uc = K.at<float>(0, 2);
	float vc = K.at<float>(1, 2);

	Mat D(RT.size()*2, 2, CV_32FC1);
	Mat d(RT.size()*2, 1, CV_32FC1);

	for (unsigned i = 0; i < RT.size(); i++)
	{
		for (unsigned j = 0; j < imagePoints.size(); i++)
		{
			Mat homog_model_coords(1, 4, CV_32FC1);
			homog_model_coords.at<float>(0, 0) = imagePoints[j].x;
			homog_model_coords.at<float>(0, 0) = imagePoints[j].y;
			homog_model_coords.at<float>(0, 0) = 0.;
			homog_model_coords.at<float>(0, 0) = 1.;

			Mat homog_coords = RT[i] * homog_model_coords;

			Mat coords = homog_coords / homog_coords.col(homog_coords.cols - 1);
			float x = coords.at<float>(0);
			float y = coords.at<float>(1);
			float r = sqrt(x*x + y*y);

			Mat P = K * homog_coords;
			P = P / P.col(2);
			float u = P.at<float>(0);
			float v = P.at<float>(1);

			float du = u - uc;
			float dv = v - vc;

			D.at<float>(i*2, 0) = du*r*r;
			D.at<float>(i*2, 1) = du*r*r*r*r;

			D.at<float>(i*2+1, 0) = dv*r*r;
			D.at<float>(i*2+1, 1) = dv*r*r*r*r;

			d.at<float>(i*2, 0) = objectPoints[i][j].x - u;
			d.at<float>(i*2+1, 0) = objectPoints[i][j].y - v;
		}
	}
}
