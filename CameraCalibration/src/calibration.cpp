#include "calibration.hpp"

/*vector<Vec3d> get_vij(const Mat &h, unsigned i, unsigned j) {
	return vector<Vec3d> {
		h(0, i)*h(0, j), h(0, i)*h(1, j) + h(1, i)*h(0, j), h(1, i)*h(1, j),
		h(2, i)*h(0, j) + h(0, i)*h(2, j), h(2, i)*h(1, j) + h(1, i)*h(2, j), h(2, i)*h(2, j)
	};
}

Mat pack_v(const std::vector<cv::Mat> &Hs) {
    Mat v = Mat::zeros(2*Hs.size(), 6, CV_8UC3);
	//Mat v = cv::Mat::create(2*Hs.size(), 6);

	for (unsigned i = 0; i < Hs.size(); ++i) {

		auto h_r_1 = v.row(i*2);
		auto h_r_2 = v.row(i*2 + 1);

		auto v12 = get_vij(Hs[i], 0, 1);
		auto v11 = get_vij(Hs[i], 0, 0);
		auto v22 = get_vij(Hs[i], 1, 1);
		auto v11_v22 = v11 - v22;

		std::copy(v12.begin(),v12.end(),h_r_1.begin());
		std::copy(v11_v22.begin(), v11_v22.end(), h_r_2.begin());
	}

	return v;
}

vector<Vec3d> solve_b(const cv::Mat &V) {

	cv::Mat U, S, Vt;
	SVD::compute(V, U, S, Vt);

	return Vt.t().col(Vt.cols() - 1);
}

cv::Mat get_B_from_b(const vector<Vec2d> &b) {
	return {
		{b[0], b[1], b[3]},
		{b[1], b[2], b[4]},
		{b[3], b[4], b[5]}
	};
}


vector<Vec3d> objectPoints(unsigned rows, unsigned cols)
{

}

Mat normalizePoint(vector<vector<Vec2d>> chessboardCorrespondences, unsigned w, unsigned h)
{

}

Mat denormalizeIntrinsics(const Mat &K, const Mat &N)
{

}


Mat findExtrinsics(const Mat &K, const Mat &H)
{

}

Mat homographyDlt(vector<Vec2d> &chessboardCorrespondencesNormalize)
{

}

*/