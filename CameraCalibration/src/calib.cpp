#include <../include/calib.hpp>

Size pattern(9,6);
string readPath = "/home/tamarar/Desktop/Novo/Camera_calibration/CameraCalibration/images/Pic_";

Mat homographyDltEstimationImg(vector<Vec2f>& imgPoints)
{
    Mat transform = Mat::eye(3, 3, CV_64FC1);
    Vec2f center(0, 0);
    Mat S;

    for (auto img : imgPoints)
    {
        center += img;
    }

    center[0] /= imgPoints.size();
    center[1] /= imgPoints.size();

    float sumDist = 0;

    for (auto img : imgPoints)
    {
        sumDist += norm(center - img);
    }

    center *= -1;

    float scale = sqrt(2.) / (sumDist / imgPoints.size());

    transform.at<float>(0, 0) = scale;
    transform.at<float>(1, 1) = scale;
    transform.at<float>(0, 2) = center[0];
    transform.at<float>(1, 2) = center[1];
    return  transform;
}

Mat homographyDltEstimationObj(vector<Vec3f>& objPoints)
{
   Mat transform = Mat::eye(3, 3, CV_64FC1);
    Vec3f center(0, 0, 0);
    Mat S;

    for (auto obj : objPoints)
    {
        center += obj;
    }

    center[0] /= objPoints.size();
    center[1] /= objPoints.size();

    float sumDist = 0;

    for (auto obj : objPoints)
    {
        sumDist += norm(center - obj);
    }

    center *= -1;

    float scale = sqrt(2.) / (sumDist / objPoints.size());

    transform.at<float>(0, 0) = scale;
    transform.at<float>(1, 1) = scale;
    transform.at<float>(0, 2) = center[0];
    transform.at<float>(1, 2) = center[1];
    return transform;
}

void homographyDltNormalizeImg(vector<Vec2f>& imgPoints, Mat& S)
{
    Mat x(3, 3, CV_64FC1), xp(3, 3, CV_64FC1);
    for (unsigned i = 0; i < imgPoints.size(); i++)
    {
        x.at<float>(0, 0) = imgPoints[i][0];
        x.at<float>(1, 0) = imgPoints[i][1];
        x.at<float>(2, 0) = 1.;
        xp = x.dot(S);
        imgPoints[i][0] = xp.at<float>(0, 0) / xp.at<float>(2, 0);
        imgPoints[0][1] = xp.at<float>(1, 0) / xp.at<float>(2, 0);
    }
}

void homographyDltNormalizeObj(vector<Vec3f>& objPoints, Mat& S)
{
    Mat x(3, 3, CV_64FC1), xp(3, 3, CV_64FC1);
    for (unsigned i = 0; i < objPoints.size(); i++)
    {
        x.at<float>(0, 0) = objPoints[i][0];
        x.at<float>(1, 0) = objPoints[i][1];
        x.at<float>(2, 0) = objPoints[i][2];
        xp = x.dot(S);
        objPoints[i][0] = xp.at<float>(0, 0) / xp.at<float>(2, 0);
        objPoints[i][1] = xp.at<float>(1, 0) / xp.at<float>(2, 0);
        objPoints[i][2] = 1.;
    }
}

Mat homographyDlt(vector<Vec2f>& src_pts, vector<Vec3f>& tgt_pts)
{
    Mat H;
    Mat srcS(3, 3, CV_64FC1), tgtS(3, 3, CV_64FC1), invTgtS;
    Mat A = Mat::zeros(2*src_pts.size(), 9, CV_64FC1);

    srcS = homographyDltEstimationImg(src_pts);
    tgtS = homographyDltEstimationObj(tgt_pts);

    auto src_n = src_pts;
    auto tgt_n = tgt_pts;

    invTgtS = tgtS.clone();
    invert(invTgtS, invTgtS);
    homographyDltNormalizeImg(src_n, srcS);
    homographyDltNormalizeObj(tgt_n, tgtS);

    for (unsigned i = 0; i < src_pts.size(); i++)
    {
        A.at<float>(i*2+0, 0) = -1*src_n[i][0];
        A.at<float>(i*2+0, 1) = -1*src_n[i][1];
        A.at<float>(i*2+0, 2) = -1;
        A.at<float>(i*2+0, 3) = 0.;
        A.at<float>(i*2+0, 4) = 0.;
        A.at<float>(i*2+0, 5) = 0.;
        A.at<float>(i*2+0, 6) = tgt_n[i][0]*src_n[i][0];
        A.at<float>(i*2+0, 7) = tgt_n[i][0] * src_n[i][1];
        A.at<float>(i*2+0, 8) = tgt_n[i][0];

        A.at<float>(i*2+1, 0) = 0.;
        A.at<float>(i*2+1, 1) = 0.;
        A.at<float>(i*2+1, 2) = 0.;
        A.at<float>(i*2+1, 3) = -1*src_n[i][0];
        A.at<float>(i*2+1, 4) = -1*src_n[i][1];
        A.at<float>(i*2+1, 5) = -1;
        A.at<float>(i*2+1, 6) = tgt_n[i][1] * src_n[i][0];
        A.at<float>(i*2+1, 7) = tgt_n[i][1] * src_n[i][1];
        A.at<float>(i*2+1, 8) = tgt_n[i][1];
    }

    Mat S, U, VT;
	SVDecomp(A, U, S, VT);

	H = Mat(VT.row(8));
	H = H.reshape(3, 3);

	Mat h(3, 3, CV_64FC1);
	h.at<float>(0, 0) = VT.at<float>(8, 0);
	h.at<float>(0, 1) = VT.at<float>(8, 1);
	h.at<float>(0, 2) = VT.at<float>(8, 2);
	h.at<float>(1, 0) = VT.at<float>(8, 3);
	h.at<float>(1, 1) = VT.at<float>(8, 4);
	h.at<float>(1, 2) = VT.at<float>(8, 5);
	h.at<float>(2, 0) = VT.at<float>(8, 6);
	h.at<float>(2, 1) = VT.at<float>(8, 7);
	h.at<float>(2, 2) = VT.at<float>(8, 8);

	h.dot(invTgtS);
	//cout << "h size novo = " << h.size() << endl;
	h.dot(srcS);
	//gemm(invTgtS, h, 1.0, noArray(), 0.0, h); //* img;
	//gemm(H, srcS, 1.0, noArray(), 0.0, h);
	return h;

}

vector<Vec3f> obj(unsigned row, unsigned col, float squarSize)
{
	vector<Vec3f> objectPoint;
    objectPoint.reserve(row*col);

	for (unsigned i = 0; i < row; i++)
	{
		for (unsigned j = 0; j < col; j++)
		{
			objectPoint.push_back({j*squarSize, i*squarSize, 1.});
		}
	}

	return objectPoint;
}

vector<vector<Vec2f>> imageCorners()
{
	vector<Vec2f> corners;
    vector<vector<Vec2f>> imagePoint;
	//vector<Point3f> objPoint = objectPoint(9, 6, 1.);
	bool patternfound;
	
	for (int i = 1; i < 13; i++)
	{
		Mat image = imread(readPath + to_string(i) + ".png");
		patternfound = findChessboardCorners(image, pattern, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (patternfound)
		{
			cout << "Corners found!" << endl;
			//imagePoints.insert(imagePoints.end(), corners.begin(), corners.end());
            imagePoint.push_back(corners);
			//objectPoints.insert(objectPoints.end(), objPoint.begin(), objPoint.end());
			//imagePoints.push_back(corners);
			//objectPoints.push_back(objPoint);
		}
	}

	return imagePoint;
}

Mat normalizeImagePoints(vector<vector<Vec2f>> &points, unsigned w, unsigned h)
{
    float sx = 2. / w;
    float sy = 2. / h;
    float x0 = w / 2.;
    float y0 = h / 2.;

    for (unsigned i = 0; i < points.size(); i++)
    {
        for (unsigned j = 0; j < points[i].size(); j++)
        {
            points[i][j][0] = sx*(points[i][j][0] - x0);
            points[i][j][1] = sy*(points[i][j][1] - y0);
        }
    }

    Mat norm(3, 3, CV_64FC1);
    norm.at<float>(0, 0) = sx;
    norm.at<float>(0, 1) = 0.;
    norm.at<float>(0, 2) = -sx*x0;
    norm.at<float>(1, 0) = 0.;
    norm.at<float>(1, 1) = sy;
    norm.at<float>(1, 2) = -sy*y0;
    norm.at<float>(2, 0) = 0.;
    norm.at<float>(2, 1) = 0.;
    norm.at<float>(2, 2) = 1.;

    return norm;
}