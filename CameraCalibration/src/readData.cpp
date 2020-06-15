#include "../include/readData.hpp"
#include "../include/calibration.hpp"

vector<string> split_string(string s, string del = " ")
{
    std::vector<std::string> tokens;

	size_t pos = 0;
	std::string token;

	while ((pos = s.find(del)) != std::string::npos) {
		tokens.push_back(s.substr(0, pos));
		s.erase(0, pos + del.length());
	}
	tokens.push_back(s);

	return tokens;
}

bool readZhang(string readPathZhang, vector<vector<Vec2f>> &imagePointsNorm, vector<Vec3f> &modelPoints, int &w, int &h)
{
    int i;
    int n = 0;
    FILE* fmodel = fopen((readPathZhang + "/model.txt").c_str(), "rt");
    FILE* fdata1 = fopen((readPathZhang + "/data1.txt").c_str(), "rt");
    FILE* fdata2 = fopen((readPathZhang + "/data2.txt").c_str(), "rt");
    FILE* fdata3 = fopen((readPathZhang + "/data3.txt").c_str(), "rt");
    FILE* fdata4 = fopen((readPathZhang + "/data4.txt").c_str(), "rt");
    FILE* fdata5 = fopen((readPathZhang + "/data5.txt").c_str(), "rt");

    if (fmodel == NULL || fdata1 == NULL || fdata2 == NULL || fdata3 == NULL || fdata4 == NULL || fdata5 == NULL)
    {
        cout << "Error opening file..." << endl;
        return 1;
    }

    for (n=0; !feof(fmodel); n++ ) {
		double x, y;
		fscanf(fmodel,"%lf %lf ",&x,&y);
		modelPoints.push_back(Vec3f(x, y, 1.));
	}

	fclose(fmodel);

	imagePointsNorm.resize(5);
	for (i=0; i<n; i++ ) {
		double x, y;
		fscanf(fdata1,"%lf %lf ",&x,&y);
		imagePointsNorm[0].push_back(Vec2f(x, y));
		fscanf(fdata2,"%lf %lf ",&x,&y);
		imagePointsNorm[1].push_back(Vec2f(x, y));
		fscanf(fdata3,"%lf %lf ",&x,&y);
		imagePointsNorm[2].push_back(Vec2f(x, y));
		fscanf(fdata4,"%lf %lf ",&x,&y);
		imagePointsNorm[3].push_back(Vec2f(x, y));
		fscanf(fdata5,"%lf %lf ",&x,&y);
		imagePointsNorm[4].push_back(Vec2f(x, y));
	}
    //cout << imagePointsNorm[4] << endl;

	fclose(fdata1);
	fclose(fdata2);
	fclose(fdata3);
	fclose(fdata4);
	fclose(fdata5);

	w = 640;
	h = 480;
	return true;

}

bool readData(string readPath, vector<vector<Vec2f>> &imagePointNorm, vector<Vec3f> &modelPoints, int &w, int &h, float modelSize)
{
    imagePointNorm = readPattern(readPath.c_str(), w, h);
    modelPoints = objectPoint(6, 9, modelSize);

    return true;

}

vector<vector<Vec2f>> readPattern(string readPattern, int &w, int &h)
{
    vector<vector<Vec2f>> pattern;
    ifstream read(readPattern);
    string line;
    
   if(std::getline(read, line)) {
		auto l = split_string(line, " ");
		if (l.size() == 2) {
			w = std::atoi(l[0].c_str());
			h = std::atoi(l[1].c_str());
		}
	} else {
		w = h = 0;
		return pattern;
	}

    while(std::getline(read, line)) {
		pattern.push_back(vector<Vec2f>());
		auto &p = pattern.back();
		for (auto &v : split_string(line, ",")) {
			auto vec = split_string(v, " ");
			if (vec.size() == 2) {
				p.push_back(Vec2f(static_cast<float>(std::atof(vec[0].c_str())), static_cast<float>(std::atof(vec[1].c_str()))));
			}
		}
	}

    read.close();
    return pattern;
}

