#pragma once
#include <unordered_map>
#include <opencv2/opencv.hpp>

using namespace std;
//using namespace cv;


class ColorModel
{
	using FeatureType = std::unordered_map<std::string, int>;//单色特征
	std::unordered_map<std::string, FeatureType> model_all;
public:
	ColorModel(string model_root);
	void update(string name, vector<string> img_path);
	void update(string name, string root);

	FeatureType getFeature(vector<string> img_path);
	FeatureType getFeature(string root);

	string recognize(string root);
	float featureDis(const FeatureType& model_feature, const FeatureType& source_feature);

	int mostColor(const cv::Mat& img);
};

