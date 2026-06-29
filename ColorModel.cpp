#include "ColorModel.h"
#include"czxDependence/czxTool.h"
using namespace cv;

std::string getLastFolder(const std::string& path) 
{
    size_t found = path.find_last_of("/\\");
    if (found != std::string::npos) {
        return path.substr(found + 1);
    }
    return path; // 如果路径中没有分隔符，则返回整个路径
}

ColorModel::ColorModel(string model_root)
{
    auto dic_list = arsenal::getSubdirectories(model_root);
    for (auto& dic : dic_list)
    {
        string name = getLastFolder(dic);
        update(name, dic);
    }
}

void ColorModel::update(string name, vector<string> img_path)
{
    model_all[name] = getFeature(img_path);
}

void ColorModel::update(string name, string root)
{
    model_all[name] = getFeature(root);
}

ColorModel::FeatureType ColorModel::getFeature(vector<string> img_path)
{
    FeatureType model_single;
    #pragma omp parallel for
    for (int i=0;i<img_path.size();i++)
    {
        auto path = img_path[i];
        cv::Mat img;
        {
            //CzxTimer _("load img");
            img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        }
        {
            //CzxTimer _(__func__);
            model_single[getLastFolder(path)] = mostColor(img);
        }
    }
    return model_single;
}

ColorModel::FeatureType ColorModel::getFeature(string root)
{
    auto path_list = arsenal::pathGather(root, "*.png");
    return getFeature(path_list);
}

string ColorModel::recognize(string root)
{
    auto path_list = arsenal::pathGather(root, "*.png");
    auto feature = getFeature(root);
    float min_dis = std::numeric_limits<float>::max();
    string ret = "NULL";
    for (auto& feature_pair : model_all)
    {
        float cur_dis = featureDis(feature_pair.second, feature);
        if (cur_dis < min_dis)
        {
            ret = feature_pair.first;
            min_dis = cur_dis;
        }
    }
    return ret;
}

float ColorModel::featureDis(const FeatureType& model_feature, const FeatureType& source_feature)
{
    float sum = 0;
    int num = 0;
    // 使用迭代器遍历
    for (auto it = source_feature.begin(); it != source_feature.end(); ++it) 
    {
        if (model_feature.find(it->first) != model_feature.end())
        {
            int dis = abs(model_feature.at(it->first) - it->second);
            sum += dis;
            num++;
        }
        else
        {
            std::stringstream msg;
            msg << "模型不存在 " << it->first << "的特征，请检查照片命名";
            throw logic_error(msg.str());
        }
    }
    if (num == 0)
    {
        throw logic_error("没有找到与模型配对的照片");
    }
    return sum / num;
}

int ColorModel::mostColor(const cv::Mat& image)
{
    if (image.channels() != 1)
    {
        throw logic_error("输入图片不是灰度图");
    }

    // 定义直方图参数
    int histSize = 256 / 5;  // 256个灰度级，分成5个单位
    float range[] = { 0, 256 };
    const float* histRange = { range };

    cv::SparseMat hist;
    // 计算直方图
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    // 找到直方图中的最大值位置
    double maxVal = 0;
    int maxIdx = 0;
    double minVal = 0;
    int minIdx = 0;
    cv::minMaxLoc(hist, &minVal, &maxVal, &minIdx, &maxIdx);

    // 计算众数对应的灰度值范围
    int modeValue = maxIdx * 5;
    return modeValue;
}
