
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdexcept>
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>

using namespace std;

void photometric_stereo_GPU(vector<cv::Mat>& images, const vector<cv::Vec3f>& lights, cv::Mat& normals, cv::Mat& pgrads, cv::Mat& qgrads, cv::Mat& albedo);