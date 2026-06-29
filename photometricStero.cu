#include"photometricStero_cu.h"
#include"czxDependence/czxTool.h"
using namespace cv;

// CUDA内核函数
__global__ void compute_normals(const float* images, float* normals, float* pgrads, float* qgrads, float* albedo, const float* lightsInv, int width, int height, int num_imgs) 
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    // 动态分配内存以获取像素值
    //float* I = (float*)malloc(num_imgs * sizeof(float));
    float I[100];
    for (int i = 0; i < num_imgs; i++) {
        I[i] = images[(i * height + y) * width + x];
    }

    // 计算法向量
    float n[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < num_imgs; j++) {
            n[i] += lightsInv[i * num_imgs + j] * I[j];
        }
    }
    float p = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    albedo[y * width + x] = p / 255.0f;
    if (p > 0) {
        n[0] /= p;
        n[1] /= p;
        n[2] /= p;
    }
    if (n[2] == 0) {
        n[2] = 1.0f;
    }

    //int legit = 1;
    //for (int i = 0; i < num_imgs; i++) {
    //    if (I[i] < 0) {
    //        legit = 0;
    //        break;
    //    }
    //}

    //if (legit) {
        normals[(y * width + x) * 3 + 0] = n[0];
        normals[(y * width + x) * 3 + 1] = n[1];
        normals[(y * width + x) * 3 + 2] = n[2];
        pgrads[y * width + x] = n[0] / n[2];
        qgrads[y * width + x] = n[1] / n[2];
    //}
    //else {
    //    normals[(y * width + x) * 3 + 0] = 0.0f;
    //    normals[(y * width + x) * 3 + 1] = 0.0f;
    //    normals[(y * width + x) * 3 + 2] = 1.0f;
    //    pgrads[y * width + x] = 0.0f;
    //    qgrads[y * width + x] = 0.0f;
    //    albedo[y * width + x] = 0.0f;
    //}

    // 释放动态分配的内存
    free(I);
}

void photometric_stereo_GPU(vector<Mat>& images, const vector<Vec3f>& lights, Mat& normals_in, Mat& pgrads_in, Mat& qgrads_in, Mat& albedo_in)
{
    if (images.size() > 100)
    {
        throw logic_error("图片数目超过100不能使用GPU");
    }
    if (images.size() != lights.size())
    {
        throw logic_error("光线数目与图片数目不一致");
    }
    if (images.size() == 0)
    {
        throw logic_error("没有图片");
    }

    //vector<Mat> images;
    //for (const auto& path : img_paths) {
    //    auto img = cv::imread(path, cv::IMREAD_GRAYSCALE);
    //    images.push_back(img);
    //}

    int height = images[0].rows;
    int width = images[0].cols;
    int num_imgs = images.size();

    Mat lightsMat(num_imgs, 3, CV_32F);
    for (int i = 0; i < num_imgs; i++) {
        lightsMat.at<float>(i, 0) = lights[i][0];
        lightsMat.at<float>(i, 1) = lights[i][1];
        lightsMat.at<float>(i, 2) = lights[i][2];
    }

    Mat lightsInv;
    cv::invert(lightsMat, lightsInv, cv::DECOMP_SVD);

    Mat normals(height, width, CV_32FC3, Scalar::all(0));
    Mat pgrads(height, width, CV_32F, Scalar::all(0));
    Mat qgrads(height, width, CV_32F, Scalar::all(0));
    Mat albedo(height, width, CV_32F, Scalar::all(0));

    // 分配和拷贝数据到GPU
    float* d_images;
    float* d_normals;
    float* d_pgrads;
    float* d_qgrads;
    float* d_albedo;
    float* d_lightsInv;
    cudaMalloc(&d_images, num_imgs * width * height * sizeof(float));
    cudaMalloc(&d_normals, width * height * 3 * sizeof(float));
    cudaMalloc(&d_pgrads, width * height * sizeof(float));
    cudaMalloc(&d_qgrads, width * height * sizeof(float));
    cudaMalloc(&d_albedo, width * height * sizeof(float));
    cudaMalloc(&d_lightsInv, 3 * num_imgs * sizeof(float));

    float* h_images = new float[num_imgs * width * height];
    for (int i = 0; i < num_imgs; i++) {
        images[i].convertTo(images[i], CV_32F);
        memcpy(h_images + i * width * height, images[i].ptr<float>(), width * height * sizeof(float));
    }
    cudaMemcpy(d_images, h_images, num_imgs * width * height * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_lightsInv, lightsInv.ptr<float>(), 3 * num_imgs * sizeof(float), cudaMemcpyHostToDevice);

    dim3 blockSize(32, 32);
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);
    cout << gridSize.x << endl;
    cout << gridSize.y << endl;
    
    compute_normals << <gridSize, blockSize >> > (d_images, d_normals, d_pgrads, d_qgrads, d_albedo, d_lightsInv, width, height, num_imgs);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error: " << cudaGetErrorString(err) << std::endl;
    }
    cudaMemcpy(normals.ptr<float>(), d_normals, width * height * 3 * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(pgrads.ptr<float>(), d_pgrads, width * height * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(qgrads.ptr<float>(), d_qgrads, width * height * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(albedo.ptr<float>(), d_albedo, width * height * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_images);
    cudaFree(d_normals);
    cudaFree(d_pgrads);
    cudaFree(d_qgrads);
    cudaFree(d_albedo);
    cudaFree(d_lightsInv);
    delete[] h_images;

    normals_in = normals;
    albedo_in = albedo;
    pgrads_in = pgrads;
    qgrads_in = qgrads;
}