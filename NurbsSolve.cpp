#include "NurbsSolve.h"

#include <iostream>

NurbsSolve::NurbsSolve()
{
    //cublasCreate_v2(&cublasH);
    //cusolverDnCreate(&cusolverHandle);
}

NurbsSolve::~NurbsSolve()
{
    //cusolverDnDestroy(cusolverHandle);
    //cublasDestroy(cublasH);
}


bool NurbsSolve::solve()
{
	//CzxTimer adgdfh(__func__);
	m_xeig = m_Keig.colPivHouseholderQr().solve(m_feig);
	//Eigen::MatrixXd x = A.householderQr().solve(b);
	//m_xeig = m_Keig.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(m_feig);

	return true;
}

//bool NurbsSolve::solveCuda()
//{
//    cusolverStatus_t status;
//
//    double* cuda_k, *cuda_f;
//    double* tau_device;
//    double* cuda_Rx;
//    double* workspace_device;
//    int* devInfo;
//
//    //F=Kx
//    
//    // 获取矩阵数据的指针
//    double* k_ptr = m_Keig.data();
//    double* f_ptr = m_feig.data();
//
//    // 在CUDA中分配设备内存
//    cudaMalloc((void**)&cuda_k, m_Keig.rows() * m_Keig.cols() * sizeof(double));
//    cudaMalloc((void**)&cuda_f, m_feig.rows() * m_feig.cols() * sizeof(double));
//    cudaMalloc((void**)&tau_device, m_Keig.cols() * sizeof(double));
//    cudaMalloc((void**)&cuda_Rx, m_feig.rows() * m_feig.cols() * sizeof(double));
//
//    // 将数据从主机内存复制到设备内存
//    cudaMemcpy(cuda_k, k_ptr, m_Keig.rows() * m_Keig.cols() * sizeof(double), cudaMemcpyHostToDevice);
//    cudaMemcpy(cuda_f, f_ptr, m_feig.rows() * m_feig.cols() * sizeof(double), cudaMemcpyHostToDevice);
//
//
//    cudaMalloc((void**)&devInfo, sizeof(int));
//
//
//    // 获取cusolverDnDgeqrf函数所需的工作空间大小
//    int workspace_size;
//    status = cusolverDnDgeqrf_bufferSize(cusolverHandle, m_Keig.rows(), m_Keig.cols(), cuda_k, m_Keig.rows(), &workspace_size);
//    if (status != CUSOLVER_STATUS_SUCCESS) {
//        std::cerr << "cusolverDnDgeqrf_bufferSize failed with error code: " << status << std::endl;
//    }
//    // 在设备上为 workspace_device 分配内存
//    cudaMalloc((void**)&workspace_device, workspace_size);
//
//    status = cusolverDnDgeqrf(cusolverHandle, m_Keig.rows(), m_Keig.cols(), cuda_k, m_Keig.rows(), tau_device, workspace_device, workspace_size, devInfo);
//
//    if (status != CUSOLVER_STATUS_SUCCESS) {
//        std::cerr << "cusolverDnDgeqrf failed with error code: " << status << std::endl;
//    }
//
//    // 计算R*x=Q^T*F, C是R*x
//    status = cusolverDnDormqr(cusolverHandle, CUBLAS_SIDE_LEFT, CUBLAS_OP_T, m_feig.rows(), m_feig.cols(), m_Keig.cols(), cuda_k, m_Keig.rows(), tau_device, cuda_Rx, m_feig.rows(), workspace_device, workspace_size, devInfo);
//    if (status != CUSOLVER_STATUS_SUCCESS) {
//        std::cerr << "cusolverDnDormqr failed with error code: " << status << std::endl;
//    }
//
//    // 解上三角方程 R * x = Q^T * b
//    double alpha = 1.0;
//    cublasStatus_t status_cublas = cublasDtrsm_v2(cublasH, CUBLAS_SIDE_LEFT, CUBLAS_FILL_MODE_UPPER, CUBLAS_OP_N, CUBLAS_DIAG_NON_UNIT, m_feig.rows(), m_feig.cols(), &alpha, cuda_k, m_Keig.rows(), cuda_Rx, m_feig.rows());//cuda_k 存疑
//
//    //cudaMemcpy(m_xeig.data(), cuda_Rx, sizeof(double) * m_xeig.rows() * m_xeig.cols(), cudaMemcpyDeviceToHost);
//
//    // 最后，记得释放设备内存
//    cudaFree(cuda_k);
//    cudaFree(cuda_f);
//    cudaFree(cuda_Rx);
//    cudaFree(tau_device);
//    cudaFree(workspace_device);
//    cudaFree(devInfo);
//
//
//	return true;
//}
