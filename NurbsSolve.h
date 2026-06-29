#pragma once

#include<Eigen/dense>
#include <iostream>
//#include <cuda_runtime.h>
//#include <cublas_v2.h>
//#include <cusolverDn.h>

class NurbsSolve
{
public:
	NurbsSolve();
	~NurbsSolve();
	void assign(unsigned rows, unsigned cols, unsigned dims);
	void K(unsigned i, unsigned j, double v)
	{
		m_Keig(i, j) = v;
	}
	void f(unsigned i, unsigned j, double v)
	{
		m_feig(i, j) = v;
	}
	double K(unsigned i, unsigned j)
	{
		return m_Keig(i, j);
	};

	bool solve();
	//bool solveCuda();

	void x(unsigned i, unsigned j, double v)
	{
		m_xeig(i, j) = v;
	};

	double x(unsigned i, unsigned j)
	{
		return m_xeig(i, j);
	};

public:
	Eigen::MatrixXd 	m_feig;
	Eigen::MatrixXd 	m_Keig;
	//SparseMat 	m_Ksparse;
	bool 	m_quiet;
	Eigen::MatrixXd 	m_xeig;

	//cublasHandle_t cublasH;
	//cusolverDnHandle_t cusolverHandle;
};