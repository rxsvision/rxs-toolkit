// dllmain.cpp : 定义 DLL 应用程序的入口点。


//HMODULE hDLL = LoadLibrary(L"czxToolkit.dll");
//typedef vector<PointT>(__cdecl* MYTYPE)(CP cloud, double min_x, double max_x, double min_y, double max_y);
//MYTYPE semiconductor = (MYTYPE)GetProcAddress(hDLL, "semiconductor");

#include "pch.h"
#include"nanoflann.hpp"
#include <direct.h> // 包含 _mkdir 函数


#define CZX_DEBUG

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{

    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

/// <summary>
/// 
/// </summary>
/// <param name="cloud">输入点云</param>
/// <param name="visual">是否可视化</param>
/// <returns>输出弧长，2d长度，轮廓度</returns>
__declspec(dllexport) vector<double> fitBSpline_(CP cloud, bool visual=false)
{
    BSplineLength bsp;
    bsp.visual = visual;
    return bsp.fitBSpline(cloud);
}

CP findPlane(CP cloud, double min_x, double max_x, double min_y, double max_y)
{
    CP plane(new CloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*plane);
    pass.setInputCloud(plane);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*plane);
    return plane;
}

typedef bool (*VerifyFunction)(CP, PointT&);

bool verifyBall(CP cloud, PointT& highest)
{
	//pcl::PointXYZ min_point, max_point;
	//pcl::getMinMax3D(*cloud, min_point, max_point);
	//if (max_point.x - min_point.x > 0.8) return false;
	//highest = max_point.z;

	float min_x = 9999, max_x = -9999;
	float max_z = -9999;
	for (auto p : *cloud)
	{
		if (p.x < min_x) min_x = p.x;
		if (p.x > max_x)
		{
			max_x = p.x;
		}
		if (p.z > max_z)
		{
			highest = p;
			max_z = p.z;
		}
	}
	if (max_x - min_x > 0.8) return false;
	return true;
}

vector<PointT> highestPointGather(CP cloud, double tolerance, VerifyFunction verify)
{
	vector<PointT> highs;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance); // 设置欧氏聚类的阈值
	ec.setMinClusterSize(1000);    // 设置最小聚类尺寸
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);


	for (auto indices : cluster_indices)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(indices));
		extract.setIndices(cluster_indices_ptr);
		extract.setNegative(false); // 设置为false，表示提取指定索引的点

		// 提取聚类的点
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		extract.filter(*cluster_cloud);

		PointT highest;
		if (verify(cluster_cloud, highest))
		{
			//ret.push_back(cluster_cloud);
			highs.push_back(highest);
		}
	}
	return highs ;
}

__declspec(dllexport) vector<PointT> semiconductor(CP cloud, double min_x, double max_x, double min_y, double max_y)
{
    vector<PointT> ret;
    CP plane = findPlane(cloud, min_x, max_x, min_y, max_y);
    if (plane->size() > 100) {}
    else return ret;
    //CzxLog("log.czx") << "plane size" << plane->size() << endl;

    pcl::PCA<PointT> pca;
    pca.setInputCloud(plane);
    Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
    if (projection(2, 2) < 0)
    {
        projection = -projection;
    }
    cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);
    plane->getMatrixXfMap(3, 4, 0) = projection * plane->getMatrixXfMap(3, 4, 0);
    float z_mean = plane->getMatrixXfMap(3, 4, 0).rowwise().mean()[2];
    cloud->getMatrixXfMap(3, 4, 0).row(2) -= Eigen::VectorXf::Constant(cloud->size(), z_mean);
    //CzxLog("log.czx") << "pca done" << endl;

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(max_point.z - 0.6, FLT_MAX);
	CP filtered(new CloudT);
    pass.filter(*filtered);

    if (filtered->size() < 1000) return ret;
    ret = highestPointGather(filtered, 0.02, verifyBall);
	return ret;
}

bool verifyPIN(CP cloud, PointT& highest)
{
    float max_z = -9999;
    for (auto p : *cloud)
    {

        if (p.z > max_z)
        {
            highest = p;
            max_z = p.z;
        }
    }
    return true;
}

__declspec(dllexport) vector<PointT> PIN(CP cloud, double min_x, double max_x, double min_y, double max_y)
{
    CP plane = findPlane(cloud, min_x, max_x, min_y, max_y);

    pcl::PCA<PointT> pca;
    pca.setInputCloud(plane);
    Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
    if (projection(2, 2) < 0)
    {
        projection = -projection;
    }
    cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);
    plane->getMatrixXfMap(3, 4, 0) = projection * plane->getMatrixXfMap(3, 4, 0);
    float z_mean = plane->getMatrixXfMap(3, 4, 0).rowwise().mean()[2];
    cloud->getMatrixXfMap(3, 4, 0).row(2) -= Eigen::VectorXf::Constant(cloud->size(), z_mean);

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(max_point.z - 0.6, FLT_MAX);
    CP filtered(new CloudT);
    pass.filter(*filtered);

    auto ret = highestPointGather(filtered, 0.02, verifyPIN);
    return ret;
}

CP gatherCenter(CP cloud, string hole_pos_path)
{
    CzxTimer sfag(__func__);
    CP ret(new CloudT);
    vector<double> bound_points = arsenal::readDoubleFromFile(hole_pos_path);
    CP h1(new CloudT);
    CP h2(new CloudT);
    CP h3(new CloudT);
    CP b1, b2, b3;
    PointT c1, c2, c3;

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            Hole ho;
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(bound_points[0], bound_points[1]);
                pass.filter(*h1);
                pass.setInputCloud(h1);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(bound_points[2], bound_points[3]);
                pass.filter(*h1);
                #ifdef CZX_DEBUG
                cout << "h1 num" << h1->size() << endl;
                cout << "h1 done" << endl;
                Tool::show(h1);
                #endif
            }
            b1 = ho.KNNBoundExtract(h1);
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(b1);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(bound_points[0] + 0.02, bound_points[1] - 0.02);
                pass.filter(*b1);
                pass.setInputCloud(b1);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(bound_points[2] + 0.02, bound_points[3] - 0.02);
                pass.filter(*b1);
                #ifdef CZX_DEBUG
                Tool::show(b1);
                #endif
                c1 = ho.centerFit(b1);
            }
        }

        #pragma omp section
        {
            Hole ho;
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(bound_points[0 + 4], bound_points[1 + 4]);
                pass.filter(*h2);
                pass.setInputCloud(h2);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(bound_points[2 + 4], bound_points[3 + 4]);
                pass.filter(*h2);
                #ifdef CZX_DEBUG
                cout << "h2 num" << h2->size() << endl;
                Tool::show(h2);
                #endif
            }
            b2 = ho.KNNBoundExtract(h2);
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(b2);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(bound_points[0 + 4] + 0.02, bound_points[1 + 4] - 0.02);
                pass.filter(*b2);
                pass.setInputCloud(b2);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(bound_points[2 + 4] + 0.02, bound_points[3 + 4] - 0.02);
                pass.filter(*b2);
                #ifdef CZX_DEBUG
                Tool::show(b2);
                #endif
            }
            c2 = ho.centerFit(b2);
        }

        #pragma omp section
        {
            Hole ho;
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(cloud);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(bound_points[0 + 4 + 4], bound_points[1 + 4 + 4]);
                pass.filter(*h3);
                pass.setInputCloud(h3);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(bound_points[2 + 4 + 4], bound_points[3 + 4 + 4]);
                pass.filter(*h3);
                #ifdef CZX_DEBUG
                cout << "h3 num" << h3->size() <<endl;
                Tool::show(h3);
                #endif
            }
            b3 = ho.KNNBoundExtract(h3);
            {
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud(b3);
                pass.setFilterFieldName("x");
                pass.setFilterLimits(bound_points[0 + 4 + 4] + 0.02, bound_points[1 + 4 + 4] - 0.02);
                pass.filter(*b3);
                pass.setInputCloud(b3);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(bound_points[2 + 4 + 4] + 0.02, bound_points[3 + 4 + 4] - 0.02);
                pass.filter(*b3);
                #ifdef CZX_DEBUG
                Tool::show(b3);
                #endif
            }
            c3 = ho.centerFit(b3);
        }
    }
    ret->push_back(c1);
    ret->push_back(c2);
    ret->push_back(c3);
    return ret;
}

__declspec(dllexport) Eigen::Matrix4f registration(CP c1, CP c2, string hole_pos_path_x, string hole_pos_path_y)
{
    CzxTimer sfag(__func__);
    CP x_hole, y_hole;
    omp_set_nested(true);
    #pragma omp parallel sections
    {
        #pragma omp section
        x_hole = gatherCenter(c1, hole_pos_path_x);
        #pragma omp section
        y_hole = gatherCenter(c2, hole_pos_path_y);
    }
    *c1 = *x_hole;
    *c2 = *y_hole;
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    transformation_estimation.estimateRigidTransformation(*x_hole, *y_hole, transformation);
    return transformation;
}

float computeFlatness(CP cloud, Matrix3f rotation, PointT& min_point, PointT& max_point)
{
    CzxTimer dgsa(__func__);
    cloud->getMatrixXfMap(3, 4, 0) = rotation * cloud->getMatrixXfMap(3, 4, 0);
    double max = -9999, min = 9999;
    for (auto p : *cloud)
    {
        if (p.z > min)
        {
        }
        else
        {
            min = p.z;
            min_point = p;
        }

        if (p.z < max) {}
        else {
            max = p.z;
            max_point = p;
        }
    }
    return max_point.z - min_point.z;
}

vector<CP> spilite(CP cloud, string path, int step = 4)
{
    vector<CP> ret;
    auto plane_pos = arsenal::readDoubleFromFile(path);
    if (plane_pos.size() % step != 0) throw std::runtime_error("参数数目不正确");
    for (int i = 0; i < plane_pos.size() / step; i++)
    {
        CP plane(new CloudT);
        *plane = *cloud;
        arsenal::passThrough(plane, plane_pos[step * i], plane_pos[step * i + 1], plane_pos[step * i + 2], plane_pos[step * i + 3]);
        ret.push_back(plane);
    }
    return ret;
}

VectorXf planeFitLSM(CP cloud)
{
    CzxTimer dsghfa(__func__);
    MatrixXf xyz = cloud->getMatrixXfMap(3, 4, 0);
    VectorXf z = xyz.row(2);
    xyz.row(2).setOnes();
    Eigen::VectorXf coff = Eigen::JacobiSVD<Eigen::MatrixXf>(xyz.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV).solve(z);
    return coff;
}
/// <summary>
/// 计算平面度
/// </summary>
/// <param name="cloud">输入平面点云</param>
/// <param name="min_point">输出的最低点</param>
/// <param name="max_point">输出最高点</param>
/// <returns></returns>
__declspec(dllexport) float Flatness(CP cloud, PointT& min_point, PointT& max_point)
{
    CzxTimer dsghfa(__func__);
    auto coff = planeFitLSM(cloud);
    Vector3f normal = Vector3f(coff[0], coff[1], -1).normalized();
    VectorXf z = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);
    int max_index = 0;
    float max_value = z.maxCoeff(&max_index);
    int min_index = 0;
    float min_value = z.minCoeff(&min_index);
    min_point = cloud->points[min_index];
    max_point = cloud->points[max_index];
    return max_value - min_value;
}
MatrixXf buildCoordinate(VectorXf normal)
{
    Vector3f z_axis = normal.normalized();
    Eigen::Vector3f x_axis = z_axis.unitOrthogonal();
    Eigen::Vector3f y_axis = z_axis.cross(x_axis);
    Eigen::MatrixXf rotation(3, 3);
    rotation << x_axis, y_axis, z_axis;
    return rotation.transpose();
}

__declspec(dllexport) void keyboard(CP cloud, string initial_plane_pos, vector<string> verify_path, vector<CP>& planes, vector<vector<float>>& flatness, MatrixXf& offset)
{
    CzxTimer dsghfa(__func__);

    auto plane_pos = arsenal::readDoubleFromFile(initial_plane_pos);
    if (plane_pos.size() % 2 != 0 || plane_pos.size() / 2 != verify_path.size())
    {
        CzxLog("log.czx") << "参数数目不正确" << endl;
        throw std::runtime_error("参数数目不正确");
    }

    vector<CP> ini_plane;
    vector<Matrix3f> plane_coff;

    for (int i = 0; i < plane_pos.size() / 2; i++)
    {
        CP plane(new CloudT);
        *plane = *cloud;
        arsenal::passThrough(plane, plane_pos[2 * i] - 1, plane_pos[2 * i] + 1, plane_pos[2 * i + 1] - 1, plane_pos[2 * i + 1] + 1);
        ini_plane.push_back(plane);
    }
    for(auto p:ini_plane)
        CzxLog("log.czx") << "p size: " << p->size() << endl;

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    for (int i = 0; i < ini_plane.size(); i++)
    {
        auto coff = planeFitLSM(ini_plane[i]);
        Vector3f normal = Vector3f(coff[0], coff[1], -1).normalized();


        CP final_plane(new CloudT);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        {
            CzxTimer sdghsd("extract points");
            VectorXf z = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);
            float z_mean = (normal.transpose() * ini_plane[i]->getMatrixXfMap(3, 4, 0)).mean();

            for (int index = 0; index < z.size(); ++index) {
                if (z[index] > z_mean - 0.1 && z[index] < z_mean + 0.1)
                {
                    indices->indices.push_back(index);
                }
            }
            extract.setIndices(indices);
            extract.filter(*final_plane);
            CzxLog("log.czx") << "final size: " << final_plane->size() << endl;

        }


        coff = planeFitLSM(final_plane);
        normal = Vector3f(coff[0], coff[1], -1).normalized();
        indices->indices.clear();
        {
            CzxTimer sdghsd("extract more points");
            VectorXf z = normal.transpose() * cloud->getMatrixXfMap(3, 4, 0);
            float z_mean = (normal.transpose() * final_plane->getMatrixXfMap(3, 4, 0)).mean();

            for (int index = 0; index < z.size(); ++index) {
                if (z[index] > z_mean - 0.1 && z[index] < z_mean + 0.1)
                {
                    indices->indices.push_back(index);
                }
            }
            extract.setIndices(indices);
            extract.filter(*final_plane);
        }

        auto rot = buildCoordinate(normal);
        plane_coff.push_back(rot);
        planes.push_back(final_plane);

        auto verify_planes = spilite(cloud, verify_path[i]);
        vector<float> flatness_one;
        for (auto cl : verify_planes)
        {
            PointT max_point, min_point;
            flatness_one.push_back(computeFlatness(cl, rot, min_point, max_point));
        }
        flatness.push_back(flatness_one);
    }


    offset = MatrixXf::Zero(planes.size(), planes.size());
    {
        CzxTimer dasg("compute offset");
        for (int i = 0; i < planes.size(); i++)
        {
            for (int j = i + 1; j < planes.size(); j++)
            {
                CP ci = planes[i];
                CP cj = planes[j];

                auto evsi = plane_coff[i];
                auto evsj = plane_coff[j];
                float off_i, off_j;
                {
                    CP copy_ci(new CloudT);
                    *copy_ci = *ci;
                    CP copy_cj(new CloudT);
                    *copy_cj = *cj;
                    copy_ci->getMatrixXfMap(3, 4, 0) = evsi * copy_ci->getMatrixXfMap(3, 4, 0);
                    copy_cj->getMatrixXfMap(3, 4, 0) = evsi * copy_cj->getMatrixXfMap(3, 4, 0);
                    off_i = copy_ci->getMatrixXfMap(3, 4, 0).row(2).mean() - copy_cj->getMatrixXfMap(3, 4, 0).row(2).mean();
                }

                //{
                //	CP copy_ci(new CloudT);
                //	*copy_ci = *ci;
                //	CP copy_cj(new CloudT);
                //	*copy_cj = *cj;
                //	copy_ci->getMatrixXfMap(3, 4, 0) = evsj * copy_ci->getMatrixXfMap(3, 4, 0);
                //	copy_cj->getMatrixXfMap(3, 4, 0) = evsj * copy_cj->getMatrixXfMap(3, 4, 0);
                //	off_j = copy_ci->getMatrixXfMap(3, 4, 0).row(2).mean() - copy_cj->getMatrixXfMap(3, 4, 0).row(2).mean();
                //}

                //offset(i, j) = (off_i + off_j) / 2;
                //offset(j, i) = (off_i + off_j) / 2;

                offset(i, j) = off_i;
                offset(j, i) = off_i;
            }
        }
    }
}

__declspec(dllexport) pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 创建凸包对象
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(3);
    // 计算凸包
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*hull_cloud);

    return hull_cloud;
}

using namespace nanoflann;

struct PointCloudAdapter {
    const CloudT& cloud;

    inline size_t kdtree_get_point_count() const { return cloud.points.size(); }

    inline float kdtree_distance(const float* p1, const size_t idx_p2, size_t /*size*/) const {
        const float d0 = p1[0] - cloud.points[idx_p2].x;
        const float d1 = p1[1] - cloud.points[idx_p2].y;
        return d0 * d0 + d1 * d1;
        //const float d2 = p1[2] - cloud.points[idx_p2].z;
        //return d0 * d0 + d1 * d1 + d2 * d2;  // Euclidean distance
    }

    inline float kdtree_get_pt(const size_t idx, int dim) const {
        switch (dim) {
        case 0: return cloud.points[idx].x;
        case 1: return cloud.points[idx].y;
        case 2: return cloud.points[idx].z;
        }
        return 0;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

CP surfaceFilter(CP cloud, CONFTYPE config)
{
    CzxTimer _(__func__);
    PointCloudAdapter cloudAdapter{ *cloud };
    typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, PointCloudAdapter>, PointCloudAdapter, 2> KDTree;
    KDTree index(2, cloudAdapter, KDTreeSingleIndexAdaptorParams(stoi(config["nano_leaf"])));
    {
        CzxTimer _("build index");
        index.buildIndex();
    }
    // Perform radius search
    const float search_radius = stof(config["radius_1"]);
    int threshold = stoi(config["threshold_1"]);

    CP ret(new CloudT);

    nanoflann::SearchParams params;
    #pragma omp parallel for
    for (int i = 0; i < cloud->size(); ++i)
    {
        vector<pair<size_t, float>> ret_matches;
        const float query_point[3] = { cloud->points[i].x, cloud->points[i].y, cloud->points[i].z };
        size_t num_matches = index.radiusSearch(query_point, search_radius * search_radius, ret_matches, params);

        if (num_matches > threshold)
        {
            #pragma omp critical		
            ret->push_back(cloud->points[i]);
        }
    }
    return ret;
}

__declspec(dllexport) double surfaceProfile(CP cloud, int zone_num)
{
    if (cloud->size() < 1000)
    {
        CzxLog() << "点数目不足1000" << endl;
        return -1;
    }

    auto conf = czxTool::readProfile("conf_surface.czx");
    cloud = surfaceFilter(cloud, conf);
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f projection = pca.getEigenVectors().transpose();
    cloud->getMatrixXfMap(3, 4, 0) = projection * cloud->getMatrixXfMap(3, 4, 0);

    OMPSurface sur(zone_num);
    sur.setInputCloud(cloud);
    sur.process();
    return sur.max_profile;
}

/// <summary>
/// 
/// </summary>
/// <param name="cloud">输入待测体积的点云/// </param>
/// <param name="xyInterval">点之间xy的间隔，如果xy间隔不一致，可以取它们乘积的开方</param>
/// <returns>输出待测点云与xy平面围成的体积</returns>
__declspec(dllexport) double volume(CP cloud, double xyInterval)
{
    double sum = 0;
    for (auto p : *cloud)
    {
        sum += xyInterval * xyInterval * p.z;
    }
    return sum;
}

__declspec(dllexport) void initModelNake(CP model)
{
    LaptopLine lapline(model);
    lapline.initModel(model);
}

/// <summary>
/// 
/// </summary>
/// <param name="model">初始化之后的数模</param>
/// <param name="cloud">拍到的点云</param>
/// <returns></returns>
__declspec(dllexport) vector<float> computeDeformationNake(CP model, CP cloud)
{
    LaptopLine lapline(cloud);
    if (model)
        lapline.model = model;
    lapline.mainProcess();
    auto ret = lapline.getOutput();
    return ret;
}

/// <summary>
/// 
/// </summary>
/// <param name="model">数模</param>
/// <param name="cloud">点云</param>
/// <param name="type">笔记本类型
/*	enum LaptopType 
{
带网孔蓝色笔记本,
蓝色笔记本,
键盘,
装配键盘,
装配背板,
装配背板_普雷茨特,
装配键盘_普雷茨特,
NB6767D_GYG9D,
NB6837A,
IDC40,
IDC401X
    };
    */
/// </param>
/// <returns>变形度，锋锐度，2D长度，左间隙，右间隙，上落差，下落差，数值为-1代表没有计算成功</returns>
__declspec(dllexport) vector<float> LaptopCalculation(CP model, CP cloud, int type)
{
    LaptopLine lapline(cloud, type);
    lapline.model = model;
    if(model)
        lapline.initModel(model);
    lapline.mainProcess();
    auto ret = lapline.getOutput();
    return ret;
}

/// <summary>
/// 
/// </summary>
/// <param name="model">模型点云，必须无遮挡</param>
/// <param name="source">待测点云，无要求</param>
/// <param name="interval_x">模型点云的x间距</param>
/// <param name="interval_y">模型点云的y间距</param>
/// <returns>输出与待测点云等长的数组，每个值代表对应待测点云中的点到模型的距离</returns>
__declspec(dllexport) vector<float> dis2dis(CP model, CP source, float interval_x, float interval_y)
{
    PointGrid  model_grid(model, interval_x, interval_y);
    std::vector<float> values(source->size());
    #pragma omp parallel for
    for (int i = 0; i < source->size(); i++)
    {
        auto& p = source->points[i];
        int r, c;
        model_grid.indexCalculate(p, r, c);
        float dis;
        dis = model_grid.getNearestPoint(p, r, c);
        values[i] = dis;
    }
    return values;
}

/// <summary>
/// 
/// </summary>
/// <param name="model_root">输入模型的路径，路径下有各色文件夹，各色文件夹下以颜色命名，文件夹内是对应颜色的图片，图片以打光颜色命名</param>
/// <param name="object_directory">物体文件夹路径，文件夹内各图片以打光颜色命名</param>
/// <returns>返回预测的颜色，颜色范围取决于模型文件夹下的各文件</returns>
__declspec(dllexport) string colorReg(string model_root, string object_directory)
{
    ColorModel cm(model_root);
    return cm.recognize(object_directory);
}

// 判断 G 和 B 通道的值之和是否大于 n，并输出掩码图像
cv::Mat createMaskForGBSumGreaterThanN(const cv::Mat& input, float n) {
    // 创建一个和输入图像一样大小的单通道掩码图像，初始为0（黑色）
    cv::Mat mask = cv::Mat::zeros(input.size(), CV_8UC1);

    // 遍历每个像素，判断 G 和 B 通道的值之和是否大于 n
    for (int i = 0; i < input.rows; ++i) {
        for (int j = 0; j < input.cols; ++j) {
            // 获取当前像素的 G 和 B 通道值
            cv::Vec3f pixel = input.at<cv::Vec3f>(i, j);
            float g = pixel[1];  // G 通道
            float r = pixel[2];  // B 通道
            //float g = 0;

            // 如果 G + B 大于 n，设置掩码图像中的对应像素为255（白色）
            if (g + r > n) {
                mask.at<uchar>(i, j) = 255;
            }
        }
    }

    return mask;
}

cv::Mat erodeAndDilate(const cv::Mat& input, int kernelSize = 7)
{
    // 创建十字型结构元素
    cv::Mat kernel = cv::Mat::zeros(kernelSize, kernelSize, CV_8U);  // 初始化为全零矩阵
    int center = kernelSize / 2;  // 计算中心位置

    // 设置中心行和中心列为1，形成十字型
    for (int i = 0; i < kernelSize; ++i) {
        kernel.at<uchar>(center, i) = 1;  // 中心行
        kernel.at<uchar>(i, center) = 1;  // 中心列
    }

    // 先进行腐蚀操作
    cv::Mat eroded;
    cv::erode(input, eroded, kernel);

    // 然后进行膨胀操作
    cv::Mat dilated;
    cv::dilate(eroded, dilated, kernel);

    return dilated;
}


/// <summary>
/// 
/// </summary>
/// <param name="img_paths">图片</param>
/// <param name="slants">倾斜角，侧视图中光源和相机的夹角，顺序与图片路径对应</param>
/// <param name="tilts">俯仰角，俯视图中光源和x轴的夹角，顺序与图片路径对应</param>
/// <returns></returns>
__declspec(dllexport) cv::Mat photometricStero(const vector<cv::Mat>& imgs, const vector<float>& slants, const vector<float>& tilts)
{
    PhotometricStero ps(imgs, slants, tilts);
    cv::Mat albedo = ps.getAlbedo();
    cv::Mat normal = ps.getNormal();
    normal.convertTo(normal, CV_32FC3, 255.0);
    auto mask = createMaskForGBSumGreaterThanN(normal, 60);
    mask = erodeAndDilate(mask, 10);
    albedo.convertTo(albedo, CV_8U, 255.0);
    albedo = albedo + mask;
    albedo.convertTo(albedo, CV_32F);
    albedo = albedo / 255.0;
    return albedo;
    //return ps.getAlbedo();
    //return ps.getNormal();
    //return ps.getNormalGray();
    //return ps.globalHeights();
}

__declspec(dllexport) cv::Mat photometricSteroNormal(const vector<cv::Mat>& imgs, const vector<float>& slants, const vector<float>& tilts)
{
    PhotometricStero ps(imgs, slants, tilts);
    return ps.getNormal();
    //return ps.getNormalGray();
    //return ps.globalHeights();
}

__declspec(dllexport) cv::Mat photometricSteroGray(const vector<cv::Mat>& imgs, const vector<float>& slants, const vector<float>& tilts)
{
    PhotometricStero ps(imgs, slants, tilts);
    return ps.getNormalGray();
    //return ps.globalHeights();
}

__declspec(dllexport) cv::Mat photometricSteroHeights(const vector<cv::Mat>& imgs, const vector<float>& slants, const vector<float>& tilts)
{
    PhotometricStero ps(imgs, slants, tilts);
    return ps.globalHeights();
}

