// dllmain.cpp : 定义 DLL 应用程序的入口点。


#include "pch.h"
#include"nanoflann.hpp"

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


__declspec(dllexport) vector<float> computeDeformationNake(CP model, CP cloud)
{
    LaptopLine lapline(cloud);
    lapline.model = model;
    lapline.mainProcess();
    auto ret = lapline.getOutput();
    return ret;
}
