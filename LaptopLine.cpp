#include "LaptopLine.h"

#define cp(i) cloud->points[i]
#define difY(i,j) abs(cp(i).y-cp(j).y)

LaptopLine::LaptopLine(CP cloud)
{
	origin.reset(new CloudT);
	preprocessed_cloud.reset(new CloudT);
	using_line.reset(new CloudT);
	*origin = *cloud;
	*preprocessed_cloud = *cloud;
	*using_line = *cloud;
	preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
	preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
	preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
	preProcess.push_back(std::bind(&LaptopLine::filterSingleJump, this, placeholders::_1));

	getAnchorModel = std::bind(&LaptopLine::getAnchorModelUpInside, this, placeholders::_1, placeholders::_2);
	//getAnchorScan = std::bind(&LaptopLine::getAnchorScanUpInside, this, placeholders::_1, placeholders::_2);
	getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownInsideThreshold, this, placeholders::_1, placeholders::_2, 1);

	//deformationEstimation = std::bind(&LaptopLine::deformationArea, this);
	deformationEstimation = &LaptopLine::deformationArea;

	//getModelBottom = std::bind(&LaptopLine::getModelBottomDefault, this, placeholders::_1);
	getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
}

LaptopLine::LaptopLine(CP cloud, int type)
{
	origin.reset(new CloudT);
	preprocessed_cloud.reset(new CloudT);
	using_line.reset(new CloudT);
	*origin = *cloud;
	*preprocessed_cloud = *cloud;
	*using_line = *cloud;
	switch (type)
	{
		case LaptopType::????????????????://????????????????
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::filterDis, this, placeholders::_1, 1, 1));
			preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::filterSingleJump, this, placeholders::_1));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownInsideThreshold, this, placeholders::_1, placeholders::_2, 0.0001);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanUpOutsideThreshold, this, placeholders::_1, placeholders::_2, 1);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			break;
		}

		case LaptopType::??????????://??????????
		{
			preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::filterSingleJump, this, placeholders::_1));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownInsideThreshold, this, placeholders::_1, placeholders::_2, 0.0001);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownInsideThreshold, this, placeholders::_1, placeholders::_2, 1);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			break;
		}

		case LaptopType::????://????
		{
			preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::filterSingleJump, this, placeholders::_1));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelFlatThreshold, this, placeholders::_1, placeholders::_2, 0.0001);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 0.1);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			break;
		}

		case LaptopType::????????://????????
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::filterSingleJump, this, placeholders::_1));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelFlatThreshold, this, placeholders::_1, placeholders::_2, 0.0001);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 1);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			break;
		}

		case LaptopType::????????://????????
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::filterSingleJump, this, placeholders::_1));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			break;
		}

		case LaptopType::????????_????????://????????-????????
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::filterSingleJumpV2, this, placeholders::_1, 0.1, 3));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			break;
		}

		case LaptopType::????????_????????://????????-????????
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmoothV2, this, placeholders::_1, 0.05));
			//preProcess.push_back(std::bind(&LaptopLine::filterDis, this, placeholders::_1, 0.1, 3));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.1);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			calculateDeformation = std::bind(&LaptopLine::straightness, this);
			break;
		}

		case LaptopType::NB6767D_GYG9D:
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmoothV2, this, placeholders::_1, 0.05));
			preProcess.push_back(std::bind(&LaptopLine::filterDis, this, placeholders::_1, 0.1, 3));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 0.3);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomFlatnessAndLength, this, placeholders::_1, 0.1, 10);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			calculateDeformation = std::bind(&LaptopLine::straightness, this);
			break;
		}

		case LaptopType::NB6837A:
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmoothV2, this, placeholders::_1, 0.05));
			preProcess.push_back(std::bind(&LaptopLine::filterDis, this, placeholders::_1, 0.1, 3));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownOutsideThreshold, this, placeholders::_1, placeholders::_2, 2);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownInsideThreshold, this, placeholders::_1, placeholders::_2, 1);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomFlatnessAndLength, this, placeholders::_1, 0.1, 10);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			calculateDeformation = std::bind(&LaptopLine::straightness, this);
			break;
		}

		case LaptopType::IDC40:
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmoothV2, this, placeholders::_1, 0.05));
			preProcess.push_back(std::bind(&LaptopLine::filterDis, this, placeholders::_1, 0.1, 20));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownInsideThreshold, this, placeholders::_1, placeholders::_2, 0.3);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownInsideThreshold, this, placeholders::_1, placeholders::_2, 0.3);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomFlatnessAndLength, this, placeholders::_1, 0.1, 10);
			//calculate2D = std::bind(&LaptopLine::buttom2D, this);
			calculateDeformation = std::bind(&LaptopLine::straightness, this);
			calculateInterval = std::bind(&LaptopLine::intervalDown, this, 0.3);
			break;
		};

		case LaptopType::IDC401X:
		{
			//preProcess.push_back(std::bind(&LaptopLine::filterInvalid, this, placeholders::_1));
			//preProcess.push_back(std::bind(&LaptopLine::InsideSmooth, this, placeholders::_1));
			preProcess.push_back(std::bind(&LaptopLine::meanSmoothV2, this, placeholders::_1, 0.05));
			preProcess.push_back(std::bind(&LaptopLine::filterDis, this, placeholders::_1, 0.1, 20));

			getAnchorModel = std::bind(&LaptopLine::getAnchorModelDownInsideThreshold, this, placeholders::_1, placeholders::_2, 0.3);
			getAnchorScan = std::bind(&LaptopLine::getAnchorScanDownInsideThreshold, this, placeholders::_1, placeholders::_2, 0.3);

			deformationEstimation = &LaptopLine::deformationArea;

			getModelBottom = std::bind(&LaptopLine::getBottomByNumber, this, placeholders::_1, 0.06);
			getScanBottom = std::bind(&LaptopLine::getBottomFlatnessAndLength, this, placeholders::_1, 0.1, 10);
			calculateDeformation = std::bind(&LaptopLine::straightness, this);
			calculateInterval = std::bind(&LaptopLine::intervalDownUP, this, 0.3);
			break;
		};

	default:
		throw logic_error("????????????????");
		break;
	}
}

void LaptopLine::straightness()
{
	// ??????????????
	Eigen::Vector3f centroid(0, 0, 0);
	centroid = using_line->getMatrixXfMap(3, 4, 0).rowwise().mean();


	// ??????????????
	Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();
	for (const auto& point : using_line->points)
	{
		Eigen::Vector3f centered_point = point.getVector3fMap() - centroid;
		covariance_matrix += centered_point * centered_point.transpose();
	}
	covariance_matrix /= using_line->points.size();

	// ????????????????????
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
	Eigen::Vector3f line_direction = eigen_solver.eigenvectors().col(2); 

	float max_dis=-999, min_dis=999;
	int arg_max, arg_min;
	float sum = 0;
	for (int i=0;i<using_line->size();i++)
	{
		auto point = using_line->points[i];
		Eigen::Vector3f vec = point.getVector3fMap() - centroid;
		Eigen::Vector3f projection = vec.dot(line_direction) * line_direction;
		float distance = (vec - projection).norm();
		Eigen::Vector3f cross_product = line_direction.cross(vec);
		//float sign = cross_product.z() >= 0 ? 1.0f : -1.0f;  // ????z????????????
		//distance = sign * distance;
		sum += distance;
		//if (distance > max_dis)
		//{
		//	max_dis = distance;
		//	arg_max = i;
		//}
		//if (distance < min_dis)
		//{
		//	min_dis = distance;
		//	arg_min = i;
		//}
	}
	deformation = sum / using_line->size();
	//deformation = 2*max(max_dis, - min_dis);
	#ifdef CZX_DEBUG
	//CP top(new CloudT);
	//top->push_back(using_line->points[arg_min]);
	//top->push_back(using_line->points[arg_max]);
	cout << "??????" << deformation << endl;
	//Tool::showComparison(using_line, top, 1, 5);
	#endif
}

CP LaptopLine::meanSmooth(CP cloud)
{
	#ifdef CZX_DEBUG
	#ifdef DEBUG_SMOOTH
	cout << "????????" << endl;
	CzxComparison _(cloud);
	#endif
	#endif
	for (int t = 0; t < smooth_times; t++)
	{
		{
			CircularBuffer buffer(smooth_range);
			buffer.threshold = smooth_threshold;
			for (int i = 0; i < cloud->size(); i++)
			{
				buffer.add(cp(i).y);
				cp(i).y = buffer.average();
			}
		}
		{
			CircularBuffer buffer(smooth_range);
			buffer.threshold = smooth_threshold;
			for (int i = cloud->size() - 1; i >= 0; i--)
			{
				buffer.add(cp(i).y);
				cp(i).y = buffer.average();
			}
		}
	}
	return cloud;
}

CP LaptopLine::meanSmoothV2(CP cloud, float threshold)
{
	#ifdef CZX_DEBUG
	#ifdef DEBUG_SMOOTH
	cout << "????????" << endl;
	CzxComparison _(cloud);
	#endif
	#endif
	for (int t = 0; t < smooth_times; t++)
	{
		{
			CircularBuffer buffer(smooth_range);
			buffer.threshold = threshold;
			for (int i = 0; i < cloud->size(); i++)
			{
				buffer.add(cp(i).y);
				cp(i).y = buffer.average();
			}
		}
		{
			CircularBuffer buffer(smooth_range);
			buffer.threshold = threshold;
			for (int i = cloud->size() - 1; i >= 0; i--)
			{
				buffer.add(cp(i).y);
				cp(i).y = buffer.average();
			}
		}
	}
	return cloud;
}

CP LaptopLine::InsideSmooth(CP cloud)
{
	#ifdef CZX_DEBUG
	#ifdef DEBUG_SMOOTH
	cout << "????????" << endl;
	CzxComparison _(cloud);
	#endif
	#endif

	int points = cloud->size()/2;
	for (int t = 0; t < smooth_times; t++)
	{
		{
			CircularBuffer buffer(smooth_range);
			buffer.threshold = smooth_threshold;
			for (int i = 0; i < points; i++)
			{
				buffer.add(cp(i).y);
				cp(i).y = buffer.average();
			}
		}
		{
			CircularBuffer buffer(smooth_range);
			buffer.threshold = smooth_threshold;
			for (int i = cloud->size() - 1; i >= cloud->size() - points - 1; i--)
			{
				buffer.add(cp(i).y);
				cp(i).y = buffer.average();
			}
		}
	}
	return cloud;
}

CP LaptopLine::filterSingleJump(CP cloud)
{
	float jmp_threshold = 0.04;
	CP ret(new CloudT);
	int range = 5;

	for (int i = 0; i < cloud->size(); i++)
	{
		if (rangeVerify(cloud, i, range, jmp_threshold))
		{
			ret->push_back(cp(i));
		}

	}
	#ifdef CZX_DEBUG
	#ifdef DEBUG_SMOOTH
	cout << "????????" << endl;
	Tool::showComparison(cloud, ret);
	#endif
	#endif
	return ret;
}

CP LaptopLine::filterSingleJumpV2(CP cloud, float jmp_threshold, int range)
{
	CP ret(new CloudT);

	for (int i = 0; i < cloud->size(); i++)
	{
		if (rangeVerify(cloud, i, range, jmp_threshold))
		{
			ret->push_back(cp(i));
		}

	}
	#ifdef CZX_DEBUG
	#ifdef DEBUG_SMOOTH
	cout << "????????" << endl;
	Tool::showComparison(cloud, ret);
	#endif
	#endif
	return ret;
}

CP LaptopLine::filterDis(CP cloud, float dis_threshold, int range)
{
	CP ret(new CloudT);

	for (int i = range; i < cloud->size()- range; i++)
	{
		if (abs(cp(i).y - cp(i+ range).y)< dis_threshold || abs(cp(i).y- cp(i - range).y) < dis_threshold)
		{
			ret->push_back(cp(i));
		}

	}
	#ifdef CZX_DEBUG
	#ifdef DEBUG_SMOOTH
	cout << "????????" << endl;
	Tool::showComparison(cloud, ret);
	#endif
	#endif
	return ret;
}

//????????????
CP LaptopLine::filterInvalid(CP cloud)
{
	//return cloud;
	CP ret(new CloudT);
	bool invalid = false;
	for (int i = 0; i < cloud->size(); i++)
	{
		while (i != cloud->size() - 1 && abs(cp(i).y - cp(i + 1).y) < 1e-8)
		{
			i++;
			invalid = true;
		}
		if (invalid)
		{
			i++;
			invalid = false;
		}
		while (i < cloud->size() - 1
			&& (abs(cp(i).y - cp(i + 1).y) > 0.03 || abs(cp(i).y - cp(i + 1).y) < 1e-8)
			&& abs(cp(i).y - cp(i - 1).y) > 0.03 || abs(cp(i).y - cp(i - 1).y) < 1e-8)
			i++;
		if (i != cloud->size())
			ret->push_back(cp(i));
	}
	return ret;
}

PointT LaptopLine::getAnchorModelUpInside(CP cloud, Eigen::Matrix3f& rotation)
{
	PointT left, right;
	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > 0.001)
		{
			left = cp(i);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (cp(i).y - cp(i - 1).y > 0.001)
		{
			right = cp(i);
			break;
		}
	}

	PointT anchor;
	anchor.x = (left.x + right.x) / 2;
	anchor.y = (left.y + right.y) / 2;
	anchor.z = 0;
	Vector3f x = right.getVector3fMap() - left.getVector3fMap();
	x.normalize();
	Vector3f z(0, 0, 1);
	Vector3f y = -x.cross(z);
	rotation << x, y, z;
	rotation = rotation.transpose();

	#ifdef CZX_DEBUG
	#ifdef DEBUG_MODEL
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);
	Tool::showComparison(cloud, tmp);
	#endif
	#endif
	return anchor;
}

PointT LaptopLine::getAnchorModelDownInsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float thredshold)
{
	PointT left, right;
	if (cloud->size() == 0)
		throw logic_error("????????????");
	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > thredshold)
		{
			left = cp(i);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (cp(i).y - cp(i - 1).y > thredshold)
		{
			right = cp(i);
			break;
		}
	}

	PointT anchor;
	anchor.x = (left.x + right.x) / 2;
	anchor.y = (left.y + right.y) / 2;
	anchor.z = 0;
	Vector3f x = right.getVector3fMap() - left.getVector3fMap();
	x.normalize();
	Vector3f z(0, 0, 1);
	Vector3f y = -x.cross(z);
	rotation << x, y, z;
	rotation = rotation.transpose();

	#ifdef CZX_DEBUG
	#ifdef DEBUG_MODEL
	cout << "????????????" << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);
	Tool::showComparison(cloud, tmp);
	#endif
	#endif
	return anchor;
}

/// <summary>
/// ??????????????????
/// </summary>
/// <param name="cloud"></param>
/// <param name="rotation"></param>
/// <param name="thredshold"></param>
/// <returns></returns>
PointT LaptopLine::getAnchorModelDownOutsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float thredshold)
{
	PointT left, right;
	if (cloud->size() == 0)
		throw logic_error("????????????");
	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > thredshold)
		{
			left = cp(i+1);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (cp(i).y - cp(i - 1).y > thredshold)
		{
			right = cp(i-1);
			break;
		}
	}

	PointT anchor;
	anchor.x = (left.x + right.x) / 2;
	anchor.y = (left.y + right.y) / 2;
	anchor.z = 0;
	Vector3f x = right.getVector3fMap() - left.getVector3fMap();
	x.normalize();
	Vector3f z(0, 0, 1);
	Vector3f y = -x.cross(z);
	rotation << x, y, z;
	rotation = rotation.transpose();

	#ifdef CZX_DEBUG
	#ifdef DEBUG_MODEL
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);
	Tool::showComparison(cloud, tmp);
	#endif
	#endif
	return anchor;
}

PointT LaptopLine::getAnchorModelFlatThreshold(CP cloud, Eigen::Matrix3f& rotation, float thredshold)
{
	PointT left, right;
	if (cloud->size() == 0)
		throw logic_error("????????????");
	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (abs(cp(i).y - cp(i + 1).y) < thredshold)
		{
			left = cp(i);
			break;
}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (abs(cp(i).y - cp(i - 1).y) < thredshold)
		{
			right = cp(i);
			break;
		}
	}

	PointT anchor;
	anchor.x = (left.x + right.x) / 2;
	anchor.y = (left.y + right.y) / 2;
	anchor.z = 0;
	Vector3f x = right.getVector3fMap() - left.getVector3fMap();
	x.normalize();
	Vector3f z(0, 0, 1);
	Vector3f y = -x.cross(z);
	rotation << x, y, z;
	rotation = rotation.transpose();

	#ifdef CZX_DEBUG
	#ifdef DEBUG_MODEL
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);
	Tool::showComparison(cloud, tmp);
	#endif
	#endif
	return anchor;
}

#ifdef DEBUG_TWO
CP compare_debug;
CP last_bound[2];
CP top_points[2];
#endif

PointT LaptopLine::getAnchorScanDownInsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold)
{
	PointT left, right;

	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > threshold)
		{
			left = cp(i);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (cp(i).y - cp(i - 1).y > threshold)
		{
			right = cp(i);
			break;
		}
	}

	dis2D = pcl::euclideanDistance(left, right);

	PointT anchor;
	{
		anchor.x = (left.x + right.x) / 2;
		anchor.y = (left.y + right.y) / 2;
		anchor.z = 0;
		Vector3f x = right.getVector3fMap() - left.getVector3fMap();
		x.normalize();
		Vector3f z(0, 0, 1);
		Vector3f y = x.cross(z);
		if (y[1] < 0)
			y = -y;
		rotation << x, y, z;
		rotation = rotation.transpose();
	}

	#ifdef CZX_DEBUG
	cout << "????????????" << endl;
	cout << dis2D << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);

	#ifdef DEBUG_TWO
	last_bound[0] = last_bound[1];
	last_bound[1] = tmp;
	#endif
	Tool::showComparison(cloud, tmp);
	#endif
	return anchor;
}

PointT LaptopLine::getAnchorScanDownOutsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold)
{
	PointT left, right;

	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > threshold)
		{
			left = cp(i+1);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (cp(i).y - cp(i - 1).y > threshold)
		{
			right = cp(i-1);
			break;
		}
	}

	dis2D = pcl::euclideanDistance(left, right);

	PointT anchor;
	{
		anchor.x = (left.x + right.x) / 2;
		anchor.y = (left.y + right.y) / 2;
		anchor.z = 0;
		Vector3f x = right.getVector3fMap() - left.getVector3fMap();
		x.normalize();
		Vector3f z(0, 0, 1);
		Vector3f y = x.cross(z);
		if (y[1] < 0)
			y = -y;
		rotation << x, y, z;
		rotation = rotation.transpose();
	}

	#ifdef CZX_DEBUG
	cout << "????????????" << endl;
	cout << dis2D << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);

	#ifdef DEBUG_TWO
	last_bound[0] = last_bound[1];
	last_bound[1] = tmp;
	#endif
	Tool::showComparison(cloud, tmp);
	#endif
	return anchor;
}

//??????????,????????????????
PointT LaptopLine::getAnchorScanUpOutsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold)
{
	PointT left, right;

	bool founded = false;
	for (int i = 0; i < cloud->size()/5; i++)
	{
		if (cp(i+1).y - cp(i ).y > threshold)
		{
			left = cp(i+1);
			founded = true;
			break;
		}
	}
	if (!founded)
		left = cp(0);
	founded = false;

	for (int i = cloud->size() - 1; i > cloud->size() * 0.8; i--)
	{
		if (cp(i-1).y - cp(i).y > threshold)
		{
			right = cp(i-1);
			founded = true;
			break;
		}
	}
	if (!founded)
		right = cp(cloud->size() - 1);

	dis2D = pcl::euclideanDistance(left, right);

	PointT anchor;
	{
		anchor.x = (left.x + right.x) / 2;
		anchor.y = (left.y + right.y) / 2;
		anchor.z = 0;
		Vector3f x = right.getVector3fMap() - left.getVector3fMap();
		x.normalize();
		Vector3f z(0, 0, 1);
		Vector3f y = x.cross(z);
		if (y[1] < 0)
			y = -y;
		rotation << x, y, z;
		rotation = rotation.transpose();
	}

	#ifdef CZX_DEBUG
	cout << "????????????" << endl;
	cout << dis2D << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);

	#ifdef DEBUG_TWO
	last_bound[0] = last_bound[1];
	last_bound[1] = tmp;
	#endif
	Tool::showComparison(cloud, tmp);
	#endif
	return anchor;
}

PointT LaptopLine::getAnchorScanSlopeThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold)
{
	PointT left, right;

	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if ((cp(i).y - cp(i + 1).y)/(cp(i+1).x - cp(i).x) > threshold)
		{
			left = cp(i + 1);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if ((cp(i).y - cp(i - 1).y)/(cp(i+1).x-cp(i).x) > threshold)
		{
			right = cp(i - 1);
			break;
		}
	}

	dis2D = pcl::euclideanDistance(left, right);

	PointT anchor;
	{
		anchor.x = (left.x + right.x) / 2;
		anchor.y = (left.y + right.y) / 2;
		anchor.z = 0;
		Vector3f x = right.getVector3fMap() - left.getVector3fMap();
		x.normalize();
		Vector3f z(0, 0, 1);
		Vector3f y = x.cross(z);
		if (y[1] < 0)
			y = -y;
		rotation << x, y, z;
		rotation = rotation.transpose();
	}

	#ifdef CZX_DEBUG
	cout << "????????????" << endl;
	cout << dis2D << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);

	#ifdef DEBUG_TWO
	last_bound[0] = last_bound[1];
	last_bound[1] = tmp;
	#endif
	Tool::showComparison(cloud, tmp);
	#endif
	return anchor;
}

PointT LaptopLine::getAnchorScanUpInside(CP cloud, Eigen::Matrix3f& rotation)
{
	PointT left, right;

	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > 2)
		{
			left = cp(i);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 10; i--)
	{
		if (cp(i).y - cp(i - 1).y > 2)
		{
			right = cp(i);
			break;
		}
	}

	dis2D = pcl::euclideanDistance(left, right);

	PointT anchor;
	{
		anchor.x = (left.x + right.x) / 2;
		anchor.y = (left.y + right.y) / 2;
		anchor.z = 0;
		Vector3f x = right.getVector3fMap() - left.getVector3fMap();
		x.normalize();
		Vector3f z(0, 0, 1);
		Vector3f y = x.cross(z);
		if (y[1] < 0)
			y = -y;
		rotation << x, y, z;
		rotation = rotation.transpose();
	}

	#ifdef CZX_DEBUG
	cout << "2D??????" << dis2D << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);

	#ifdef DEBUG_TWO
	last_bound[0] = last_bound[1];
	last_bound[1] = tmp;
	#endif
	Tool::showComparison(cloud, tmp);
	#endif
	return anchor;	
}

PointT LaptopLine::getAnchorScanOutside(CP cloud, Eigen::Matrix3f& rotation)
{
	PointT left, right;

	float threshold = 0.02;
	for (int i = 0; i < cloud->size() - 1000; i++)
	{
		if(difY(i,i+1)<threshold && difY(i+2, i + 1) < threshold && difY(i+2, i + 3) < threshold && difY(i + 4, i + 3) < threshold && difY(i + 4, i + 5) < threshold
			&& cp(i).y-cp(i+1000).y>0.1)
		{
			left = cp(i);
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 1000; i--)
	{
		if (difY(i, i - 1) < threshold && difY(i - 2, i - 1) < threshold && difY(i - 4, i - 3) < threshold && difY(i - 4, i - 5) < threshold
			&& difY(i - 2, i - 3) < threshold 
			&& cp(i).y - cp(i - 1000).y > 0.1)
		{
			right = cp(i);
			break;
		}
	}

	dis2D = pcl::euclideanDistance(left, right);

	PointT anchor;
	{
		anchor.x = (left.x + right.x) / 2;
		anchor.y = (left.y + right.y) / 2;
		anchor.z = 0;
		Vector3f x = right.getVector3fMap() - left.getVector3fMap();
		x.normalize();
		Vector3f z(0, 0, 1);
		Vector3f y = x.cross(z);
		if (y[1] < 0)
			y = -y;
		rotation << x, y, z;
		rotation = rotation.transpose();
	}

	#ifdef CZX_DEBUG
	cout << "2D??????" << dis2D << endl;
	CP tmp(new CloudT);
	tmp->push_back(left);
	tmp->push_back(right);

	#ifdef DEBUG_TWO
	last_bound[0] = last_bound[1];
	last_bound[1] = tmp;
	#endif
	Tool::showComparison(cloud, tmp);
	#endif
	return anchor;
}

void LaptopLine::intervalDown(float threshold)
{
	CP cloud = origin;
	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > threshold)
		{
			interval_left = cp(i + 1).x - cp(i).x;
			#ifdef CZX_DEBUG
			#ifdef DEBUG_INTERVAL
			CP tmp(new CloudT);
			tmp->push_back(cp(i));
			tmp->push_back(cp(i+1));
			cout << "????????????" << endl;
			cout << interval_left << endl;
			Tool::showComparison(origin, tmp, 1, 3);
			#endif
			#endif
			break;
		}
	}
	for (int i = cloud->size() - 1; i > 100; i--)
	{
		if (cp(i).y - cp(i - 1).y > threshold)
		{
			interval_right = cp(i).x - cp(i - 1).x;
			#ifdef CZX_DEBUG
			#ifdef DEBUG_INTERVAL
			CP tmp(new CloudT);
			tmp->push_back(cp(i));
			tmp->push_back(cp(i - 1));
			cout << "????????????" << endl;
			cout << interval_left << endl;
			Tool::showComparison(origin, tmp, 1, 3);
			#endif
			#endif
			return;
		}
	}
}

void LaptopLine::intervalDownUP(float threshold)
{
	CP cloud = preprocessed_cloud;
	PointT star, end;
	for (int i = 0; i < cloud->size() - 10; i++)
	{
		if (cp(i).y - cp(i + 1).y > threshold)
		{
			star = cp(i);
			for (; i < cloud->size() - 10; i++)
			{
				if (cp(i + 1).y - cp(i).y > threshold)
				{
					end = cp(i + 1);
					interval_left = abs(end.x - star.x);
					drop_left = abs(end.y - star.y);
					if (interval_left < 0.1) continue;
					#ifdef CZX_DEBUG
					#ifdef DEBUG_INTERVAL
					CP tmp(new CloudT);
					tmp->push_back(star);
					tmp->push_back(end);
					cout << "????????????" << endl;
					cout << interval_left << endl;
					Tool::showComparison(cloud, tmp, 1, 3);
					#endif
					#endif
					break;
				}
			}
			break;
		}
	}

	for (int i = cloud->size() - 1; i > 100; i--)
	{
		if (cp(i).y - cp(i - 1).y > threshold)
		{
			star = cp(i);
			for (; i > 100; i--)
			{
				if (cp(i - 1).y - cp(i).y > threshold)
				{
					end = cp(i - 1);
					interval_right = abs(star.x - end.x);
					drop_right = abs(end.y - star.y);
					if (interval_right < 0.1) continue;
					#ifdef CZX_DEBUG
					#ifdef DEBUG_INTERVAL
					CP tmp(new CloudT);
					tmp->push_back(star);
					tmp->push_back(end);
					cout << "????????????" << endl;
					cout << interval_right << endl;
					Tool::showComparison(cloud, tmp, 1, 3);
					#endif
					#endif
					break;
				}
			}
			break;
		}
	}

}

void LaptopLine::deformationArea()
{
	#ifdef CZX_DEBUG
	CP source_copy(new CloudT);
	*source_copy = *using_line;
	transformScan(source_copy);
	//Tool::showComparison(using_line, source_copy);
	#endif
	
	using_line = getScanBottom(using_line);

	normalized_line.reset(new CloudT);
	*normalized_line = *using_line;
	transformScan(normalized_line);

	#ifndef EMPTY_MODEL
	CP model_bottom = getModelBottom(model);
	if (model_bottom->size() < 100)		
		return;
	arsenal::passThrough(normalized_line, model_bottom->points[0].x + 1, model_bottom->points[model_bottom->size() - 1].x - 1);

	deformation = 0;
	deformation_d = 0;
	deformation_central_moments = 0;
	int order_deformation = 4;
	{
		std::function<int(int)> getIndex;
		if (normalized_line->points[0].x < normalized_line->points[1].x)
			getIndex = [](int i) {return i; };
		else
			getIndex = [this](int i) {return (int)normalized_line->size() - i - 1; };

		int index_model = 0;
		int count = 0;
		CP deform_points(new CloudT());
		deform_points->resize(2);
		for (int i = 1; i < normalized_line->size(); i++)
		{
			auto& p_source = normalized_line->points[getIndex(i)];
			while (model_bottom->points[index_model].x < p_source.x)
				index_model++;
			auto& p_model = model_bottom->points[index_model];
			if (p_model.x - p_source.x > 0.04)
			{
				continue;
			}
			float displacement = p_model.y - p_source.y;

			if (up_dif < displacement)
			{
				up_dif = displacement;
				deform_points->points[0] = p_source;
			}
			if (down_dif > displacement)
			{
				down_dif = displacement;
				deform_points->points[1] = p_source;
			}
			up_dif = max(up_dif, displacement);
			down_dif = min(down_dif, displacement);
			deformation += abs(displacement);
			deformation_d += displacement;
			deformation_central_moments += pow(displacement, order_deformation);
			count++;
		}
		deformation /= count;
		deformation_d /= count;
		deformation_central_moments /= count;
		deformation_central_moments = pow(deformation_central_moments, 1.0f / order_deformation);
		//#ifdef CZX_DEBUG
		//cout << "??????????: " << endl;
		//Tool::showComparison(model, normalized_line, deform_points,1,1,5);
		//#endif

		#ifdef DEBUG_TWO
		top_points[1] = top_points[0];
		top_points[0] = deform_points;
		#endif


	}
	#endif


	int order = 4;
	float mean_y = normalized_line->getMatrixXfMap(3, 4, 0).row(1).mean();
	double sum = 0;
	for (auto& p : *normalized_line)
	{
		sum += pow(p.y - mean_y, order);
	}
	float central_moments = sum / normalized_line->size();
	adjust_central_moments = pow(central_moments, 1.0 / order);

	//#ifdef CZX_DEBUG
	//cout<<"??????????????: " << endl;
	//Tool::showComparison(model, normalized_line);
	//#endif
	#ifdef CZX_DEBUG
	CP copy_ori(new CloudT);
	*copy_ori = *origin;
	transformScan(copy_ori);
	#ifndef EMPTY_MODEL
	cout << "??????????????: " << endl;
	Tool::showComparison(model, copy_ori);
	#endif
	cout << "??????????????: " << endl;
	Tool::showComparison(copy_ori, normalized_line);
	#endif

	#ifdef CZX_DEBUG
	{
		cout << "????" << up_dif << endl;
		cout << "????" << down_dif << endl;
		cout << "??????????" << deformation << endl;
		cout << "??????"<<adjust_central_moments << endl;

		#ifdef DEBUG_TWO
		transformScan(last_bound[1]);		
		if (compare_debug == nullptr)
		{
			compare_debug = source_copy;
		}
		else
		{
			CP bound(new CloudT);
			*bound += *last_bound[0];
			*bound += *last_bound[1];
			//Tool::showComparison(source_copy, compare_debug, bound,1,1,3);
			cout << "??????????????: " << endl;
			Tool::showComparison(model, source_copy, compare_debug);
		}
		#endif
	}
	#endif
}

void LaptopLine::buttom2D()
{
	CP bottom = getBottomByNumber(origin, 0.2);
	if (bottom->size() == 0)
		throw logic_error("????????????");
	dis2D = pcl::euclideanDistance(bottom->points[0], bottom->points[bottom->size()-1]);
	#ifdef CZX_DEBUG
	cout << "????????????" << endl;
	CP tmp(new CloudT);
	tmp->push_back(bottom->points[0]);
	tmp->push_back(bottom->points[bottom->size() - 1]);
	Tool::showComparison(origin, tmp);
	#endif
}

CP LaptopLine::getBottomByNumber(CP cloud, float threshold)
{
	vector<CP> candidates;
	bool inserted;
	for (int i = 0; i < cloud->size(); i++)
	{
		auto& p = cp(i);
		inserted = false;
		float min_dis = 99999;
		CP min_cloud;

		if (candidates.size() > 0)
		{
			auto& cl = candidates[candidates.size() - 1];
			float dif = abs(cl->points[cl->size() - 1].y - p.y);
			float dif_x = abs(cl->points[cl->size() - 1].x - p.x);
			if (dif_x < 0.002)
				continue;
			if (dif < threshold)
			{
				cl->push_back(p);
				inserted = true;
			}
			else if (i < cloud->size() - 10 && cl->size()>10) //????????????????????????????????????
			{
				dif = abs(cl->points[cl->size() - 2].y - cp(i + 1).y);
				if (dif < threshold) //??????????????????,??????????????????
				{
					cl->points.pop_back();
					i++;
					cl->push_back(cp(i));
					inserted = true;
				}
			}
		}
		if (!inserted)
		{
			CP new_cloud(new CloudT);
			new_cloud->push_back(p);
			candidates.push_back(new_cloud);
		}
	}

	CP ret(new CloudT);
	for (auto& cl : candidates)
	{
		if (abs(cl->points[0].x - cl->points[cl->size() - 1].x) > 35)
			*ret += *cl;
	}
	#ifdef CZX_DEBUG
	#ifdef DEBUG_BOTTOM
	cout << "????????????" << endl;
	Tool::showComparison(cloud, ret);
	#endif
	#endif
	return ret;
}

CP LaptopLine::getBottomFlatnessAndLength(CP cloud, float threshold, float valid_length)
{
	vector<CP> candidates;
	bool inserted;
	for (int i = 0; i < cloud->size(); i++)
	{
		auto& p = cp(i);
		inserted = false;
		float min_dis = 99999;
		CP min_cloud;

		if (candidates.size() > 0)
		{
			auto& cl = candidates[candidates.size() - 1];
			float dif = abs(cl->points[cl->size() - 1].y - p.y);
			float dif_x = abs(cl->points[cl->size() - 1].x - p.x);
			if (dif_x < 0.002)
				continue;
			if (dif < threshold)
			{
				cl->push_back(p);
				inserted = true; 
			}
			else if (i < cloud->size() - 10 && cl->size()>10) //????????????????????????????????????
			{
				dif = abs(cl->points[cl->size() - 2].y - cp(i + 1).y);
				if (dif < threshold) //??????????????????,??????????????????
				{
					cl->points.pop_back();
					i++;
					cl->push_back(cp(i));
					inserted = true;
				}
			}
		}
		if (!inserted)
		{
			CP new_cloud(new CloudT);
			new_cloud->push_back(p);
			candidates.push_back(new_cloud);
		}
	}

	CP ret(new CloudT);
	for (auto& cl : candidates)
	{
		if (abs(cl->points[0].x - cl->points[cl->size() - 1].x) > valid_length)
			*ret += *cl;
	}
	#ifdef CZX_DEBUG
	#ifdef DEBUG_BOTTOM
	cout << "????????????" << endl;
	Tool::showComparison(cloud, ret);
	#endif
	#endif
	return ret;
}

CP LaptopLine::getModelBottomDefault(CP cloud)
{
	vector<CP> candidates;
	bool inserted;
	float threshold = 0.01;
	for (int i = 0; i < cloud->size(); i++)
	{
		auto& p = cp(i);
		inserted = false;
		float min_dis = 99999;
		CP min_cloud;

		if (candidates.size() > 0)
		{
			auto& cl = candidates[candidates.size() - 1];
			float dif = abs(cl->points[cl->size() - 1].y - p.y);

			if (dif < threshold)
			{
				cl->push_back(p);
				inserted = true;
			}

		}
		if (!inserted)
		{
			CP new_cloud(new CloudT);
			new_cloud->push_back(p);
			candidates.push_back(new_cloud);
		}
	}

	CP ret(new CloudT);
	for (auto& cl : candidates)
	{
		//#ifdef CZX_DEBUG
		//Tool::showComparison(cloud, cl);
		//#endif // CZX_DEBUG

		if (cl->size() > ret->size())
			ret = cl;
	}
	//#ifdef CZX_DEBUG
	//Tool::showComparison(cloud, ret);
	//#endif
	return ret;
}

void LaptopLine::initModel(CP model_cloud)
{
	Matrix3f rot_model;
	PointT anchor_model = getAnchorModel(model_cloud, rot_model);
	model_cloud->getMatrixXfMap(3, 4, 0).colwise() -= anchor_model.getVector3fMap();
	model_cloud->getMatrixXfMap(3, 4, 0) = rot_model * model_cloud->getMatrixXfMap(3, 4, 0);
	//model_cloud->getMatrixXfMap(3, 4, 0).row(0) *= -1;
	//sort(model_cloud->begin(), model_cloud->end(), [](PointT& a, PointT& b) {return a.x < b.x; });
}

void LaptopLine::initScanParamter(CP scan_cloud)
{
	Matrix3f rot_scan;
	PointT anchor_scan = getAnchorScan(scan_cloud, rot_scan);
	scan_rotation = rot_scan;
	scan_anchor = anchor_scan;
}

void LaptopLine::transformScan(CP scan_cloud)
{
	scan_cloud->getMatrixXfMap(3, 4, 0).colwise() -= scan_anchor.getVector3fMap();
	scan_cloud->getMatrixXfMap(3, 4, 0) = scan_rotation * scan_cloud->getMatrixXfMap(3, 4, 0);
}

void LaptopLine::mainProcess()
{
	if(preprocessed_cloud&&preprocessed_cloud->size()>1000)
	{
		#ifdef DEBUG_PREPROCESS
		CP before(new CloudT);
		*before = *using_line;
		#endif
		for (auto& pro : preProcess)
			preprocessed_cloud = pro(preprocessed_cloud);
		*using_line = *preprocessed_cloud;
		#ifdef DEBUG_PREPROCESS
		cout << "??????????????" << endl;
		Tool::showComparison(before, using_line);
		#endif
	}
	else
	{
		cout << "??????????????????????????????" << endl;
	}
	if (using_line->size() < 1000) {
		cout << "??????????????????????????????" << endl;
		return;
	}
	initScanParamter(using_line);
	if (model && model->size() > 1000)
	{
		cout << "??????????????????????" << endl;
		(this->*deformationEstimation)();
	}
	if (calculate2D != nullptr)
		calculate2D();
	if (calculateDeformation != nullptr)
		calculateDeformation();
	if (calculateInterval != nullptr)
		calculateInterval();
}

vector<float> LaptopLine::getOutput()
{
	vector<float> ret;
	
	const float up_deform = 1;
	const float down_deform = -1;
	const float wave_deform = 0;
	const float fit_deform = 666;

	//ret.push_back(up_dif);
	//ret.push_back(down_dif);
	//ret.push_back(up_dif - down_dif);
	ret.push_back(deformation);
	//ret.push_back(deformation_central_moments);
	ret.push_back(adjust_central_moments);
	ret.push_back(dis2D);
	ret.push_back(interval_left);
	ret.push_back(interval_right);
	ret.push_back(drop_left);
	ret.push_back(drop_right);


	//if (deformation < 0.1)
	//	ret.push_back(fit_deform);
	//else if (deformation_d > 0.1)
	//	ret.push_back(up_deform);
	//else if (deformation_d < -0.1)
	//	ret.push_back(down_deform);
	//else
	//	ret.push_back(wave_deform);

	score = -deformation;
	return ret;
}

void LaptopLine::combine(const LaptopLine& other)
{
	//??????????
	if (deformation > other.deformation)
	{
		deformation = other.deformation;
		adjust_central_moments = other.adjust_central_moments;
	}
	if (dis2D > other.dis2D)
	{
		dis2D = other.adjust_central_moments;
	}
}
