#pragma once
#include"../czxDependence/czxTool.h"
#include<pcl/common/common.h>
#include<pcl/common/distances.h>
#include"utlis.h"


//#define EMPTY_MODEL
//#define CZX_DEBUG
//#define DEBUG_TWO
////
//#define DEBUG_PREPROCESS
//#define DEBUG_SMOOTH
//#define DEBUG_MODEL//查看模型点是否寻找正确
//#define DEBUG_BOTTOM
//#define DEBUG_INTERVAL
//#define UN_MODEL //根据配置文件model_path，随便找个模型凑合着用


class LaptopLine
{
public:
	enum LaptopType {
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

	typedef function<PointT(CP, Matrix3f&)> PCM;
	typedef void(LaptopLine::*DEFORMTYPE)();
	typedef function<CP(CP)> CC;
	typedef function<CP(CP, float)> CCF;
	typedef function<void()> VV;
	//typedef function<vector<float>> OUTFUNC;

public:
	LaptopLine(CP cloud);
	LaptopLine(CP cloud, int type);

	void straightness();
	PCM getAnchorModel;
	PCM getAnchorScan;
	DEFORMTYPE deformationEstimation;
	vector<CC> preProcess;
	CC getModelBottom;
	CC getScanBottom;
	VV calculate2D = nullptr;
	VV calculateDeformation = nullptr;
	VV calculateInterval = nullptr;

	CP filterInvalid(CP cloud);//滤除直线
	CP InsideSmooth(CP cloud);//内向滤波
	CP meanSmooth(CP cloud); //均值滤波
	CP meanSmoothV2(CP cloud, float threshold);
	CP filterSingleJump(CP cloud);
	CP filterSingleJumpV2(CP cloud, float jmp_threshold, int range);
	CP filterDis(CP cloud, float dis_threshold, int range);//滤掉两边高度差异常点
	//删除跳变点
	PointT getAnchorModelUpInside(CP cloud, Eigen::Matrix3f& rotation);
	PointT getAnchorModelDownInsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float thredshold);
	PointT getAnchorModelDownOutsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float thredshold);
	//根据跳变点，获取顶部锚点
	PointT getAnchorModelFlatThreshold(CP cloud, Eigen::Matrix3f& rotation, float thredshold);//寻找平滑的外侧点
	PointT getAnchorScanDownInsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold=2);//下跳变起点
	PointT getAnchorScanDownOutsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold);
	PointT getAnchorScanUpOutsideThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold);
	//寻找跳变底部点
	PointT getAnchorScanSlopeThreshold(CP cloud, Eigen::Matrix3f& rotation, float threshold); //寻找斜率大的拐点
	//根据跳变点，获取顶部锚点
	/// <summary>
	/// 获得第一次跳变点
	/// </summary>
	PointT getAnchorScanUpInside(CP cloud, Eigen::Matrix3f& rotation);
	PointT getAnchorScanOutside(CP cloud, Eigen::Matrix3f& rotation);

	//一次左 下跳变的间隙
	void intervalDown(float threshold);
	//一次左 下跳变加上跳变的间隙
	void intervalDownUP(float threshold);

	void deformationArea();	
	void buttom2D();

	CP getBottomByNumber(CP cloud, float threshold = 0.06);
	CP getModelBottomDefault(CP cloud);
	CP getBottomFlatnessAndLength(CP cloud, float threshold, float valid_length);//无大跳变的组合

	void initModel(CP model_cloud);
	void initScanParamter(CP scan_cloud);
	void transformScan(CP scan_cloud);

	void mainProcess();

	vector<float> getOutput();

	void combine(const LaptopLine& other);

public:
	//输出参数
	float dis2D=-1;
	float deformation=-1;
	float deformation_d=-1;
	float deformation_central_moments =-1;
	float adjust_central_moments=-1;
	float up_dif=-999;
	float down_dif=999;

	float interval_left = -1;
	float interval_right = -1;
	float drop_left = -1;
	float drop_right = -1;

	float score;
	CP model;

	CONFTYPE conf = czxTool::readProfile("deform_conf.czx");
	int smooth_range = stoi(conf["smooth_range"]);
	int smooth_times = stoi(conf["smooth_times"]);
	float smooth_threshold = stof(conf["smooth_threshold"]);
private:
	CP origin;
	CP preprocessed_cloud;
	CP using_line;
	CP normalized_line;

	Eigen::Matrix3f scan_rotation;
	PointT scan_anchor;
};


