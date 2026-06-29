#pragma once

#include"czxDependence/czxTool.h"
#include<pcl/kdtree/kdtree_flann.h>
#include"CzxRansac.h"

class Hole
{
public:
	CONFTYPE config;

	Hole()
	{
		config = czxTool::readProfile("conf_hole.czx");
	};

	CP KNNBoundExtract(CP cloud);

	PointT centerFit(CP cloud);


	//template <typename CLOUD> void fun(CLOUD cloud);

	//template <typename CLOUD> PointT process(CLOUD cloud);
};

