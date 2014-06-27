/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "NormalEstimation.hpp"
#include "Common/Logger.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include "pcl/search/kdtree.h"
#include "pcl/search/impl/kdtree.hpp"
#include "pcl/filters/impl/filter.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Processors {
namespace NormalEstimation {

NormalEstimation::NormalEstimation(const std::string & name) :
		Base::Component(name),
		radius_search("radius",0.05){
			registerProperty(radius_search);

}

NormalEstimation::~NormalEstimation() {
}

void NormalEstimation::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);

	// Register handlers
	h_compute.setup(boost::bind(&NormalEstimation::compute, this));
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud_xyzrgb);

}

bool NormalEstimation::onInit() {
	point_cloud_ptr =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	return true;
}

bool NormalEstimation::onFinish() {
	return true;
}

bool NormalEstimation::onStop() {
	return true;
}

bool NormalEstimation::onStart() {
	return true;
}

void NormalEstimation::compute() {

	std::vector<int> indices;

	point_cloud_ptr=in_cloud_xyzrgb.read();

	CLOG(LINFO) << "NormalEstimation->in_cloud_xyzrgb->size(): "<< point_cloud_ptr->size();

	// Remove NaNs.
	point_cloud_ptr->is_dense = false;
	pcl::removeNaNFromPointCloud(*point_cloud_ptr, *point_cloud_ptr, indices);

	CLOG(LINFO) << "NormalEstimation->in_cloud_xyzrgb->size(): "<< point_cloud_ptr->size();

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (point_cloud_ptr);


	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(radius_search);

	float x_;
	float y_;
	float z_;


	for(int i=0; i<point_cloud_ptr->size(); i++){
		x_+=point_cloud_ptr->points[i].x;
		y_+=point_cloud_ptr->points[i].y;
		z_+=point_cloud_ptr->points[i].z;
	}

	x_/=point_cloud_ptr->size();
	y_/=point_cloud_ptr->size();
	z_/=point_cloud_ptr->size();

	//ne.setViewPoint(x_, y_, z_);

	ne.setViewPoint(point_cloud_ptr->points[1].x,point_cloud_ptr->points[1].y,point_cloud_ptr->points[1].z);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	ne.compute(*cloud_normals);

	pcl::concatenateFields(*point_cloud_ptr, *cloud_normals, *cloud_out);

	out_cloud_xyzrgb_normals.write(cloud_out);

}


} //: namespace NormalEstimation
} //: namespace Processors
