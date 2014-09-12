/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CloudTransformer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CloudTransformer {

CloudTransformer::CloudTransformer(const std::string & name) :
		Base::Component(name)  {

}

CloudTransformer::~CloudTransformer() {
}

void CloudTransformer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_som", &in_som);
    registerStream("in_hm", &in_hm);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_som", &out_som);
	// Register handlers
	h_fransform_xyz.setup(boost::bind(&CloudTransformer::fransform_xyz, this));
	registerHandler("fransform_xyz", &h_fransform_xyz);
	addDependency("fransform_xyz", &in_cloud_xyz);
	h_transform_xyzrgb.setup(boost::bind(&CloudTransformer::transform_xyzrgb, this));
	registerHandler("transform_xyzrgb", &h_transform_xyzrgb);
	addDependency("transform_xyzrgb", &in_cloud_xyzrgb);
	h_transform_xyzsift.setup(boost::bind(&CloudTransformer::transform_xyzsift, this));
	registerHandler("transform_xyzsift", &h_transform_xyzsift);
	addDependency("transform_xyzsift", &in_cloud_xyzsift);

}

bool CloudTransformer::onInit() {

	return true;
}

bool CloudTransformer::onFinish() {
	return true;
}

bool CloudTransformer::onStop() {
	return true;
}

bool CloudTransformer::onStart() {
	return true;
}

void CloudTransformer::fransform_xyz() {
    LOG(LTRACE) << "CloudTransformer::fransform_xyz()";
    if(in_hm.empty()){
        LOG(LWARNING) << "No data in in_hm datastream";
        return;
    }
    Types::HomogMatrix hm = in_hm.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    Eigen::Matrix4f trans = hm.getElements();
    pcl::transformPointCloud(*cloud, *cloud, trans) ;
    out_cloud_xyz.write(cloud);
}

void CloudTransformer::transform_xyzrgb() {
    LOG(LTRACE) << "CloudTransformer::fransform_xyzrgb()";
    if(in_hm.empty()){
        LOG(LWARNING) << "No data in in_hm datastream";
        return;
    }
    Types::HomogMatrix hm = in_hm.read();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    Eigen::Matrix4f trans = hm.getElements();
    pcl::transformPointCloud(*cloud, *cloud, trans) ;
    out_cloud_xyzrgb.write(cloud);
}

void CloudTransformer::transform_xyzsift() {
    LOG(LTRACE) << "CloudTransformer::fransform_xyz()";
    if(in_hm.empty()){
        LOG(LWARNING) << "No data in in_hm datastream";
        return;
    }
    Types::HomogMatrix hm = in_hm.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    Eigen::Matrix4f trans = hm.getElements();
    pcl::transformPointCloud(*cloud, *cloud, trans) ;
    out_cloud_xyzsift.write(cloud);
}

void CloudTransformer::transform_som() {
    LOG(LTRACE) << "CloudTransformer::fransform_som()";
    if(in_hm.empty()){
        LOG(LWARNING) << "No data in in_hm datastream";
        return;
    }
    SIFTObjectModel* som = in_som.read();
    Types::HomogMatrix hm = in_hm.read();
    Eigen::Matrix4f trans = hm.getElements();
    pcl::transformPointCloud(*(som->cloud_xyzrgb), *(som->cloud_xyzrgb), trans) ;
    pcl::transformPointCloud(*(som->cloud_xyzsift), *(som->cloud_xyzsift), trans) ;
    out_som.write(som);
}



} //: namespace CloudTransformer
} //: namespace Processors
