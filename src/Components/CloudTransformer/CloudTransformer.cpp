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
    h_transform_clouds.setup(boost::bind(&CloudTransformer::transform_clouds, this));
    registerHandler("transform_clouds", &h_transform_clouds);
    addDependency("transform_clouds", &in_hm);

/*	h_transform_xyzrgb.setup(boost::bind(&CloudTransformer::transform_xyzrgb, this));
	registerHandler("transform_xyzrgb", &h_transform_xyzrgb);
	addDependency("transform_xyzrgb", &in_cloud_xyzrgb);
	h_transform_xyzsift.setup(boost::bind(&CloudTransformer::transform_xyzsift, this));
	registerHandler("transform_xyzsift", &h_transform_xyzsift);
    addDependency("transform_xyzsift", &in_cloud_xyzsift);*/

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

void CloudTransformer::transform_clouds() {
    CLOG(LTRACE) << "CloudTransformer::transform_clouds()";

    // Read hmomogenous matrix.
    Types::HomogMatrix hm = in_hm.read();

    // Try to transform XYZ.
    if(!in_cloud_xyz.empty())
        transform_xyz(hm);

    // Try to transform XYZRGB.
    if(!in_cloud_xyzrgb.empty())
        transform_xyzrgb(hm);

    // Try to transform XYZSIFT.
    if(!in_cloud_xyzsift.empty())
        transform_xyzsift(hm);

    // Try to transform SOM.
    if(!in_som.empty())
        transform_som(hm);

}


void CloudTransformer::transform_xyz(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyz()";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyz.write(cloud2);
}

void CloudTransformer::transform_xyzrgb(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzrgb()";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzrgb.write(cloud2);
}

void CloudTransformer::transform_xyzsift(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_xyzsift()";
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud2(new pcl::PointCloud<PointXYZSIFT>());
    pcl::transformPointCloud(*cloud, *cloud2, trans) ;
    out_cloud_xyzsift.write(cloud2);
}

void CloudTransformer::transform_som(Types::HomogMatrix hm_) {
    CLOG(LTRACE) << "CloudTransformer::transform_som()";
    SIFTObjectModel* som = in_som.read();
    SIFTObjectModel* som2 = new SIFTObjectModel();
    Eigen::Matrix4f trans = hm_.getElements();
    pcl::transformPointCloud(*(som->cloud_xyzrgb), *(som2->cloud_xyzrgb), trans) ;
    pcl::transformPointCloud(*(som->cloud_xyzsift), *(som2->cloud_xyzsift), trans) ;
    out_som.write(som2);
}



} //: namespace CloudTransformer
} //: namespace Processors
