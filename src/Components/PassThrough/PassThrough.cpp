/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PassThrough.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PassThrough {

PassThrough::PassThrough(const std::string & name) :
		Base::Component(name) , 
		xa("x.a", 0), 
		xb("x.b", 0), 
		ya("y.a", 0), 
		yb("y.b", 0), 
		za("z.a", 0), 
		zb("z.b", 0), 
        negative_x("negative_x", false),
        negative_y("negative_y", false),
        negative_z("negative_z", false){
		registerProperty(xa);
		registerProperty(xb);
		registerProperty(ya);
		registerProperty(yb);
		registerProperty(za);
		registerProperty(zb);
        registerProperty(negative_x);
        registerProperty(negative_y);
        registerProperty(negative_z);


}

PassThrough::~PassThrough() {
}

void PassThrough::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_cloud_xyz", &in_cloud_xyz);
    registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
    registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
    registerStream("in_som", &in_som);
    registerStream("out_cloud_xyz", &out_cloud_xyz);
    registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
    registerStream("out_som", &out_som);
	// Register handlers
    h_filter_xyz.setup(boost::bind(&PassThrough::filter_xyz, this));
    registerHandler("filter_xyz", &h_filter_xyz);
    addDependency("filter_xyz", &in_cloud_xyz);
    h_filter_xyzrgb.setup(boost::bind(&PassThrough::filter_xyzrgb, this));
    registerHandler("filter_xyzrgb", &h_filter_xyzrgb);
    addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
    h_filter_xyzsift.setup(boost::bind(&PassThrough::filter_xyzsift, this));
    registerHandler("filter_xyzsift", &h_filter_xyzsift);
    addDependency("filter_xyzsift", &in_cloud_xyzsift);
    h_filter_som.setup(boost::bind(&PassThrough::filter_som, this));
    registerHandler("filter_som", &h_filter_som);
    addDependency("filter_som", &in_som);

}

bool PassThrough::onInit() {

	return true;
}

bool PassThrough::onFinish() {
	return true;
}

bool PassThrough::onStop() {
	return true;
}

bool PassThrough::onStart() {
	return true;
}

void PassThrough::filter_xyz() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (xa, xb);
    pass.setFilterLimitsNegative (negative_x);
	pass.filter (*cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (ya, yb);
    pass.setFilterLimitsNegative (negative_y);
	pass.filter (*cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (za, zb);
    pass.setFilterLimitsNegative (negative_z);
	pass.filter (*cloud);
    out_cloud_xyz.write(cloud);
}

void PassThrough::filter_xyzrgb() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xa, xb);
    pass.setFilterLimitsNegative (negative_x);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ya, yb);
    pass.setFilterLimitsNegative (negative_y);
    pass.filter (*cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (za, zb);
    pass.setFilterLimitsNegative (negative_z);
    pass.filter (*cloud);
    out_cloud_xyzrgb.write(cloud);
}

void PassThrough::filter_xyzsift() {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    pcl::PassThrough<PointXYZSIFT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xa, xb);
    pass.setFilterLimitsNegative (negative_x);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ya, yb);
    pass.setFilterLimitsNegative (negative_y);
    pass.filter (*cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (za, zb);
    pass.setFilterLimitsNegative (negative_z);
    pass.filter (*cloud);
    out_cloud_xyzsift.write(cloud);
}

void PassThrough::filter_som() {
    SIFTObjectModel* som = in_som.read();
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::PassThrough<PointXYZSIFT> pass_sift;

    pass.setInputCloud (som->cloud_xyzrgb);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xa, xb);
    pass.setFilterLimitsNegative (negative_x);
    pass.filter (*(som->cloud_xyzrgb));
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ya, yb);
    pass.setFilterLimitsNegative (negative_y);
    pass.filter (*(som->cloud_xyzrgb));
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (za, zb);
    pass.setFilterLimitsNegative (negative_z);
    pass.filter (*(som->cloud_xyzrgb));

    pass_sift.setInputCloud (som->cloud_xyzsift);
    pass_sift.setFilterFieldName ("x");
    pass_sift.setFilterLimits (xa, xb);
    pass_sift.setFilterLimitsNegative (negative_x);
    pass_sift.filter (*(som->cloud_xyzsift));
    pass_sift.setFilterFieldName ("y");
    pass_sift.setFilterLimits (ya, yb);
    pass_sift.setFilterLimitsNegative (negative_y);
    pass_sift.filter (*(som->cloud_xyzsift));
    pass_sift.setFilterFieldName ("z");
    pass_sift.setFilterLimits (za, zb);
    pass_sift.setFilterLimitsNegative (negative_z);
    pass_sift.filter (*(som->cloud_xyzsift));

    out_som.write(som);
}

} //: namespace PassThrough
} //: namespace Processors
