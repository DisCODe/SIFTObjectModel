/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "CloudViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/filter.h>

namespace Processors {
namespace CloudViewer {

CloudViewer::CloudViewer(const std::string & name) :
		Base::Component(name),
    prop_window_name("window_name", std::string("3D PC Viewer")),
    prop_coordinate_system("coordinate_system", true),
    prop_two_viewports("two_viewports", false),
    prop_background_r("background_r", 0),
    prop_background_g("background_g", 0),
    prop_background_b("background_b", 0),
    prop_bounding_box_r("bounding_box_r", 1.0),
    prop_bounding_box_g("bounding_box_g", 1.0),
    prop_bounding_box_b("bounding_box_b", 1.0),
    prop_point_r("point_r", 0),
    prop_point_g("point_g", 255),
    prop_point_b("point_b", 0),
    prop_point_size("point_size", 2)

{
  registerProperty(prop_window_name);
  registerProperty(prop_coordinate_system);
  registerProperty(prop_two_viewports);
  registerProperty(prop_background_r);
  registerProperty(prop_background_g);
  registerProperty(prop_background_b);
  registerProperty(prop_bounding_box_r);
  registerProperty(prop_bounding_box_g);
  registerProperty(prop_bounding_box_b);
  registerProperty(prop_point_r);
  registerProperty(prop_point_g);
  registerProperty(prop_point_b);
  registerProperty(prop_point_size);
  
}

CloudViewer::~CloudViewer() {
}

void CloudViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyz2", &in_cloud_xyz2);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb2", &in_cloud_xyzrgb2);
	registerStream("in_cloud_normals", &in_cloud_normals);
    registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);

    registerStream("in_min_pt", &in_min_pt);
    registerStream("in_max_pt", &in_max_pt);
    registerStream("in_point", &in_point);

	// Register handlers
	h_on_cloud_xyz.setup(boost::bind(&CloudViewer::on_cloud_xyz, this));
	registerHandler("on_cloud_xyz", &h_on_cloud_xyz);
	addDependency("on_cloud_xyz", &in_cloud_xyz);
	h_on_clouds_xyz.setup(boost::bind(&CloudViewer::on_clouds_xyz, this));
	registerHandler("on_clouds_xyz", &h_on_clouds_xyz);
	addDependency("on_clouds_xyz", &in_cloud_xyz);
	addDependency("on_clouds_xyz", &in_cloud_xyz2);
	h_on_cloud_xyzrgb.setup(boost::bind(&CloudViewer::on_cloud_xyzrgb, this));
	registerHandler("on_cloud_xyzrgb", &h_on_cloud_xyzrgb);
	addDependency("on_cloud_xyzrgb", &in_cloud_xyzrgb);
	h_on_clouds_xyzrgb.setup(boost::bind(&CloudViewer::on_clouds_xyzrgb, this));
	registerHandler("on_clouds_xyzrgb", &h_on_clouds_xyzrgb);
	addDependency("on_clouds_xyzrgb", &in_cloud_xyzrgb);
	addDependency("on_clouds_xyzrgb", &in_cloud_xyzrgb2);
    h_on_cloud_xyzsift.setup(boost::bind(&CloudViewer::on_cloud_xyzsift, this));
    registerHandler("on_cloud_xyzsift", &h_on_cloud_xyzsift);
    addDependency("on_cloud_xyzsift", &in_cloud_xyzsift);
	h_on_cloud_normals.setup(boost::bind(&CloudViewer::on_cloud_normals, this));
	registerHandler("on_cloud_normals", &h_on_cloud_normals);
	addDependency("on_cloud_normals", &in_cloud_normals);
    h_on_bounding_box.setup(boost::bind(&CloudViewer::on_bounding_box, this));
    registerHandler("on_bounding_box", &h_on_bounding_box);
    addDependency("on_bounding_box", &in_min_pt);
    addDependency("on_bounding_box", &in_max_pt);
    h_on_point.setup(boost::bind(&CloudViewer::on_point, this));
    registerHandler("on_point", &h_on_point);
    addDependency("on_point", &in_point);
	h_on_spin.setup(boost::bind(&CloudViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CloudViewer::onInit() {

	if(prop_two_viewports){
        LOG(LTRACE) << "CloudViewer::onInit, prop_two_viewports==true\n";
		viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
		v1 = 0;
		v2 = 1;
		viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor (0, 0, 0, v1);
		
		viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);			
	}
	else{
        LOG(LTRACE) << "CloudViewer::onInit, prop_two_viewports==false\n";
		viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
		viewer->setBackgroundColor(prop_background_r,prop_background_g, prop_background_b);

		viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
	}
	// Add coordinate system -- different function call depending on the PCL version(!)
	if(prop_coordinate_system) {
//#if PCL_VERSION_COMPARE(>=,1,7,1)
//		viewer->addCoordinateSystem (1.0, "ClustersViewer", 0);
//#else
		viewer->addCoordinateSystem (1.0);
//#endif
	}
		
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
	viewer->initCameraParameters ();
	return true;
}

bool CloudViewer::onFinish() {
	return true;
}

bool CloudViewer::onStop() {
	return true;
}

bool CloudViewer::onStart() {
	return true;
}

void CloudViewer::on_cloud_xyz() {
	LOG(LTRACE) << "CloudViewer::on_cloud_xyz\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	viewer->updatePointCloud<pcl::PointXYZ> (cloud, "sample cloud");
}

void CloudViewer::on_clouds_xyz() {
	if(!prop_two_viewports)
		LOG(LDEBUG) << "Set property two_viewports = 1\n";
	LOG(LTRACE) << "CloudViewer::on_clouds_xyz\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = in_cloud_xyz2.read();

	viewer->removePointCloud("viewcloud",v1) ;
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "viewcloud", v1);
	
	viewer->removePointCloud("viewcloud2",v2) ;
	viewer->addPointCloud<pcl::PointXYZ> (cloud2, "viewcloud2", v2);
}

void CloudViewer::on_cloud_xyzrgb() {
	LOG(LTRACE) << "CloudViewer::on_cloud_xyzrgb\n";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	
	// Filter the NaN points.
	std::vector<int> indices;
	cloud->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution(cloud);
	viewer->removePointCloud("viewcloud") ;
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_distribution, "viewcloud") ;
}

void CloudViewer::on_cloud_xyzsift() {
    LOG(LTRACE) << "CloudViewer::on_cloud_xyzsift\n";
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

    // Filter the NaN points.
    std::vector<int> indices;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    viewer->removePointCloud("siftcloud") ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*cloud_xyz);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz, "siftcloud") ;
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_point_size, "siftcloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
        255,
        0,
        0,
        "siftcloud");
}


void CloudViewer::on_clouds_xyzrgb() {
	if(!prop_two_viewports)
		LOG(LDEBUG) << "Set property two_viewports = 1\n";
	LOG(LTRACE) << "CloudViewer::on_clouds_xyzrgb\n";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = in_cloud_xyzrgb2.read();

	std::vector<int> indices;
	cloud->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	std::vector<int> indices2;
	cloud2->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices2);

	viewer->removePointCloud("viewcloud",v1) ;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "viewcloud", v1);
	
	viewer->removePointCloud("viewcloud2",v2) ;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (cloud2);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb2, "viewcloud2", v2);
}

void CloudViewer::on_cloud_normals() {
}

void CloudViewer::on_bounding_box(){
    pcl::PointXYZ minPt = in_min_pt.read();
    pcl::PointXYZ maxPt = in_max_pt.read();

    viewer->addCube (minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z, prop_bounding_box_r, prop_bounding_box_g, prop_bounding_box_b);

}

void CloudViewer::on_point(){
	pcl::PointXYZ point = in_point.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->push_back(point);
	cout<<"point " <<point.x<< " " <<point.y << " " <<point.z<<endl;
	cout<<"size: "<<cloud->size()<<endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud, prop_point_r, prop_point_g, prop_point_b);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "centroid");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, prop_point_size, "centroid");
}

void CloudViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace CloudViewer
} //: namespace Processors
