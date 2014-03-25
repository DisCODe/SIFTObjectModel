/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "CorrespondencesViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/registration/correspondence_estimation.h>

namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name),
		prop_window_name("window_name", std::string("Correspondences Viewer"))  {
			registerProperty(prop_window_name);

}

CorrespondencesViewer::~CorrespondencesViewer() {
}

void CorrespondencesViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud_xyzsift1", &in_cloud_xyzsift1);
registerStream("in_cloud_xyzsift2", &in_cloud_xyzsift2);
registerStream("in_cloud_xyzrgb1", &in_cloud_xyzrgb1);
registerStream("in_cloud_xyzrgb2", &in_cloud_xyzrgb2);
registerStream("in_correspondences", &in_correspondences);
	// Register handlers
	h_on_clouds.setup(boost::bind(&CorrespondencesViewer::on_clouds, this));
	registerHandler("on_clouds", &h_on_clouds);
	addDependency("on_clouds", &in_cloud_xyzsift1);
	addDependency("on_clouds", &in_cloud_xyzsift2);
	addDependency("on_clouds", &in_cloud_xyzrgb1);
	addDependency("on_clouds", &in_correspondences);
	addDependency("on_clouds", &in_cloud_xyzrgb2);
	// Register spin handler.
	h_on_spin.setup(boost::bind(&CorrespondencesViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CorrespondencesViewer::onInit() {
	LOG(LTRACE) << "CorrespondencesViewer::onInit";
	viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	return true;
}

bool CorrespondencesViewer::onFinish() {
	return true;
}

bool CorrespondencesViewer::onStop() {
	return true;
}

bool CorrespondencesViewer::onStart() {
	return true;
}

void CorrespondencesViewer::on_clouds() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb1 = in_cloud_xyzrgb1.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb2 = in_cloud_xyzrgb2.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift1 = in_cloud_xyzsift1.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift2 = in_cloud_xyzsift2.read();
	pcl::CorrespondencesPtr correspondences = in_correspondences.read();
	
	
	//Define a small translation between clouds	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
	float tx = 0.3f, ty = 0.0f, tz = 0.0f ;
	trans(0, 3) = tx ; trans(1, 3) = ty ; trans(2, 3) = tz ;

	//Transform one of the clouds	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb2trans(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift2trans(new pcl::PointCloud<PointXYZSIFT>) ;
	pcl::transformPointCloud(*cloud_xyzrgb2, *cloud_xyzrgb2trans, trans) ;
	pcl::transformPointCloud(*cloud_xyzsift2, *cloud_xyzsift2trans, trans) ;
	
	//Display clouds
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution1(cloud_xyzrgb1);
	viewer->removePointCloud("viewcloud1") ;
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_xyzrgb1, color_distribution1, "viewcloud1") ;
	viewer->removePointCloud("siftcloud1") ;
	viewer->addPointCloud<PointXYZSIFT>(cloud_xyzsift1, "siftcloud1") ;
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "siftcloud1");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "siftcloud1");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution2(cloud_xyzrgb2trans);
	viewer->removePointCloud("viewcloud2") ;
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_xyzrgb2trans, color_distribution2, "viewcloud2") ;
	viewer->removePointCloud("siftcloud2") ;
	viewer->addPointCloud<PointXYZSIFT>(cloud_xyzsift2trans, "siftcloud2") ;
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "siftcloud2");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "siftcloud2");

	//Display correspondences
	viewer->removeCorrespondences("correspondences") ;
	viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift1, cloud_xyzsift2trans, *correspondences, "correspondences") ;
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "correspondences") ;
	
}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}


} //: namespace CorrespondencesViewer
} //: namespace Processors
