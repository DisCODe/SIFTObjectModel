/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "Visualization.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

namespace Processors {
namespace Visualization {

Visualization::Visualization(const std::string & name) :
		Base::Component(name),
		filenames("filenames", boost::bind(&Visualization::onFilenamesChanged, this, _1, _2), ""),
		radius_search("radius",0.05)
		{
			registerProperty(filenames);
			registerProperty(radius_search);

}

Visualization::~Visualization() {
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualization::normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualization::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void Visualization::visualize(){

	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filenames, *cloud_xyzrgb) == -1)
	  {
		cout <<"Cannot read PointXYZRGB cloud from "<<filenames;
	  }
	  else
		  point_cloud_ptr=cloud_xyzrgb;
	  
	  std::cout<<"Rozmiar wczytanej chmury: "<<cloud_xyzrgb->points.size()<<std::endl;
	  // ----------------------------------------------------------------
	   // -----Calculate surface normals with a search radius of 0.05-----
	   // ----------------------------------------------------------------
	   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	   ne.setInputCloud (point_cloud_ptr);
	   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	   ne.setSearchMethod (tree);
	   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
	   ne.setRadiusSearch (radius_search); //property
	   ne.compute (*cloud_normals1);

	   // ---------------------------------------------------------------
	   // -----Calculate surface normals with a search radius of 0.1-----
	   // ---------------------------------------------------------------
	   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	   ne.setRadiusSearch (radius_search); //property
	   ne.compute(*cloud_normals2);
	   

	   rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(point_cloud_ptr);
	   viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
	   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	   viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr, cloud_normals2, 10, 0.05, "normals");
	   //--------------------
	   // -----Main loop-----
	   //--------------------
	   if (!viewer->wasStopped ())
	   {
	     viewer->spinOnce (100);

	   }

	   viewer->removeAllPointClouds();
}

void Visualization::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers

    // Register single handler - the "visualize()" function.
    h_visualize.setup(boost::bind(&Visualization::visualize, this));
    registerHandler("Visualize", &h_visualize);
    addDependency("Visualize", NULL);
}

bool Visualization::onInit() {
	  basic_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	  point_cloud_ptr =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	  cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	  viewer= boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);

	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();

	return true;
}

bool Visualization::onFinish() {
	return true;
}

bool Visualization::onStop() {
	return true;
}

bool Visualization::onStart() {
	return true;
}
void Visualization::onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames) {
	filenames = new_filenames;
	CLOG(LTRACE) << "onFilenamesChanged: " << std::string(filenames) << std::endl;
}





} //: namespace Visualization
} //: namespace Processors
