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
		show_normals("normals visible",false)
		{
			registerProperty(filenames);
			registerProperty(show_normals);
}

Visualization::~Visualization() {
}
void Visualization::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);



    // Register handlers
    h_visualize.setup(boost::bind(&Visualization::visualize, this));
    registerHandler("Visualize", &h_visualize);
    addDependency("Visualize", &in_cloud_xyzrgb);


    h_visualize_normals.setup(boost::bind(&Visualization::visualize_normals, this));
    registerHandler("Visualize_normals", &h_visualize_normals);
    addDependency("Visualize_normals", &in_cloud_xyzrgb_normals);


    h_refresh.setup(boost::bind(&Visualization::refresh, this));
    registerHandler("Refresh", &h_refresh);
    addDependency("Refresh",NULL);
}

bool Visualization::onInit() {
	  basic_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	  point_cloud_ptr=  pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	  point_cloud=  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  cloud_ptr_normals= pcl::PointCloud<pcl::PointXYZRGBNormal >::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal >);
	  cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	  viewer= boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);

	 // viewer->addCoordinateSystem (1.0);
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
void Visualization::refresh(){
	viewer->spinOnce(100);

	if(show_normals)
	{
		   if(point_cloud->size()!=cloud_ptr_normals->size())
			   viewer->addPointCloudNormals<pcl::PointXYZRGBNormal> (cloud_ptr_normals, 15, 0.05, "normals");

		   point_cloud=cloud_ptr_normals;
	}
}

void Visualization::visualize(){
	//visualization of model
	viewer->removeAllPointClouds();
/*
	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filenames, *cloud_xyzrgb) == -1)
	  {
		cout <<"Cannot read PointXYZRGB cloud from "<<filenames;
	  }
	  else
		  point_cloud_ptr=cloud_xyzrgb;

*/


	   point_cloud_ptr=in_cloud_xyzrgb.read();
	   rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(point_cloud_ptr);
	   viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud");

	   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");



	   if (!viewer->wasStopped ())
	   {
	     viewer->spinOnce (100);
	   }

}



void Visualization::visualize_normals(){
	//visualization of model
	viewer->removeAllPointClouds();
/*
	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filenames, *cloud_xyzrgb) == -1)
	  {
		cout <<"Cannot read PointXYZRGB cloud from "<<filenames;
	  }
	  else
		  point_cloud_ptr=cloud_xyzrgb;

*/
	   cloud_ptr_normals=in_cloud_xyzrgb_normals.read();

	   std::cout<<"Size of in_cloud_xyzrgb_normals: "<<cloud_ptr_normals->size()<<std::endl;

	   rgb_normals = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud_ptr_normals);

	   viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud_ptr_normals, rgb_normals, "sample cloud");

	   if(show_normals)
		   viewer->addPointCloudNormals<pcl::PointXYZRGBNormal> (cloud_ptr_normals, 15, 0.05, "normals");

	   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");


	   if (!viewer->wasStopped ())
	   {
	     viewer->spinOnce (100);
	   }
}
} //: namespace Visualization
} //: namespace Processors
