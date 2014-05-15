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

void Visualization::refresh(){
	viewer->spinOnce(100);
	//viewer2->spinOnce(100);
}

void Visualization::visualize(){
	//visualization of model
//	viewer2->removeAllPointClouds();
	viewer->removeAllPointClouds();
/*
	  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filenames, *cloud_xyzrgb) == -1)
	  {
		cout <<"Cannot read PointXYZRGB cloud from "<<filenames;
	  }
	  else
		  point_cloud_ptr=cloud_xyzrgb;

	  point2_cloud_ptr=in_cloud_xyzrgb.read();
	  std::cout<< "in_cloud_xyzrgb.read()->size()"<< point2_cloud_ptr->size() <<std::endl;


	   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	   ne.setInputCloud (point_cloud_ptr);
	   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	   ne.setSearchMethod (tree);

	   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	   ne.setRadiusSearch (radius_search); //property


	   //ne.setViewPoint(1.1*point_cloud_ptr->points[1].x, 1.1*point_cloud_ptr->points.data()->y, 1.1*point_cloud_ptr->points.data()->z);

	   ne.compute(*cloud_normals2);


	   pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne1;
	   ne1.setInputCloud(point2_cloud_ptr);
	  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	   ne1.setSearchMethod (tree);
	   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
	   ne1.setRadiusSearch (radius_search); //property
	   ne1.compute (*cloud_normals1);
*/
	//pomyslec nad typem tego
		//point_cloud_ptr=(pcl::PointXYZRGB)in_cloud_xyzrgb_normals.read();
		cloud_ptr_normals=in_cloud_xyzrgb_normals.read();
	   rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud_ptr_normals);
	   viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud_ptr_normals, rgb, "sample cloud");
	   viewer->addPointCloudNormals<pcl::PointXYZRGBNormal> (cloud_ptr_normals, 15, 0.05, "normals");

	   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

/*
	   rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(point2_cloud_ptr);
	   viewer2->addPointCloud<pcl::PointXYZRGB> (point2_cloud_ptr, rgb, "sample cloud");
	   viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	   viewer2->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point2_cloud_ptr, cloud_normals1, 10, 0.05, "normals");
*/

	   if (!viewer->wasStopped ())
	   {
	     viewer->spinOnce (100);
	    // viewer2->spinOnce (100);
	   }

}

void Visualization::prepareInterface() {
	// Register data streams, events and event handlers HERE!
//	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);

	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);



    // Register handlers
    h_visualize.setup(boost::bind(&Visualization::visualize, this));
    registerHandler("Visualize", &h_visualize);
    //addDependency("Visualize", &in_cloud_xyzrgb);
    addDependency("Visualize", &in_cloud_xyzrgb_normals);

    h_refresh.setup(boost::bind(&Visualization::refresh, this));
    registerHandler("Refresh", &h_refresh);
    addDependency("Refresh",NULL);
}

bool Visualization::onInit() {
	  basic_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	  point_cloud_ptr =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	  cloud_ptr_normals= pcl::PointCloud<pcl::PointXYZRGBNormal >::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal >);
	  cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	  viewer= boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);

	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();

/*
	  viewer2= boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer2->setBackgroundColor (0, 0, 0);

	  viewer2->addCoordinateSystem (1.0);
	  viewer2->initCameraParameters ();
*/
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
