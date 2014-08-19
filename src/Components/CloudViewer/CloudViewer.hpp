/*!
 * \file
 * \brief 
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#ifndef CLOUDVIEWER_HPP_
#define CLOUDVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Logger.hpp"

#include "EventHandler2.hpp"
#include "Property.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <pcl/visualization/pcl_visualizer.h>


namespace Processors {
namespace CloudViewer {

/*!
 * \class CloudViewer
 * \brief CloudViewer processor class.
 *
 * Pointcloud viewer with normals visualization
 */
class CloudViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudViewer(const std::string & name = "CloudViewer");

	/*!
	 * Destructor
	 */
	virtual ~CloudViewer();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	// Data streams
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz2;
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb2;
	Base::DataStreamIn< pcl::PointCloud<pcl::Normal>::Ptr > in_cloud_normals;
    Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud_xyzsift;

    Base::DataStreamIn<pcl::PointXYZ> in_min_pt;
    Base::DataStreamIn<pcl::PointXYZ> in_max_pt;

	Base::DataStreamIn<pcl::PointXYZ> in_point;
	// Handlers
	Base::EventHandler2 h_on_cloud_xyz;
	Base::EventHandler2 h_on_clouds_xyz;
	Base::EventHandler2 h_on_cloud_xyzrgb;
    Base::EventHandler2 h_on_cloud_xyzsift;
	Base::EventHandler2 h_on_clouds_xyzrgb;
	Base::EventHandler2 h_on_cloud_normals;
    Base::EventHandler2 h_on_bounding_box;
    Base::EventHandler2 h_on_point;
	Base::EventHandler2 h_on_spin;

	
	// Handlers
	void on_cloud_xyz();
	void on_clouds_xyz();
	void on_cloud_xyzrgb();
	void on_clouds_xyzrgb();
	void on_cloud_normals();
    void on_cloud_xyzsift();
    void on_bounding_box();
    void on_point();
	void on_spin();

    Base::Property<std::string> prop_window_name;
    Base::Property<bool> prop_coordinate_system;
    Base::Property<bool> prop_two_viewports;
    Base::Property<int> prop_background_r;
    Base::Property<int> prop_background_g;
    Base::Property<int> prop_background_b;
    Base::Property<float> prop_bounding_box_r;
    Base::Property<float> prop_bounding_box_g;
    Base::Property<float> prop_bounding_box_b;
    Base::Property<float> prop_point_r;
    Base::Property<float> prop_point_g;
    Base::Property<float> prop_point_b;
    Base::Property<float> prop_point_size;


	pcl::visualization::PCLVisualizer * viewer;
	pcl::visualization::PCLVisualizer * viewer2;
	int v1,v2;
	
};

} //: namespace CloudViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudViewer", Processors::CloudViewer::CloudViewer)

#endif /* CLOUDVIEWER_HPP_ */
