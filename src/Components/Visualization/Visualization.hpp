/*!
 * \file
 * \brief 
 * \author Marta Lepicka
 */

#ifndef VISUALIZATION_HPP_
#define VISUALIZATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp> 
#include <Types/SIFTObjectModel.hpp> 
#include <Types/SIFTObjectModelFactory.hpp> 

#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"


namespace Processors {
namespace Visualization {

/*!
 * \class Visualization
 * \brief Visualization processor class.
 *
 * Visualization processor.
 */
class Visualization: public Base::Component,SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	Visualization(const std::string & name = "Visualization");

	/*!
	 * Destructor
	 */
	virtual ~Visualization();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
			pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	
	void visualize();
	
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


// Input data streams


// Output data streams

	// Handlers
	Base::EventHandler2 h_visualize;
	Base::Property<string> filenames;
	// Handlers

	void onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames);
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;

	Base::Property<float> radius_search;

};

} //: namespace Visualization
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Visualization", Processors::Visualization::Visualization)

#endif /* VISUALIZATION_HPP_ */
