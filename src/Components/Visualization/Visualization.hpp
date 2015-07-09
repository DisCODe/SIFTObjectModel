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
	
	void visualize();
	void visualize_normals();
	void refresh();
	
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



	/// Input data stream containing point cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > in_cloud_xyzrgb_normals;

	/// Output data stream containing SIFTObjectModel - depricated.
	Base::DataStreamOut<AbstractObject*> out_instance;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;

	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;



	// Handlers
	Base::EventHandler2 h_visualize;
	Base::EventHandler2 h_visualize_normals;
	Base::EventHandler2 h_refresh;
	// Handlers


	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr_normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_normals;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_normals;

	Base::Property<string> filenames;
	Base::Property<bool> show_normals;
	int counter;
	void onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames);
};

} //: namespace Visualization
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Visualization", Processors::Visualization::Visualization)

#endif /* VISUALIZATION_HPP_ */
