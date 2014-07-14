/*!
 * \file
 * \brief 
 * \author Marta Lepicka
 */

#ifndef NORMALESTIMATION_HPP_
#define NORMALESTIMATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

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
namespace NormalEstimation {

/*!
 * \class NormalEstimation
 * \brief NormalEstimation processor class.
 *
 * NormalEstimation processor.
 */
class NormalEstimation: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	NormalEstimation(const std::string & name = "NormalEstimation");

	/*!
	 * Destructor
	 */
	virtual ~NormalEstimation();

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

//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;

	Base::Property<float> radius_search;
// Input data streams

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;

// Output data streams

	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > out_cloud_xyzrgb_normals;
	// Handlers
	Base::EventHandler2 h_compute;

	
	// Handlers
	void compute();

};

} //: namespace NormalEstimation
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("NormalEstimation", Processors::NormalEstimation::NormalEstimation)

#endif /* NORMALESTIMATION_HPP_ */
