/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SIFTOBJECTMATCHER_HPP_
#define SIFTOBJECTMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include <Types/PointXYZSIFT.hpp> 
#include <Types/SIFTObjectModel.hpp> 
#include <pcl/point_representation.h>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>

namespace Processors {
namespace SIFTObjectMatcher {

/*!
 * \class SIFTObjectMatcher
 * \brief SIFTObjectMatcher processor class.
 *
 * SIFTObjectMatcher processor.
 */
class SIFTObjectMatcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTObjectMatcher(const std::string & name = "SIFTObjectMatcher");

	/*!
	 * Destructor
	 */
	virtual ~SIFTObjectMatcher();

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


// Input data streams

	Base::DataStreamIn<std::vector<AbstractObject*> > in_models;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb_model;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift_model;
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences;
    Base::DataStreamOut<pcl::CorrespondencesPtr> out_good_correspondences;

	// Handlers
	Base::EventHandler2 h_readModels;
	Base::EventHandler2 h_match;

	
	// Handlers
	void readModels();
	void match();

	std::vector<SIFTObjectModel*> models;
	
	Base::Property<float> threshold;
	Base::Property<float> inlier_threshold;
    Base::Property<float> max_distance;

};

} //: namespace SIFTObjectMatcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTObjectMatcher", Processors::SIFTObjectMatcher::SIFTObjectMatcher)

#endif /* SIFTOBJECTMATCHER_HPP_ */
