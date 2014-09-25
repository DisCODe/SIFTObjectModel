
#ifndef CORRESPONDENCEMATCHER_HPP_
#define CORRESPONDENCEMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/MergeUtils.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>
#include <Types/SIFTObjectModelFactory.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

#include <opencv2/core/core.hpp>
#include <pcl/registration/lum.h>


namespace Processors {
namespace CorrespondenceMatcher {

class CorrespondenceMatcher: public Base::Component,SIFTObjectModelFactory {

public:
	/*!
	 * Constructor.
	 */
    CorrespondenceMatcher(const std::string & name = "CorrespondenceMatcher");
	/*!
	 * Destructor
	 */
    virtual ~CorrespondenceMatcher();

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

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_first_xyzsift;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_sec_xyzsift;
	
	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_first_xyzsift;
	
	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_sec_xyzsift;

	/// Output data stream containing corespondences beetwen previous and last cloud
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences;

	// Mean number of features per view.
	Base::DataStreamOut<int> out_mean_viewpoint_features_number;

	// Handlers
	Base::EventHandler2 h_mach;

	void mach();

	MergeUtils::Properties properties;

	Eigen::Matrix4f global_trans;


public:
    Base::Property<bool> prop_ICP_alignment;
    Base::Property<bool> prop_ICP_alignment_normal;
    Base::Property<bool> prop_ICP_alignment_color;
    Base::Property<double> ICP_transformation_epsilon;
    Base::Property<float> ICP_max_correspondence_distance;
    Base::Property<int> ICP_max_iterations;

    ///RanSAC Properties
    Base::Property<float> RanSAC_inliers_threshold;
    Base::Property<float> RanSAC_max_iterations;

};

REGISTER_COMPONENT("CorrespondenceMatcher", Processors::CorrespondenceMatcher::CorrespondenceMatcher)

} // namespace Processors
} // namespace CorrespondenceMatcher

#endif /* CORRESPONDENCEMATCHER_HPP_ */
