/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SOMGENERATOR_HPP_
#define SOMGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp> 
#include <Types/SIFTObjectModel.hpp> 
#include <Types/SIFTObjectModelFactory.hpp> 

#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

#include <opencv2/core/core.hpp>


namespace Processors {
namespace SOMGenerator {

/*!
 * \class SOMGenerator
 * \brief SOMGenerator processor class.
 *
 * SOMGenerator processor.
 */
class SOMGenerator: public Base::Component,SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
    SOMGenerator(const std::string & name = "SOMGenerator");

	/*!
	 * Destructor
	 */
    virtual ~SOMGenerator();

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

	/// Input data stream containing point cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> in_cloud_xyzrgb;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

	/// Output data stream containing SIFTObjectModel - depricated.
	Base::DataStreamOut<AbstractObject*> out_instance; 
		
	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb;

	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Mean number of features per view. 
	Base::DataStreamOut<int> out_mean_viewpoint_features_number;

	// Handlers
    Base::EventHandler2 h_addViewToModel;
    Base::EventHandler2 h_Trigger;
	
	// Handlers
    void addViewToModel();
    //void ICP_points();

	/// Computes the transformation between two XYZSIFT clouds basing on the found correspondences.
	Eigen::Matrix4f computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg, 
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers);


	/// Number of views.	
	int counter;

	/// Total number of features (in all views).	
	int total_viewpoint_features_number;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_merged;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift_merged;
	Eigen::Matrix4f global_trans;

    /// Alignment mode: use ICP alignment or not.
	/// ICP properties
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

} //: namespace SOMGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMGenerator", Processors::SOMGenerator::SOMGenerator)

#endif /* SOMGENERATOR_HPP_ */
