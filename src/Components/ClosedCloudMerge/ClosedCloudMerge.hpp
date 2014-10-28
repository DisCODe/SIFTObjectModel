

#ifndef CLOSEDCLOUDMERGE_HPP_
#define CLOSEDCLOUDMERGE_HPP_

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
namespace ClosedCloudMerge {

class ClosedCloudMerge: public Base::Component,SIFTObjectModelFactory {

public:
	/*!
	 * Constructor.
	 */
    ClosedCloudMerge(const std::string & name = "ClosedCloudMerge");
	/*!
	 * Destructor
	 */
    virtual ~ClosedCloudMerge();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	/// Input data stream containing point cloud with normals from a given view.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> in_cloud_xyzrgb_normals;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

	/// Output data stream containing SIFTObjectModel - depricated.
	Base::DataStreamOut<AbstractObject*> out_instance;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;

	/// Output data stream containing object model point cloud with normals.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb_normals;


	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Mean number of features per view.
	Base::DataStreamOut<int> out_mean_viewpoint_features_number;

	// Handlers
    Base::EventHandler2 h_addViewToModel;
    Base::EventHandler2 h_addViewToModelNormals;

    void addViewToModel();

    void addViewToModelNormals();

    MergeUtils::Properties properties;

	/// Number of views.
	int counter;


	/// Total number of features (in all views).
	int total_viewpoint_features_number;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal_merged;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift_merged;
	Eigen::Matrix4f global_trans;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_views;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> rgbn_views;
    	pcl::registration::LUM<PointXYZSIFT> lum_sift;


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

    Base::Property<int> viewNumber, maxIterations, corrTreshold;
};

REGISTER_COMPONENT("ClosedCloudMerge", Processors::ClosedCloudMerge::ClosedCloudMerge)

} // namespace Processors
} // namespace ClosedCloudMerge

#endif /* CLOSEDCLOUDMERGE_HPP_ */
