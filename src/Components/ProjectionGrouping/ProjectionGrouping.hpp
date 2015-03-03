/*!
 * \file
 * \brief 
 * \author Michał Laszkowski
 */

#ifndef PROJECTIONGROUPING_HPP_
#define PROJECTIONGROUPING_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/correspondence_estimation.h>


namespace Processors {
namespace ProjectionGrouping {

/*!
 * \class ProjectionGrouping
 * \brief ProjectionGrouping processor class.
 *
 * 
 */
class ProjectionGrouping: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ProjectionGrouping(const std::string & name = "ProjectionGrouping");

	/*!
	 * Destructor
	 */
	virtual ~ProjectionGrouping();

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

    /// Get 8 points XYZ cloud BoundingBox
    pcl::PointCloud<pcl::PointXYZ>::Ptr getBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getBoundingBox(pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift);

    float cuboidIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid1, pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid2);

    void threePointsToPlane (const pcl::PointXYZ &point_a, const pcl::PointXYZ &point_b, const pcl::PointXYZ &point_c, const pcl::ModelCoefficients::Ptr plane);

	// Input data streams
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb_model;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift_model;
    Base::DataStreamIn<std::vector<pcl::Correspondences> > in_clustered_correspondences;
    Base::DataStreamIn<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > in_rototranslations;

	// Output data streams
    Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_projections;
    Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr > out_model_bounding_box;

	// Properties
    ///Precision of Monte carlo method
    Base::Property<int> mcn;
	
	// Handlers
	void group();

};

} //: namespace ProjectionGrouping
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ProjectionGrouping", Processors::ProjectionGrouping::ProjectionGrouping)

#endif /* PROJECTIONGROUPING_HPP_ */
