/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef PROJECTION_HPP_
#define PROJECTION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace Processors {
namespace Projection {

/*!
 * \class Projection
 * \brief Projection processor class.
 *
 * 
 */
class Projection: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Projection(const std::string & name = "Projection");

	/*!
	 * Destructor
	 */
	virtual ~Projection();

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
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz_scene;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz_model;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb_scene;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb_model;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift_scene;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift_model;
	Base::DataStreamIn<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > in_rototranslations;

	// Output data streams
    Base::DataStreamOut<std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> > out_registered_instances_xyzsift;
    Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> > out_registered_instances_xyzrgb;
    Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> > out_registered_instances_xyz;

	// Properties
    Base::Property<int> icp_max_iter;
    Base::Property<float> icp_corr_distance;
	
	// Handlers
    void project();
    void project_xyzrgb();
    void project_xyzsift();

};

} //: namespace Projection
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Projection", Processors::Projection::Projection)

#endif /* PROJECTION_HPP_ */
