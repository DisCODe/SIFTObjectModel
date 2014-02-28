/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef PC2Octree_HPP_
#define PC2Octree_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Types/PointXYZSIFT.hpp>


namespace Processors {
namespace PC2Octree {

/*!
 * \class PC2Octree
 * \brief PC2Octree processor class.
 *
 * PC2Octree processor.
 */
class PC2Octree: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PC2Octree(const std::string & name = "PC2Octree");

	/*!
	 * Destructor
	 */
	virtual ~PC2Octree();

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

	/// Input cloud containing XYZ points.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr > in_cloud_xyz;

	/// Input cloud containing XYZRGB points.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;

	/// Input cloud containing XYZSIFT points.
        Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud_xyzsift;

	// Output data streams

	/// Handler calling function cloud_xyzrgb_to_octree(). 
	Base::EventHandler2 h_cloud_xyzrgb_to_octree;
	
	/// Function putting xyzrgb cloud to octree.
	void cloud_xyzrgb_to_octree();

	

};

} //: namespace PC2Octree
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PC2Octree", Processors::PC2Octree::PC2Octree)

#endif /* PC2Octree_HPP_ */
