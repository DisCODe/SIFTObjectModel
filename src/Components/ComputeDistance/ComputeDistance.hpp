/*!
 * \file
 * \brief 
 * \author Mikolaj Kamionka
 */

#ifndef COMPUTEDISTANCE_HPP_
#define COMPUTEDISTANCE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace ComputeDistance {

/*!
 * \class ComputeDistance
 * \brief ComputeDistance processor class.
 *
 * 
 */
class ComputeDistance: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ComputeDistance(const std::string & name = "ComputeDistance");

	/*!
	 * Destructor
	 */
	virtual ~ComputeDistance();

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
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift1;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift2;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb1;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb2;

	Base::DataStreamOut<double> out_distance;

	// Properties

	// Handlers
	Base::EventHandler<ComputeDistance> h_computeSIFTDist;
	Base::EventHandler<ComputeDistance> h_computeRGBDist;
    Base::EventHandler<ComputeDistance> h_computeTransDist;


	void computeSIFTDist();

	void computeRGBDist();

    void computeTransDist();

};

} //: namespace ComputeDistance
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ComputeDistance", Processors::ComputeDistance::ComputeDistance)

#endif /* COMPUTEDISTANCE_HPP_ */
