/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef DOWNSAMPLING_HPP_
#define DOWNSAMPLING_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp> 

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace Downsampling {

/*!
 * \class Downsampling
 * \brief Downsampling processor class.
 *
 * Downsampling processor.
 */
class Downsampling: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Downsampling(const std::string & name = "Downsampling");

	/*!
	 * Destructor
	 */
	virtual ~Downsampling();

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

		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

// Output data streams

		Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
	// Handlers
	Base::EventHandler2 h_downsample_xyzsift;

	
	// Handlers
	void downsample_xyzsift();
	
	
	Base::Property<float> radius;

};

} //: namespace Downsampling
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Downsampling", Processors::Downsampling::Downsampling)

#endif /* DOWNSAMPLING_HPP_ */
