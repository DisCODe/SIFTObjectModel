/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef XYZRGB2XYZSIFT_HPP_
#define XYZRGB2XYZSIFT_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Types/PointXYZSIFT.hpp>

#include <opencv2/opencv.hpp>

namespace Processors {
namespace xyzrgb2xyzsift {

/*!
 * \class xyzrgb2xyzsift
 * \brief xyzrgb2xyzsift processor class.
 *
 * 
 */
class xyzrgb2xyzsift: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	xyzrgb2xyzsift(const std::string & name = "xyzrgb2xyzsift");

	/*!
	 * Destructor
	 */
	virtual ~xyzrgb2xyzsift();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Handlers
	Base::EventHandler2 h_compute;

	// Properties

	
	// Handlers
	void compute();

};

} //: namespace xyzrgb2xyzsift
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("xyzrgb2xyzsift", Processors::xyzrgb2xyzsift::xyzrgb2xyzsift)

#endif /* XYZRGB2XYZSIFT_HPP_ */
