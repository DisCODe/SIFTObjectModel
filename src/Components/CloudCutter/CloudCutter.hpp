/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef CLOUDCUTTER_HPP_
#define CLOUDCUTTER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp> 

namespace Processors {
namespace CloudCutter {

/*!
 * \class ExtractIndices
 * \brief ExtractIndices processor class.
 *
 * ExtractIndices processor.
 */
class CloudCutter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudCutter(const std::string & name = "CloudCutter");

	/*!
	 * Destructor
	 */
	virtual ~CloudCutter();

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

		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud;
		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_indices;

// Output data streams

		Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud;
	// Handlers
	Base::EventHandler2 h_cut;
		Base::Property<float> radius;

	
	// Handlers
	void cut();

};

} //: namespace CloudCutter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudCutter", Processors::CloudCutter::CloudCutter)

#endif /* CLOUDCUTTER_HPP_ */
