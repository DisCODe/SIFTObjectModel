/*!
 * \file
 * \brief 
 * \author Marta Lepicka
 */

#ifndef ComputeTrans_HPP_
#define ComputeTrans_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

namespace Processors {
namespace ComputeTrans {

/*!
 * \class ComputeTrans
 * \brief ComputeTrans processor class.
 *
 * 
 */
class ComputeTrans: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ComputeTrans(const std::string & name = "ComputeTrans");

	/*!
	 * Destructor
	 */
	virtual ~ComputeTrans();

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

	// Properties

	// Handlers
	Base::EventHandler<ComputeTrans> h_compute;

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_old;
	bool first;

	void Compute();

};

} //: namespace ComputeTrans
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ComputeTrans", Processors::ComputeTrans::ComputeTrans)

#endif /* COMPUTETRANS_HPP_ */
