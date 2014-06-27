/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef PCDWRITER_HPP_
#define PCDWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace PCDWriter {

/*!
 * \class PCDWriter
 * \brief PCDWriter processor class.
 *
 * 
 */
class PCDWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PCDWriter(const std::string & name = "PCDWriter");

	/*!
	 * Destructor
	 */
	virtual ~PCDWriter();

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

	// Handlers
	Base::EventHandler2 h_Write;

	// Properties
	Base::Property<std::string> filename;

	
	// Handlers
	void Write();

};

} //: namespace PCDWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PCDWriter", Processors::PCDWriter::PCDWriter)

#endif /* PCDWRITER_HPP_ */
