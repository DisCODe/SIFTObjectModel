/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SINGLESOMJSONREADER_HPP_
#define SINGLESOMJSONREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointCloudObject.hpp>
#include <Types/PointXYZSIFT.hpp>

#include <Types/SIFTObjectModelFactory.hpp>

namespace Processors {
namespace SingleSOMJsonReader {

/*!
 * \class SingleSOMJsonReader
 * \brief SingleSOMJsonReader processor class.
 *
 * SingleSOMJsonReader processor.
 */
class SingleSOMJsonReader: public Base::Component, SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	SingleSOMJsonReader(const std::string & name = "SingleSOMJsonReader");

	/*!
	 * Destructor
	 */
	virtual ~SingleSOMJsonReader();

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

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
	Base::DataStreamOut<std::string> out_name;

	// Handlers
	Base::EventHandler2 h_loadModel;

	// Properties
	Base::Property<string> filenames;


	// Handlers
	void loadModel();

	

};

} //: namespace SingleSOMJsonReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SingleSOMJsonReader", Processors::SingleSOMJsonReader::SingleSOMJsonReader)

#endif /* SINGLESOMJSONREADER_HPP_ */
