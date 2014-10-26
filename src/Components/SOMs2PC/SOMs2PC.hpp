/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef SOMS2PC_HPP_
#define SOMS2PC_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/SIFTObjectModel.hpp>



namespace Processors {
namespace SOMs2PC {

/*!
 * \class SOMs2PC
 * \brief SOMs2PC processor class.
 *
 * SOMs2PC processor.
 */
class SOMs2PC: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SOMs2PC(const std::string & name = "SOMs2PC");

	/*!
	 * Destructor
	 */
	virtual ~SOMs2PC();

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

	// Receive and store SOMS.
	void receiveSOMs();

	// Returns clouds of a selected model.
	void returnSelectedSOMClouds();

	/// Input data stream containing list of SIFT Object Models.
	Base::DataStreamIn<std::vector<AbstractObject*>, Base::DataStreamBuffer::Newest> in_models;

	/// Output XYZRGB cloud
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;

	/// Output XYZSIFT cloud
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	/// Number of the model from list of the model - used for generation of names of JSON ("major" file) and PCDs ("minor" files containing clouds).
	Base::Property<int> prop_model_number;

	// Local copy of models.
	std::vector<SIFTObjectModel*> models;

};

} //: namespace SOMs2PC
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMs2PC", Processors::SOMs2PC::SOMs2PC)

#endif /* SOMS2PC_HPP_ */
