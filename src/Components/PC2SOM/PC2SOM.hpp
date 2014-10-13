/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef PC2SOM_HPP_
#define PC2SOM_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/SIFTObjectModelFactory.hpp>


namespace Processors {
namespace PC2SOM {

/*!
 * \class PC2SOM
 * \brief PC2SOM processor class.
 *
 * 
 */
class PC2SOM: public Base::Component, SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	PC2SOM(const std::string & name = "PC2SOM");

	/*!
	 * Destructor
	 */
	virtual ~PC2SOM();

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
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzrgb;
    Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzsift;
    Base::DataStreamIn<int, Base::DataStreamBuffer::Newest> in_mean_viewpoint_features_number;

	// Output data streams
	Base::DataStreamOut<AbstractObject*> out_model;

	// Handlers
	Base::EventHandler2 h_createSOM;

	// Properties
    Base::Property<std::string> SOMname;
	
	// Handlers
	void createSOM();

};

} //: namespace PC2SOM
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PC2SOM", Processors::PC2SOM::PC2SOM)

#endif /* PC2SOM_HPP_ */
