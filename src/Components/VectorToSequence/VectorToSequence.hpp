#ifndef VECTORTOSEQENCE_HPP_
#define VECTORTOSEQENCE_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

//#include <Types/SIFTObjectModel.hpp> 
#include <Types/SIFTObjectModelFactory.hpp> 
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace VectorToSequence {

/*!
 * \class VectorToSequence
 * \brief VectorToSequence processor class.
 *
 * VECTORTOSEQENCE processor.
 */
class VectorToSequence: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	VectorToSequence(const std::string & name = "VectorToSequence");

	/*!
	 * Destructor
	 */
	virtual ~VectorToSequence();

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

        Base::DataStreamIn<Base::UnitType> in_trigger;

	/// Output data stream containing models.
	Base::DataStreamIn<std::vector<AbstractObject*> > in_models;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Handlerswas
	Base::EventHandler2 h_loadModels;
	Base::EventHandler2 h_throwClouds;

	/// Load models from files.
	void loadModels();
	void throwClouds();

	std::vector<AbstractObject*> models;
	int place;
};

} //: namespace VectorToSequence
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("VectorToSequence", Processors::VectorToSequence::VectorToSequence)

#endif /* VECTORTOSEQENCE_HPP_ */
