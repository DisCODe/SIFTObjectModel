/*!
 * \file
 * \brief 
 * \author Tomek Kornuta,,,
 */

#ifndef INSTANCEGENERATOR_HPP_
#define INSTANCEGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace InstanceGenerator {

/*!
 * \class InstanceGenerator
 * \brief InstanceGenerator processor class.
 *
 * InstanceGenerator processor.
 */
class InstanceGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	InstanceGenerator(const std::string & name = "InstanceGenerator");

	/*!
	 * Destructor
	 */
	virtual ~InstanceGenerator();

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


	

};

} //: namespace InstanceGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("InstanceGenerator", Processors::InstanceGenerator::InstanceGenerator)

#endif /* INSTANCEGENERATOR_HPP_ */
