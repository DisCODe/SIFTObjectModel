/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "InstanceGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace InstanceGenerator {

InstanceGenerator::InstanceGenerator(const std::string & name) :
		Base::Component(name)  {

}

InstanceGenerator::~InstanceGenerator() {
}

void InstanceGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool InstanceGenerator::onInit() {

	return true;
}

bool InstanceGenerator::onFinish() {
	return true;
}

bool InstanceGenerator::onStop() {
	return true;
}

bool InstanceGenerator::onStart() {
	return true;
}



} //: namespace InstanceGenerator
} //: namespace Processors
