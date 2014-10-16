/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "PC2SOM.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PC2SOM {

PC2SOM::PC2SOM(const std::string & name) :
        Base::Component(name),
        SOMname("SOMname", std::string(" "))
{
        registerProperty(SOMname);
}

PC2SOM::~PC2SOM() {
}

void PC2SOM::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_mean_viewpoint_features_number", &in_mean_viewpoint_features_number);
	registerStream("out_model", &out_model);
	// Register handlers
	h_createSOM.setup(boost::bind(&PC2SOM::createSOM, this));
	registerHandler("createSOM", &h_createSOM);
	addDependency("createSOM", &in_cloud_xyzrgb);
	addDependency("createSOM", &in_cloud_xyzsift);

}

bool PC2SOM::onInit() {

	return true;
}

bool PC2SOM::onFinish() {
	return true;
}

bool PC2SOM::onStop() {
	return true;
}

bool PC2SOM::onStart() {
	return true;
}

void PC2SOM::createSOM() {
    cloud_xyzrgb = in_cloud_xyzrgb.read();
    cloud_xyzsift = in_cloud_xyzsift.read();
    mean_viewpoint_features_number = -1;
    if(!in_mean_viewpoint_features_number.empty())
        mean_viewpoint_features_number = in_mean_viewpoint_features_number.read();
    model_name = SOMname;
    // Create SOModel
    SIFTObjectModel* model;
    model = dynamic_cast<SIFTObjectModel*>(produce());
    out_model.write(model);
}



} //: namespace PC2SOM
} //: namespace Processors
