/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "SOMs2PC.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SOMs2PC {

SOMs2PC::SOMs2PC(const std::string & name) :
		Base::Component(name),
		prop_model_number("model_number", boost::bind(&SOMs2PC::returnSelectedSOMClouds, this), 0)
{
	registerProperty(prop_model_number);
}


SOMs2PC::~SOMs2PC() {
}

void SOMs2PC::prepareInterface() {
	// Register data streams.
	registerStream("in_models", &in_models);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);

	// Register handlers.
	registerHandler("receiveSOMs", boost::bind(&SOMs2PC::receiveSOMs, this));
	addDependency("receiveSOMs", &in_models);

	registerHandler("returnSelectedSOMClouds", boost::bind(&SOMs2PC::returnSelectedSOMClouds, this));

}

bool SOMs2PC::onInit() {

	return true;
}

bool SOMs2PC::onFinish() {
	return true;
}

bool SOMs2PC::onStop() {
	return true;
}

bool SOMs2PC::onStart() {
	return true;
}

void SOMs2PC::returnSelectedSOMClouds() {
	if ((prop_model_number >= models.size()) || (prop_model_number <0)){
		CLOG(LERROR) << "Model number "<<prop_model_number<<" exceeds the models size ("<<models.size()<<")";
		return;
		}

	// Write clouds of the selected model to output ports.
	out_cloud_xyzrgb.write(models[prop_model_number]->cloud_xyzrgb);
	out_cloud_xyzsift.write(models[prop_model_number]->cloud_xyzsift);
	CLOG(LTRACE) << "Model" << prop_model_number << " returned";
}

void SOMs2PC::receiveSOMs() {
	CLOG(LTRACE) << "SOMs2PC::ReturnSOMClouds";

	// Delete old models.
	for( int i = 0 ; i<models.size(); i++){
		delete models[i];
	}
	models.clear();

	// Load new list of models.
	std::vector<AbstractObject*> abstractObjects = in_models.read();
	for( int i = 0 ; i<abstractObjects.size(); i++){
        CLOG(LTRACE)<<"Name: "<<abstractObjects[i]->name<<endl;
		SIFTObjectModel *model = dynamic_cast<SIFTObjectModel*>(abstractObjects[i]);
		if(model!=NULL)
			models.push_back(model);
		else
            CLOG(LWARNING) << "Improper model type!" << endl;
	}
    CLOG(LTRACE) << "Received " <<models.size() << " models" << endl;

    //
    returnSelectedSOMClouds();
}

} //: namespace SOMs2PC
} //: namespace Processors
