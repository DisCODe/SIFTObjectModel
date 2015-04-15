#include <memory>
#include <string>

#include "VectorToSequence.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace VectorToSequence {

VectorToSequence::VectorToSequence(const std::string & name) :
		Base::Component(name) ,
		place(0)
{
}

VectorToSequence::~VectorToSequence() {
}

void VectorToSequence::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_trigger", &in_trigger);
	registerStream("in_models", &in_models);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	// Register handlers
	h_throwClouds.setup(boost::bind(&VectorToSequence::throwClouds, this));

	h_loadModels.setup(boost::bind(&VectorToSequence::loadModels, this));
	registerHandler("loadModels", &h_loadModels);
	registerHandler("throwClouds", &h_throwClouds);
  	addDependency("loadModels", &in_models);
	addDependency("throwClouds",&in_trigger);

}

bool VectorToSequence::onInit() {
	LOG(LTRACE) << "VectorToSequence::onInit()";
	return true;
}

bool VectorToSequence::onFinish() {
	return true;
}

bool VectorToSequence::onStop() {
	return true;
}

bool VectorToSequence::onStart() {
	return true;
}

void VectorToSequence::loadModels() {
	LOG(LTRACE) << "VectorToSequence::loadModels()";

	// List of the returned SOMs.
	models = in_models.read();
	
	if (models.empty()) {
		LOG(LERROR) << "VectorToSequence: vector is empty\n";
		return;
	}

}

void VectorToSequence::throwClouds() {
	LOG(LTRACE) << "VectorToSequence::throwClouds()";
	if (models.empty())
		return;
	in_trigger.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());

	place = place % models.size();
	SIFTObjectModel* model = dynamic_cast<SIFTObjectModel*>(models[place]);
	place++;

	*cloudrgb = *(model->cloud_xyzrgb);
	*cloud_sift = *(model->cloud_xyzsift);
	// Push clouds to output datastream.
	out_cloud_xyzrgb.write(cloudrgb);
	out_cloud_xyzsift.write(cloud_sift);
}


} //: namespace VectorToSequence
} //: namespace Processors
