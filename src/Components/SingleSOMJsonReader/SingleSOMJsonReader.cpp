/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SingleSOMJsonReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SingleSOMJsonReader {

SingleSOMJsonReader::SingleSOMJsonReader(const std::string & name) :
				Base::Component(name), filenames("filenames", string("./")) {
			registerProperty(filenames);

}

SingleSOMJsonReader::~SingleSOMJsonReader() {
}

void SingleSOMJsonReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register data streams, events and event handlers HERE!
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_name", &out_name);
	// Register handlers
	h_loadModel.setup(boost::bind(&SingleSOMJsonReader::loadModel, this));
	registerHandler("loadModel", &h_loadModel);
	addDependency("loadModel", NULL);
}

bool SingleSOMJsonReader::onInit() {

	return true;
}

bool SingleSOMJsonReader::onFinish() {
	return true;
}

bool SingleSOMJsonReader::onStop() {
	return true;
}

bool SingleSOMJsonReader::onStart() {
	return true;
}

void SingleSOMJsonReader::loadModel() {
	LOG(LTRACE) << "SingleSOMJsonReader::loadModel()";

	std::string s = filenames;

	// Temporary variables - names.
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;

	// Iterate through JSON files.

	ptree ptree_file;
	try {
		// Open JSON file and load it to ptree.
		read_json(s, ptree_file);
		// Read JSON properties.
		model_name = ptree_file.get < std::string > ("name");
		mean_viewpoint_features_number = ptree_file.get<int>(
				"mean_viewpoint_features_number");
		name_cloud_xyzrgb = ptree_file.get < std::string > ("cloud_xyzrgb");
		name_cloud_xyzsift = ptree_file.get < std::string > ("cloud_xyzsift");
	} //: try
	catch (std::exception const& e) {
		LOG(LERROR) << "SingleSOMJsonReader: file " << s
				<< " not found or invalid\n";
		return;
	} //: catch

	LOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
	LOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;

	// Read XYZRGB cloud.
	cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_cloud_xyzrgb, *cloud_xyzrgb)
			== -1) {
		LOG(LERROR) << "SingleSOMJsonReader: file " << name_cloud_xyzrgb
				<< " not found\n";
		return;
	} //: if

	// Read SIFT cloud.
	cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr(
			new pcl::PointCloud<PointXYZSIFT>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<PointXYZSIFT>(name_cloud_xyzsift, *cloud_xyzsift)
			== -1) {
		LOG(LERROR) << "SingleSOMJsonReader: file " << name_cloud_xyzsift
				<< " not found\n";
		return;
	} //: if


	out_cloud_xyzrgb.write(cloud_xyzrgb);
	out_cloud_xyzsift.write(cloud_xyzsift);
	out_name.write(model_name);


}


} //: namespace SingleSOMJsonReader
} //: namespace Processors
