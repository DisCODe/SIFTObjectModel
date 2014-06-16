/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "SIFTNOMReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SIFTNOMReader {

SIFTNOMReader::SIFTNOMReader(const std::string & name) :
		Base::Component(name) ,
		filenames("filenames", boost::bind(&SIFTNOMReader::onFilenamesChanged, this, _1, _2), "")
		{
			registerProperty(filenames);

		}

SIFTNOMReader::~SIFTNOMReader() {
}

void SIFTNOMReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register handlers
	// Register data streams, events and event handlers HERE!
	registerStream("out_models", &out_models);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);

	// Register handlers
	h_loadModels.setup(boost::bind(&SIFTNOMReader::loadModels, this));
	registerHandler("loadModels", &h_loadModels);

}

bool SIFTNOMReader::onInit() {
	LOG(LTRACE) << "SIFTNOMReader::onInit()";
	// Load models at start.
	loadModels();
	return true;
}

bool SIFTNOMReader::onFinish() {
	return true;
}

bool SIFTNOMReader::onStop() {
	return true;
}

bool SIFTNOMReader::onStart() {
	return true;
}
void SIFTNOMReader::loadModels() {
	LOG(LTRACE) << "SIFTNOMJSONReader::loadModels()";

	// List of the returned SOMs.
	std::vector<AbstractObject*> models;

	// Names of models/JSON files.
	std::vector<std::string> namesList;
	std::string s= filenames;
	boost::split(namesList, s, boost::is_any_of(";"));

	// Temporary variables - names.
	std::string name_cloud_xyzsift;
	std::string name_cloud_xyzrgbnormal;

	// Iterate through JSON files.
	for (size_t i = 0; i < namesList.size(); i++){
		ptree ptree_file;
		try{
			// Open JSON file and load it to ptree.
			read_json(namesList[i], ptree_file);
			// Read JSON properties.
			model_name = ptree_file.get<std::string>("name");
			mean_viewpoint_features_number = ptree_file.get<int>("mean_viewpoint_features_number");
			name_cloud_xyzsift = ptree_file.get<std::string>("cloud_xyzsift");
			name_cloud_xyzrgbnormal= ptree_file.get<std::string>("cloud_xyzrgb_normals");
		}//: try
		catch(std::exception const& e){
			LOG(LERROR) << "SOMJSONReader: file "<< namesList[i] <<" not found or invalid. " << e.what();
			continue;
		}//: catch


		LOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
		LOG(LDEBUG) << "name_cloud_xyzrgbnormal:" << name_cloud_xyzrgbnormal;

		// Read XYZRGBNormal cloud.
		cloud_xyzrgb_normals = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (name_cloud_xyzrgbnormal, *cloud_xyzrgb_normals) == -1)
		{
			LOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzrgbnormal <<" not found\n";
			continue;
		}//: if


		// Create SOModel and add it to list.
		SIFTObjectModel* model;
		model = dynamic_cast<SIFTObjectModel*>(produce());
		models.push_back(model);


	}//: for

	// Push models to output datastream.
	out_models.write(models);
	out_cloud_xyzrgb_normals.write(cloud_xyzrgb_normals);
}


void SIFTNOMReader::onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames) {
	filenames = new_filenames;
	CLOG(LTRACE) << "onFilenamesChanged: " << std::string(filenames) << std::endl;
}



} //: namespace SIFTNOMReader
} //: namespace Processors
