/*!
 * \file
 * \brief
 * \author tkornuta,,,
 */

#include <memory>
#include <string>

#include "SOMJSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <pcl/common/common.h>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SOMJSONReader {

std::string dirnameOf(const std::string& fname)
{
     size_t pos = fname.find_last_of("\\/");
     return (std::string::npos == pos)
         ? ""
         : fname.substr(0, pos);
}

SOMJSONReader::SOMJSONReader(const std::string & name) :
	Base::Component(name),
	prop_load_on_init("load_on_init", true),
	prop_filenames("filenames", std::string(""))
{
	registerProperty(prop_filenames);
	registerProperty(prop_load_on_init);

}

SOMJSONReader::~SOMJSONReader() {
}


void SOMJSONReader::prepareInterface() {
	// Register data streams.
	registerStream("out_models", &out_models);
	registerStream("out_model_ids", &out_model_ids);
	registerStream("out_model_clouds_xyzrgb", &out_model_clouds_xyzrgb);
	registerStream("out_model_clouds_xyzsift", &out_model_clouds_xyzsift);
	registerStream("out_model_corners_xyz", &out_model_corners_xyz);

	// Register manually triggered handlers.
	registerHandler("loadModelsButtonPressed", boost::bind(&SOMJSONReader::loadModelsButtonPressed, this));
	registerHandler("clearModelsButtonPressed", boost::bind(&SOMJSONReader::clearModelsButtonPressed, this));

	// Register handler - unconditioned model publication
	registerHandler("publishModels", boost::bind(&SOMJSONReader::publishModels, this));
	addDependency("publishModels", NULL);
}


bool SOMJSONReader::onInit() {
	CLOG(LTRACE) << "onInit()";

	// Set flags.
	clear_models = false;

	// Load models at start - if property set.
	load_models = prop_load_on_init;

	return true;
}


bool SOMJSONReader::onFinish() {
	return true;
}


bool SOMJSONReader::onStop() {
	return true;
}


bool SOMJSONReader::onStart() {
	return true;
}


void SOMJSONReader::publishModels() {
	CLOG(LTRACE) << "publishModels()";

	// Check user input.
	if (load_models)
		loadModels();

	if (clear_models)
		clearModels();

	CLOG(LDEBUG) << "Publishing " << models.size() << " SIFT Object Models";

	// Push models to output datastream -- DEPRICATED.
	out_models.write(models);
	
	// New: dataports containing simple types.
	out_model_ids.write(model_names);
	out_model_clouds_xyzrgb.write(model_clouds_xyzrgb);
	out_model_clouds_xyzsift.write(model_clouds_xyzsift);
	out_model_corners_xyz.write(model_corners_xyz);
}


void SOMJSONReader::loadModels() {
	CLOG(LTRACE) << "loadModels()";

	load_models = false;

	// DEPRICATED.
	models.clear();

	// New: dataports containing simple types.
	model_names.clear();
	model_clouds_xyzrgb.clear();
	model_clouds_xyzsift.clear();
	model_corners_xyz.clear();

	// Names of models/JSON files.	
	std::vector<std::string> namesList;
	std::string s= prop_filenames;
	boost::split(namesList, s, boost::is_any_of(";"));

	// Temporary variables - names.
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;

	
	// Iterate through JSON files.
	for (size_t i = 0; i < namesList.size(); i++){
		ptree ptree_file;
		try{
			// Open JSON file and load it to ptree.
			read_json(namesList[i], ptree_file);
			// Read JSON properties.
			model_name = ptree_file.get<std::string>("name");
			mean_viewpoint_features_number = ptree_file.get<int>("mean_viewpoint_features_number");

			// Get path to files.
			std::string dir = dirnameOf(namesList[i]);
			if (dir != "")
				dir = dir + "/";

			name_cloud_xyzrgb = dir + ptree_file.get<std::string>("cloud_xyzrgb");
			name_cloud_xyzsift = dir + ptree_file.get<std::string>("cloud_xyzsift");

		}//: try
		catch(std::exception const& e){
			CLOG(LERROR) << "SOMJSONReader: file "<< namesList[i] <<" not found or invalid\n";
			continue;	
		}//: catch
		CLOG(LINFO) << "Loading SOM from JSON file:" << namesList[i];

		CLOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
		CLOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
		

		// Read XYZRGB cloud.
		cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name_cloud_xyzrgb, *cloud_xyzrgb) == -1) 
		{
			CLOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzrgb <<" not found\n";
			continue;
		}//: if

		// Read XYZRGB cloud.
		cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_cloud_xyzsift, *cloud_xyzsift) == -1) 
		{
			CLOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzsift <<" not found\n";
			continue;
		}//: if

		// Generate clouds consisting of eight model corners.
		pcl::PointCloud<pcl::PointXYZ>::Ptr corners_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
		// Concatenate cloud - get only XYZ coordinates.
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		copyPointCloud(*cloud_xyzrgb, *tmp_cloud_xyz);

		pcl::PointXYZ minPt, maxPt, tmpPt;
		pcl::getMinMax3D(*tmp_cloud_xyz, minPt, maxPt);
		// Get width, height and length.
		float dx = maxPt.x - minPt.x;
		float dy = maxPt.y - minPt.y;
		float dz = maxPt.z - minPt.z;
		// Add corners to cloud.
		corners_xyz->push_back(minPt); //0
		tmpPt.x = minPt.x + dx;
		tmpPt.y = minPt.y;
		tmpPt.z = minPt.z;
		corners_xyz->push_back(tmpPt); //1
		tmpPt.x = minPt.x + dx;
		tmpPt.y = minPt.y + dy;
		tmpPt.z = minPt.z;
		corners_xyz->push_back(tmpPt); //2
		tmpPt.x = minPt.x;
		tmpPt.y = minPt.y + dy;
		tmpPt.z = minPt.z;
		corners_xyz->push_back(tmpPt); //3
		tmpPt.x = minPt.x;
		tmpPt.y = minPt.y;
		tmpPt.z = minPt.z + dz;
		corners_xyz->push_back(tmpPt); //4
		tmpPt.x = minPt.x + dx;
		tmpPt.y = minPt.y;
		tmpPt.z = minPt.z + dz;
		corners_xyz->push_back(tmpPt); //5
		corners_xyz->push_back(maxPt); //6
		tmpPt.x = minPt.x;
		tmpPt.y = minPt.y + dy;
		tmpPt.z = minPt.z + dz;
		corners_xyz->push_back(tmpPt); //7



		// Create SOModel and add it to list.
		SIFTObjectModel* model;
		model = dynamic_cast<SIFTObjectModel*>(produce());
		models.push_back(model);

		// Add data (names/clouds) to lists.
		model_names.push_back(model_name);
		model_clouds_xyzrgb.push_back(cloud_xyzrgb);
		model_clouds_xyzsift.push_back(cloud_xyzsift);
		model_corners_xyz.push_back(corners_xyz);

		CLOG(LINFO) << "Properly loaded " << model_name << "SIFT Object Model";

	}//: for

	CLOG(LINFO) << "Loaded " << models.size() << " SIFT Object Models";
}


void SOMJSONReader::clearModels(){
	CLOG(LTRACE) << "clearModels()";

	clear_models = false;

	// DEPRICATED.
	models.clear();
	
	// New: dataports containing simple types.
	model_names.clear();
	model_clouds_xyzrgb.clear();
	model_clouds_xyzsift.clear();

}

void SOMJSONReader::loadModelsButtonPressed(){
	CLOG(LDEBUG) << "loadModelsButtonPressed";
	load_models = true;
}



void SOMJSONReader::clearModelsButtonPressed(){
	CLOG(LDEBUG) << "clearModelsButtonPressed";
	clear_models = true;
}


} //: namespace SOMJSONReader
} //: namespace Processors
