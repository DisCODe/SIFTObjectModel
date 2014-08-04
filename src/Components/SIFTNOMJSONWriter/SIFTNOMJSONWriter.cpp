/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "SIFTNOMJSONWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SIFTNOMJSONWriter {

SIFTNOMJSONWriter::SIFTNOMJSONWriter(const std::string & name) :
		Base::Component(name),
		dir("directory", boost::bind(&SIFTNOMJSONWriter::onDirChanged, this, _1, _2), "./"),
		SOMname("SOM", boost::bind(&SIFTNOMJSONWriter::onSOMNameChanged, this, _1, _2), "SOM")
		{
			CLOG(LTRACE) << "Hello SIFTNOMJSONWriter\n";
			registerProperty(SOMname);
			registerProperty(dir);
		}


SIFTNOMJSONWriter::~SIFTNOMJSONWriter() {
}

void SIFTNOMJSONWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_som", &in_som);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_mean_viewpoint_features_number", &in_mean_viewpoint_features_number);

	// Register handlers
	h_Write_normals.setup(boost::bind(&SIFTNOMJSONWriter::Write_normals, this));
	registerHandler("Write_normals", &h_Write_normals);
}

bool SIFTNOMJSONWriter::onInit() {

	return true;
}

void SIFTNOMJSONWriter::onSOMNameChanged(const std::string & old_SOMname, const std::string & new_SOMname) {
	SOMname = new_SOMname;
	CLOG(LTRACE) << "onSOMNameChanged: " << std::string(SOMname) << std::endl;
}

void SIFTNOMJSONWriter::onDirChanged(const std::string & old_dir,
		const std::string & new_dir) {
	dir = new_dir;
	CLOG(LTRACE) << "onDirChanged: " << std::string(dir) << std::endl;
}


bool SIFTNOMJSONWriter::onFinish() {
	return true;
}

bool SIFTNOMJSONWriter::onStop() {
	return true;
}

bool SIFTNOMJSONWriter::onStart() {
	return true;
}

void SIFTNOMJSONWriter::Write_normals() {

	LOG(LTRACE) << "SIFTNOMJSONWriter::Write";
	// Try to save the model retrieved from the SOM data stream.
	ptree ptree_file;

	//if(in_cloud_xyzrgb_normals.empty()&&in_cloud_xyzsift.empty()&&in_mean_viewpoint_features_number.empty()){
	//	CLOG(LWARNING) << "There are no required datastreams enabling save of the SOM to file.";
	//	return;
	//}

	if (!in_som.empty()) {
		LOG(LDEBUG) << "!in_som.empty()";

		// Get SOM.
		SIFTObjectModel* som = in_som.read();

		// Save point cloud.
		std::string name_cloud_xyzrgb_normals = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzrgb_normals.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzrgb_normals, *(som->cloud_xyzrgb_normals));
		CLOG(LTRACE) << "Write: saved " << som->cloud_xyzrgb_normals->points.size () << " cloud points to "<< name_cloud_xyzrgb_normals;

		// Save feature cloud.
		std::string name_cloud_xyzsift = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzsift.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzsift, *(som->cloud_xyzsift));
		CLOG(LTRACE) << "Write: saved " << som->cloud_xyzsift->points.size () << " feature points to "<< name_cloud_xyzsift;

		// Save JSON model description.
		ptree_file.put("name", SOMname);
		ptree_file.put("type", "SIFTObjectModel");
		ptree_file.put("mean_viewpoint_features_number", som->mean_viewpoint_features_number);
		ptree_file.put("cloud_xyzsift", name_cloud_xyzsift);

	}

	// Try to save the model retrieved from the three separate data streams.
	if (!in_cloud_xyzsift.empty() && !in_mean_viewpoint_features_number.empty()) {
		LOG(LDEBUG) << "!in_cloud_xyzsift.empty() && !in_mean_viewpoint_features_number.empty()";

		// Get model from datastreams.

		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
		int mean_viewpoint_features_number = in_mean_viewpoint_features_number.read();


		// Save feature cloud.
		std::string name_cloud_xyzsift = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzsift.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzsift, *(cloud_xyzsift));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzsift->points.size () << " feature points to "<< name_cloud_xyzsift;

		// Save JSON model description.

		ptree_file.put("name", SOMname);
		ptree_file.put("type", "SIFTObjectModel");
		ptree_file.put("mean_viewpoint_features_number", mean_viewpoint_features_number);
		ptree_file.put("cloud_xyzsift", name_cloud_xyzsift);


	}

	if (!in_cloud_xyzrgb_normals.empty()) {
		LOG(LDEBUG) << "!in_cloud_xyzrgb_normals.empty()";

		// Get model from datastreams.
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgb_normals = in_cloud_xyzrgb_normals.read();


		// Save point cloud.
		std::string name_cloud_xyzrgb_normals = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzrgb_normals.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzrgb_normals, *(cloud_xyzrgb_normals));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzrgb_normals->points.size () << " cloud points to "<< name_cloud_xyzrgb_normals;


		// Save JSON model description.
		//ptree ptree_file;
		ptree_file.put("name", SOMname);
		ptree_file.put("type", "SIFTObjectModel");

		ptree_file.put("cloud_xyzrgb_normals", name_cloud_xyzrgb_normals);
	}

	write_json (std::string(dir) + std::string("/") + std::string(SOMname) + std::string(".json"), ptree_file);


}


} //: namespace SIFTNOMJSONWriter
} //: namespace Processors
