/*!
 * \file
 * \brief
 * \author tkornuta,,,
 */

#include <memory>
#include <string>

#include "SOMJSONWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SOMJSONWriter {

SOMJSONWriter::SOMJSONWriter(const std::string & name) :
		Base::Component(name),
		dir("directory", boost::bind(&SOMJSONWriter::onDirChanged, this, _1, _2), "./"),
		SOMname("SOM", boost::bind(&SOMJSONWriter::onSOMNameChanged, this, _1, _2), "SOM")
{
	CLOG(LTRACE) << "Hello SOMJSONWriter\n";
	registerProperty(SOMname);
	registerProperty(dir);
}


SOMJSONWriter::~SOMJSONWriter() {
	CLOG(LTRACE) << "Bye SOMJSONWriter\n";
}

void SOMJSONWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_som", &in_som);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_mean_viewpoint_features_number", &in_mean_viewpoint_features_number);

	// Register handlers
	h_Write.setup(boost::bind(&SOMJSONWriter::Write, this));
	registerHandler("Write", &h_Write);
}

bool SOMJSONWriter::onInit() {

	return true;
}

bool SOMJSONWriter::onFinish() {
	return true;
}

bool SOMJSONWriter::onStop() {
	return true;
}

bool SOMJSONWriter::onStart() {
	return true;
}

void SOMJSONWriter::onSOMNameChanged(const std::string & old_SOMname, const std::string & new_SOMname) {
	SOMname = new_SOMname;
	CLOG(LTRACE) << "onSOMNameChanged: " << std::string(SOMname) << std::endl;
}

void SOMJSONWriter::onDirChanged(const std::string & old_dir,
		const std::string & new_dir) {
	dir = new_dir;
	CLOG(LTRACE) << "onDirChanged: " << std::string(dir) << std::endl;
}


void SOMJSONWriter::Write() {
	LOG(LTRACE) << "SOMJSONWriter::Write";
	// Try to save the model retrieved from the SOM data stream.
	if (!in_som.empty()) {
		LOG(LDEBUG) << "!in_som.empty()";

		// Get SOM.
		SIFTObjectModel* som = in_som.read();

		// Save point cloud.
		std::string name_cloud_xyzrgb = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzrgb.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzrgb, *(som->cloud_xyzrgb));
		CLOG(LTRACE) << "Write: saved " << som->cloud_xyzrgb->points.size () << " cloud points to "<< name_cloud_xyzrgb;

		// Save feature cloud.
		std::string name_cloud_xyzsift = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzsift.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzsift, *(som->cloud_xyzsift));
		CLOG(LTRACE) << "Write: saved " << som->cloud_xyzsift->points.size () << " feature points to "<< name_cloud_xyzsift;

		// Save JSON model description.
		ptree ptree_file;
		ptree_file.put("name", SOMname);
		ptree_file.put("type", "SIFTObjectModel");
		ptree_file.put("mean_viewpoint_features_number", som->mean_viewpoint_features_number);
		ptree_file.put("cloud_xyzrgb", name_cloud_xyzrgb);
		ptree_file.put("cloud_xyzsift", name_cloud_xyzsift);
		write_json (std::string(dir) + std::string("/") + std::string(SOMname) + std::string(".json"), ptree_file);
		return;
	}

	// Try to save the model retrieved from the three separate data streams.
	if (!in_cloud_xyzrgb.empty() && !in_cloud_xyzsift.empty() && !in_mean_viewpoint_features_number.empty()) {
		LOG(LDEBUG) << "!in_cloud_xyzrgb.empty() && !in_cloud_xyzsift.empty() && !in_mean_viewpoint_features_number.empty()";

		// Get model from datastreams.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
		int mean_viewpoint_features_number = in_mean_viewpoint_features_number.read();

		// Save point cloud.
		std::string name_cloud_xyzrgb = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzrgb.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzrgb, *(cloud_xyzrgb));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzrgb->points.size () << " cloud points to "<< name_cloud_xyzrgb;

		// Save feature cloud.
		std::string name_cloud_xyzsift = std::string(dir) + std::string("/") + std::string(SOMname) + std::string("_xyzsift.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzsift, *(cloud_xyzsift));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzsift->points.size () << " feature points to "<< name_cloud_xyzsift;

		// Save JSON model description.
		ptree ptree_file;
		ptree_file.put("name", SOMname);
		ptree_file.put("type", "SIFTObjectModel");
		ptree_file.put("mean_viewpoint_features_number", mean_viewpoint_features_number);
		ptree_file.put("cloud_xyzrgb", name_cloud_xyzrgb);
		ptree_file.put("cloud_xyzsift", name_cloud_xyzsift);
		write_json (std::string(dir) + std::string("/") + std::string(SOMname) + std::string(".json"), ptree_file);
		std::cout<<"Rozmiar zapisanej chmury: "<<cloud_xyzrgb->points.size()<<std::endl;
		return;
	}
	

	CLOG(LWARNING) << "There are no required datastreams enabling save of the SOM to file.";
} 


} //: namespace SOMJSONWriter
} //: namespace Processors
