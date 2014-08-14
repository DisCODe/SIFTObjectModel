/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "PCDWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PCDWriter {

PCDWriter::PCDWriter(const std::string & name) :
		Base::Component(name) , 
    filename("filename", std::string(".") ) {
	registerProperty(filename);

}

PCDWriter::~PCDWriter() {
}

void PCDWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	// Register handlers
	h_Write.setup(boost::bind(&PCDWriter::Write, this));
    registerHandler("Write", &h_Write);
}

bool PCDWriter::onInit() {

	return true;
}

bool PCDWriter::onFinish() {
	return true;
}

bool PCDWriter::onStop() {
	return true;
}

bool PCDWriter::onStart() {
	return true;
}

void PCDWriter::Write() {
    LOG(LTRACE) << "PCDWriter::Write_xyzrgb";
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
    pcl::io::savePCDFileASCII (filename, *cloud);
    LOG(LINFO) << "Saved " << cloud->points.size () << " data points to "<< filename << std::endl;

}



} //: namespace PCDWriter
} //: namespace Processors
