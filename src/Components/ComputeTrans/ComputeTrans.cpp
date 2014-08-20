/*!
 * \file
 * \brief
 * \author Mikolaj Kamionka
 */

#include <memory>
#include <string>

#include "ComputeTrans.hpp"

#include <Types/MergeUtils.hpp>

namespace Processors {
namespace ComputeTrans {

ComputeTrans::ComputeTrans(const std::string & name) :
		Base::Component(name),
		first(false),
		cloud_old(new pcl::PointCloud<pcl::PointXYZRGB>()) {
}

ComputeTrans::~ComputeTrans() {
}

void ComputeTrans::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_sift", &in_cloud_xyzsift);

	// Register handlers
	h_compute.setup(this, &ComputeTrans::compute);
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud_xyzsift);
}

bool ComputeTrans::onInit() {
	return true;
}

bool ComputeTrans::onFinish() {
	return true;
}

bool ComputeTrans::onStop() {
	return true;
}

bool ComputeTrans::onStart() {
	return true;
}

void ComputeTrans::compute() {
	CLOG(LDEBUG)<<"compute()";

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();
	
	if

}

} //: namespace CalcStatistics
} //: namespace Processors
