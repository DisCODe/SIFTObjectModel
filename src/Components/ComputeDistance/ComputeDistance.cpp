/*!
 * \file
 * \brief
 * \author Mikolaj Kamionka
 */

#include <memory>
#include <string>

#include "ComputeDistance.hpp"

#include <Types/MergeUtils.hpp>

namespace Processors {
namespace ComputeDistance {

ComputeDistance::ComputeDistance(const std::string & name) :
		Base::Component(name) {
}

ComputeDistance::~ComputeDistance() {
}

void ComputeDistance::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_sift1", &in_cloud_xyzsift1);
	registerStream("in_cloud_sift2", &in_cloud_xyzsift2);

	registerStream("in_cloud_rgb1", &in_cloud_xyzrgb1);
	registerStream("in_cloud_rgb2", &in_cloud_xyzrgb2);
	
	registerStream("out_distance", &out_distance);

	// Register handlers
	registerHandler("computeSIFTDist", boost::bind(&ComputeDistance::computeSIFTDist, this));
	addDependency("computeSIFTDist", &in_cloud_xyzsift1);
	addDependency("computeSIFTDist", &in_cloud_xyzsift2);

	// Register handlers
	registerHandler("computeRGBDist", boost::bind(&ComputeDistance::computeRGBDist, this));
	addDependency("computeRGBDist", &in_cloud_xyzrgb1);
	addDependency("computeRGBDist", &in_cloud_xyzrgb2);
}

bool ComputeDistance::onInit() {
	return true;
}

bool ComputeDistance::onFinish() {
	return true;
}

bool ComputeDistance::onStop() {
	return true;
}

bool ComputeDistance::onStart() {
	return true;
}

void ComputeDistance::computeSIFTDist() {
	CLOG(LDEBUG)<<"computeSIFTDist()";
	
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_first = in_cloud_xyzsift1.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sec = in_cloud_xyzsift2.read();
	
	// Remove NaNs.
	std::vector<int> indices;
	cloud_first->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_first, *cloud_first, indices);
	cloud_sec->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_sec, *cloud_sec, indices);

	pcl::PointCloud<PointXYZSIFT>::iterator first_iter = cloud_first->begin();
	pcl::PointCloud<PointXYZSIFT>::iterator sec_iter = cloud_sec->begin();

    double distance_sum = 0;
    CLOG(LNOTICE)<<"distances: \n";

	while(first_iter!=cloud_first->end() && sec_iter!=cloud_sec->end()){
        double distance = (first_iter->x - sec_iter->x) * (first_iter->x - sec_iter->x)
			 + (first_iter->y - sec_iter->y) * (first_iter->y - sec_iter->y)
			 + (first_iter->z - sec_iter->z) * (first_iter->z - sec_iter->z);
			 distance = sqrt(distance);
        std::cout<<" " << distance;
        distance_sum += distance;
		first_iter++;
		sec_iter++;
	}

    out_distance.write(distance_sum);
    CLOG(LNOTICE)<<"\ndistance_sum "<< distance_sum;
	
	
}

void ComputeDistance::computeRGBDist() {
	CLOG(LDEBUG)<<"computeRGBDist()";
	CLOG(LFATAL)<<"NOT IMPLEMENTED YET";
}

void ComputeDistance::computeTransDist() {
    CLOG(LDEBUG)<<"computeRGBDist()";
    CLOG(LFATAL)<<"NOT IMPLEMENTED YET";
}

} //: namespace CalcStatistics
} //: namespace Processors
