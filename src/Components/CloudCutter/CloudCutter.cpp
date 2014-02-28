/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "CloudCutter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace Processors {
namespace CloudCutter {

CloudCutter::CloudCutter(const std::string & name) :
		Base::Component(name) , 
		radius("radius", 0) {
		registerProperty(radius);

}

CloudCutter::~CloudCutter() {
}

void CloudCutter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud", &in_cloud);
registerStream("in_indices", &in_indices);
registerStream("out_cloud", &out_cloud);
	// Register handlers
	h_cut.setup(boost::bind(&CloudCutter::cut, this));
	registerHandler("cut", &h_cut);
	addDependency("cut", &in_cloud);
	addDependency("cut", &in_indices);

}

bool CloudCutter::onInit() {

	return true;
}

bool CloudCutter::onFinish() {
	return true;
}

bool CloudCutter::onStop() {
	return true;
}

bool CloudCutter::onStart() {
	return true;
}

void CloudCutter::cut() {
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr indices = in_indices.read();
	
	pcl::KdTreeFLANN<PointXYZSIFT> kdtree; 
	kdtree.setInputCloud (cloud); 
	
	for(int i=0; i<indices->size();i++){
		pcl::PointXYZ searchPoint = indices->points[i]; 
		PointXYZSIFT searchPoint_xyzsift;
		searchPoint_xyzsift.x = searchPoint.x;
		searchPoint_xyzsift.y = searchPoint.y;
		searchPoint_xyzsift.z = searchPoint.z;
		
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if ( kdtree.radiusSearch (searchPoint_xyzsift, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) 
        { 
                cout<<"Znaleziono " <<pointIdxRadiusSearch.size() << " punktów"<<endl;
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
					cloud->erase(cloud->begin() + pointIdxRadiusSearch[i] -1);//-1??
				}
                
                
        } 
	}

}



} //: namespace CloudCutter
} //: namespace Processors
