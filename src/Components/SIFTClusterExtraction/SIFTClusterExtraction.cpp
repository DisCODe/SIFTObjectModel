/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SIFTClusterExtraction.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/search/impl/search.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/point_representation.h>

namespace Processors {
namespace SIFTClusterExtraction {


SIFTClusterExtraction::SIFTClusterExtraction(const std::string & name) :
		Base::Component(name) , 
		clusterTolerance("clusterTolerance", 0.05), 
		minClusterSize("minClusterSize", 25), 
		maxClusterSize("maxClusterSize", 5000) {
		registerProperty(clusterTolerance);
		registerProperty(minClusterSize);
		registerProperty(maxClusterSize);

}

SIFTClusterExtraction::~SIFTClusterExtraction() {
}

void SIFTClusterExtraction::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("out_clusters", &out_clusters);
	registerStream("out_clusters_xyz", &out_clusters_xyz);
	// Register handlers
	h_extract.setup(boost::bind(&SIFTClusterExtraction::extract, this));
	registerHandler("extract", &h_extract);
	addDependency("extract", &in_cloud_xyzsift);

}

bool SIFTClusterExtraction::onInit() {

	return true;
}

bool SIFTClusterExtraction::onFinish() {
	return true;
}

bool SIFTClusterExtraction::onStop() {
	return true;
}

bool SIFTClusterExtraction::onStart() {
	return true;
}

void SIFTClusterExtraction::extract() {
  LOG(LTRACE) <<"SIFTClusterExtraction::extract()";
  
  
  pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointXYZSIFT>::Ptr tree (new pcl::search::KdTree<PointXYZSIFT>);
  tree->setInputCloud (cloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZSIFT> ec;
  ec.setClusterTolerance (clusterTolerance); // 2cm
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> clusters;
  
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud_cluster (new pcl::PointCloud<PointXYZSIFT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

	clusters.push_back(cloud_cluster);
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
  }	
	
	out_clusters.write(clusters);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>); 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_xyz;
	for(int i = 0; i < clusters.size(); i++){
		pcl::copyPointCloud(*clusters[i],*cloud_xyz);
		clusters_xyz.push_back(cloud_xyz);
	}
	out_clusters_xyz.write(clusters_xyz);

}

} //: namespace SIFTClusterExtraction
} //: namespace Processors
