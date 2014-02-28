/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "Update.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
///////////////////////////////////////////
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/search/kdtree.h" 
#include "pcl/search/impl/kdtree.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
////////////////////////////////////////////////////////////////////////
#include <pcl/filters/filter.h>

#include<pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>



namespace Processors {
namespace Update {



//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

/*
 * \brief Class used for transformation from SIFT descriptor to array of floats.
 */
class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation <PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
	/// Templatiated number of SIFT descriptor dimensions.
	using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;

	public:
	SIFTFeatureRepresentation ()
	{
		// Define the number of dimensions.
		nr_dimensions_ = 128 ;
		trivial_ = false ;
	}

	/// Overrides the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointXYZSIFT &p, float * out) const
	{
		//This representation is only for determining correspondences (not for use in Kd-tree for example - so use only the SIFT part of the point.
		for (register int i = 0; i < 128 ; i++)
			out[i] = p.descriptor[i];//p.descriptor.at<float>(0, i) ;
	}
};


Eigen::Matrix4f Update::computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg, 
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers)
{
	CLOG(LTRACE) << "Computing SAC" << std::endl ;	

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
	sac.setInputSource(cloud_src) ;
	sac.setInputTarget(cloud_trg) ;
	sac.setInlierThreshold(0.001f) ;
	sac.setMaximumIterations(2000) ;
	sac.setInputCorrespondences(correspondences) ;
	sac.getCorrespondences(inliers) ;

	CLOG(LINFO) << "SAC inliers " << inliers.size();

	if ( ((float)inliers.size()/(float)correspondences->size()) >85)
		return Eigen::Matrix4f::Identity();
	return sac.getBestTransformation() ;
}

Update::Update(const std::string & name) :
		Base::Component(name)  {

}

Update::~Update() {
}

void Update::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);
	// Register single handler - the "Update" function.
	h_update.setup(boost::bind(&Update::update, this));
	registerHandler("update", &h_update);
	addDependency("update", &in_cloud_xyzsift);
	addDependency("update", &in_cloud_xyzrgb);

}

bool Update::onInit() {
	// Number of viewpoints.
	counter = 0;
	// Mean number of features per view. 
	mean_viewpoint_features_number = 0;
		
	global_trans = Eigen::Matrix4f::Identity();

	cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_merged = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
/*
	cloud_prev = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_next = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_prev = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	cloud_sift_next = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());	
*/
	
	return true;
}

bool Update::onFinish() {
	return true;
}

bool Update::onStop() {
	return true;
}

bool Update::onStart() {
	return true;
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, bool downsample = false) // PointCloud::Ptr output, 
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
//    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
//  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

/*  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");*/

  //add the source to the transformed target
//  *output += *cloud_src;
  
  final_transform = targetToSource;
 }


void Update::update() {
	CLOG(LTRACE) << "Update::update";
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();

	// TODO if empty()

	CLOG(LDEBUG) << "cloud_xyzrgb size: "<<cloud->size();
	CLOG(LDEBUG) << "cloud_xyzsift size: "<<cloud_sift->size();

	// Remove NaNs.
	std::vector<int> indices;
	cloud->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);	
	cloud_sift->is_dense = false; 
	pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);	

	CLOG(LDEBUG) << "cloud_xyzrgb size without NaN: "<<cloud->size();
	CLOG(LDEBUG) << "cloud_xyzsift size without NaN: "<<cloud_sift->size();

	CLOG(LINFO) << "view number: "<<counter;
	CLOG(LINFO) << "view cloud->size(): "<<cloud->size();
	CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift->size();

	// First cloud.
	if (counter == 0 ){
		*cloud_merged = *cloud;
		*cloud_sift_merged = *cloud_sift;

		counter++;
		mean_viewpoint_features_number = cloud_sift->size();
		// Push results to output data ports.
		out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);

		// Push SOM - depricated.
//		out_instance.write(produce());	
		return;
	}

	// Update view count and feature numbers.
	counter++;
	total_viewpoint_features_number += cloud_sift->size();
	
	// Find corespondences between feature clouds.
	// Initialize parameters.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;
	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
	correst.setPointRepresentation(point_representation) ;
	correst.setInputSource(cloud_sift) ;
	correst.setInputTarget(cloud_sift_merged) ;
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences) ;
	CLOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_sift->size() << " features";

	// Computate multiplicity of features (designating how many multiplicity given feature appears in all views).
	for(int i = 0; i< correspondences->size();i++){	
		if (correspondences->at(i).index_query >=cloud_sift->size() || correspondences->at(i).index_match >=cloud_sift_merged->size()){
			continue;
		}
		// 		
		cloud_sift->at(correspondences->at(i).index_query).multiplicity = cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity + 1;
		cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity=-1;
	}

	//displayCorrespondences(cloud_next, cloud_sift_next, cloud_prev, cloud_sift_prev, correspondences, viewer) ;

	// Compute transformation between clouds and update global transformation of cloud.
	pcl::Correspondences inliers;
	Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers) ;
	if (current_trans == Eigen::Matrix4f::Identity()){
		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);
		return;
	}//: if
	CLOG(LINFO) << "SAC Transformation from current view cloud to cloud_merged: " << std::endl << current_trans;
/*	global_trans = global_trans * current_trans;
	CLOG(LINFO) << "Transformation from current view cloud to first view: " << std::endl << global_trans << std::endl ;
*/

	// Delete points.
	pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift_merged->begin();
	while(pt_iter!=cloud_sift_merged->end()){
		if(pt_iter->multiplicity==-1){
			pt_iter = cloud_sift_merged->erase(pt_iter);
		} else {
			++pt_iter;	
		}
	}

	// Transform both view clouds.
/*	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_merge;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift_to_merge;
	cloud_to_merge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_to_merge = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());*/

	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

	// Use ICP to get "better" transformation.
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

//	max_correspondence_distance_ (1.0*1.0),
//	nr_iterations_ (500)
//	normal_radius_ (0.2),
//	feature_radius_ (0.2) 

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.005);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);

	icp.setInputSource(cloud_merged);
	icp.setInputTarget(cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
	icp.align(*Final);
	CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

	// Get the transformation from target to source.
	current_trans = icp.getFinalTransformation().inverse();
	CLOG(LINFO) << "ICP transformation refinement: " << std::endl << current_trans;

	// Refine the 	
	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

/*
	Eigen::Matrix4f icp_trans;
	pairAlign (cloud_merged, cloud, icp_trans, false); 
	CLOG(LINFO) << "ICP transformation refinement: " << std::endl << icp_trans;
*/
	//addCloudToScene(cloud_to_merge, sceneviewer, counter - 1) ; 

	// Add clouds.
	*cloud_merged += *cloud;
	*cloud_sift_merged += *cloud_sift;

	CLOG(LINFO) << "model cloud->size(): "<<cloud_merged->size();
	CLOG(LINFO) << "model cloud_sift->size(): "<<cloud_sift_merged->size();


	// Compute mean number of features.
	mean_viewpoint_features_number = total_viewpoint_features_number/counter;

	// Push results to output data ports.
	out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
	out_cloud_xyzrgb.write(cloud_merged);
	out_cloud_xyzsift.write(cloud_sift_merged);

	// Push SOM - depricated.
//	out_instance.write(produce());	
}



} //: namespace Update
} //: namespace Processors
