/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "LUMGenerator.hpp"
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
#include <pcl/registration/lum.h>

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
namespace LUMGenerator {



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


Eigen::Matrix4f LUMGenerator::computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg,
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers)
{
	CLOG(LTRACE) << "Computing SAC" << std::endl ;

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
	sac.setInputSource(cloud_src) ;
	sac.setInputTarget(cloud_trg) ;
	sac.setInlierThreshold(0.01f) ; //property RanSAC
	sac.setMaximumIterations(2000) ; //property RanSAC
	sac.setInputCorrespondences(correspondences) ;
	sac.getCorrespondences(inliers) ;

	//CLOG(LINFO) << "SAC inliers " << inliers.size();


	return sac.getBestTransformation() ;
}


LUMGenerator::LUMGenerator(const std::string & name) :
    Base::Component(name),
    prop_ICP_alignment("ICP.Iterative", false)
{
    registerProperty(prop_ICP_alignment);

}

LUMGenerator::~LUMGenerator() {
}


void LUMGenerator::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);
	//registerStream("out_trigger", &out_Trigger);
	//registerStream("in_trigger", &in_trigger);

    // Register single handler - the "addViewToModel" function.
    h_addViewToModel.setup(boost::bind(&LUMGenerator::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
    addDependency("addViewToModel", &in_cloud_xyzrgb);



   // h_Trigger.setup(Trigger::trigger(),this);
   // registerHandler("Trigger", &h_Trigger);
	//addDependency("Trigger", &h_Trigger);

	//h_Trigger.setup(boost::bind(&LUMGenerator::out_trigger, this));
	//registerHandler("trigger", &h_Trigger);
	//addDependency("trigger", &out_Trigger);
	//addDependency("trigger", &in_trigger);

}

bool LUMGenerator::onInit() {
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

bool LUMGenerator::onFinish() {
	return true;
}

bool LUMGenerator::onStop() {
	return true;
}

bool LUMGenerator::onStart() {
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
  }//in_
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

  reg.setTransformationEpsilon (1e-6); //property ICP

  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  //property ICP
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2); //property ICP
  for (int i = 0; i < 30; ++i) //magiczna liczba 30 :D
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


void LUMGenerator::addViewToModel() {
    CLOG(LTRACE) << "LUMGenerator::addViewToModel";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();

	// TODO if empty()

	CLOG(LINFO) << "cloud_xyzrgb size: "<<cloud->size();
	CLOG(LINFO) << "cloud_xyzsift size: "<<cloud_sift->size();

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

	rgb_views.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>()));
	CLOG(LINFO) << "rgb views size : " << rgb_views.size();
	*(rgb_views[counter]) = *cloud;
	CLOG(LINFO) << "cos: ";
    lum_sift.addPointCloud(cloud_sift);

	// First cloud.
	if (counter == 0 ){
		CLOG(LINFO) << "counter 0: ";
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
		CLOG(LINFO) << "return ";
		return;
	}

    // LUMGenerator view count and feature numbers.
	counter++;
	total_viewpoint_features_number += cloud_sift->size();

	// Find corespondences between feature clouds.
	// Initialize parameters.
	for (int i = counter - 2; i >= 0; i--)
	{
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
		pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;
		SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
		correst.setPointRepresentation(point_representation) ;
		correst.setInputSource(cloud_sift);
		correst.setInputTarget(lum_sift.getPointCloud(i));
		// Find correspondences.
		correst.determineReciprocalCorrespondences(*correspondences) ;
		//CLOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_sift->size() << " features";
		if (correspondences->size() > 2) {
			lum_sift.setCorrespondences(counter-1, i, correspondences);
			//break;
			//CLOG(LINFO) << "added";
		}
	}

	CLOG(LINFO) << "compute";
	//lum_rgb.compute();
	lum_sift.compute();
	CLOG(LINFO) << "ended";

	cloud_sift_merged = lum_sift.getConcatenatedCloud ();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (int i = 0; i < counter-1; i++) {
		*cloud_tmp = *rgb_views[i];
		Eigen::Matrix4f m = lum_sift.getTransformation(i).matrix();
		pcl::transformPointCloud(*cloud_tmp, *cloud_tmp, m);
		CLOG(LINFO) << "LUM Transformation " << i << " : "<< std::endl << m;
		if(i)
			*cloud_merged += *cloud_tmp;
		else
			*cloud_merged = *cloud_tmp;
	}
//
	CLOG(LINFO) << "model cloud->size(): "<< cloud_merged->size();
	CLOG(LINFO) << "model cloud_sift->size(): "<< cloud_sift_merged->size();


	// Compute mean number of features.
	mean_viewpoint_features_number = total_viewpoint_features_number/counter;

	// Push results to output data ports.
	out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
	out_cloud_xyzrgb.write(cloud_merged);
	out_cloud_xyzsift.write(cloud_sift_merged);

	// Push SOM - depricated.
//	out_instance.write(produce());
}
void LUMGenerator::out_trigger(){

}


} //: namespace LUMGenerator
} //: namespace Processors
