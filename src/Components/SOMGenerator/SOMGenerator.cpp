/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "SOMGenerator.hpp"
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
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>



namespace Processors {
namespace SOMGenerator {
/*TO DO:
 * -przetestowac writer oraz reader czy odpowiednio zapisuja/wczytuja z typu PointXYZRGBNormal
 * -ICP color: czy nowy typ, czy tylko ma wyciagac informacge z PointXYZRGB o rgb i porownywac, dziecziczone po IterativeClosestPoint
 * -domykanie pętli
 *
 *
 *
 */


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


Eigen::Matrix4f SOMGenerator::computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg,
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers)
{
	CLOG(LTRACE) << "Computing SAC" << std::endl ;	

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
	sac.setInputSource(cloud_src) ;
	sac.setInputTarget(cloud_trg) ;
	sac.setInlierThreshold(RanSAC_inliers_threshold) ; //property RanSAC
	sac.setMaximumIterations(RanSAC_max_iterations) ; //property RanSAC
	sac.setInputCorrespondences(correspondences) ;
	sac.getCorrespondences(inliers) ;

	CLOG(LINFO) << "SAC inliers " << inliers.size();

	return sac.getBestTransformation() ;
}



SOMGenerator::SOMGenerator(const std::string & name) :
    Base::Component(name),
    prop_ICP_alignment("ICP.Points", true),
    prop_ICP_alignment_normal("ICP.Normals",true),
    prop_ICP_alignment_color("ICP.Color",false),
    ICP_transformation_epsilon("ICP.Tranformation_epsilon",1e-6),
    ICP_max_correspondence_distance("ICP.Correspondence_distance",0.1),
    ICP_max_iterations("ICP.Iterations",2000),
    RanSAC_inliers_threshold("RanSac.Inliers_threshold",0.01f),
    RanSAC_max_iterations("RanSac.Iterations",2000)
{
    registerProperty(prop_ICP_alignment);
    registerProperty(prop_ICP_alignment_normal);
    registerProperty(prop_ICP_alignment_color);
    registerProperty(ICP_transformation_epsilon);
    registerProperty(ICP_max_correspondence_distance);
    registerProperty(ICP_max_iterations);
    registerProperty(RanSAC_inliers_threshold);
    registerProperty(RanSAC_max_iterations);
}


SOMGenerator::~SOMGenerator() {
}

void SOMGenerator::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb_normals", &in_cloud_xyzrgb_normals);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);

    // Register single handler - the "addViewToModel" function.
    h_addViewToModel.setup(boost::bind(&SOMGenerator::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
    addDependency("addViewToModel", &in_cloud_xyzrgb);

    h_addViewToModel_normals.setup(boost::bind(&SOMGenerator::addViewToModel_normals, this));
    registerHandler("addViewToModel_normals", &h_addViewToModel_normals);
    addDependency("addViewToModel_normals", &in_cloud_xyzsift);
    addDependency("addViewToModel_normals", &in_cloud_xyzrgb_normals);

}

bool SOMGenerator::onInit() {
	// Number of viewpoints.
	counter = 0;
	// Mean number of features per view. 
	mean_viewpoint_features_number = 0;
		
	global_trans = Eigen::Matrix4f::Identity();

	cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
	cloud_sift_merged = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	cloud_normal_merged = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	return true;
}

bool SOMGenerator::onFinish() {
	return true;
}

bool SOMGenerator::onStop() {
	return true;
}

bool SOMGenerator::onStart() {
	return true;
}

void SOMGenerator::addViewToModel() {
    CLOG(LTRACE) << "SOMGenerator::addViewToModel";
	
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

		return;
	}

    // SOMGenerator view count and feature numbers.
	counter++;
	total_viewpoint_features_number += cloud_sift->size();
	
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;
	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation());
	correst.setPointRepresentation(point_representation);
	correst.setInputSource(cloud_sift) ;
	correst.setInputTarget(cloud_sift_merged) ;
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences) ;
	CLOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_sift->size() << " features";

	// Compute multiplicity of features (designating how many multiplicity given feature appears in all views).
	for(int i = 0; i< correspondences->size();i++){	
		if (correspondences->at(i).index_query >=cloud_sift->size() || correspondences->at(i).index_match >=cloud_sift_merged->size()){
			continue;
		}
		// 		
		cloud_sift->at(correspondences->at(i).index_query).multiplicity = cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity + 1;
		cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity=-1;
	}


    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
	pcl::Correspondences inliers;
	Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers) ;

	if (current_trans == Eigen::Matrix4f::Identity()){
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
		return;
	}//: if
//	CLOG(LINFO) << "SAC Transformation from current view cloud to cloud_merged: " << std::endl << current_trans;
	global_trans = global_trans * current_trans;
	CLOG(LINFO) << "Transformation from current view cloud to first view: " << std::endl << global_trans << std::endl ;


	// Delete points.
	pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift_merged->begin();
	while(pt_iter!=cloud_sift_merged->end()){
		if(pt_iter->multiplicity==-1){
			pt_iter = cloud_sift_merged->erase(pt_iter);
		} else {
			++pt_iter;	
		}
	}

	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

    if (prop_ICP_alignment) {
        // Use ICP to get "better" transformation.
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (SOMGenerator::ICP_max_correspondence_distance); //property
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (SOMGenerator::ICP_max_iterations); // property
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (SOMGenerator::ICP_transformation_epsilon); //property
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1); // property ?

        icp.setInputSource(cloud);
        icp.setInputTarget(cloud_merged);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*Final);
        CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

        // Get the transformation from target to source.
        current_trans = icp.getFinalTransformation().inverse();
        CLOG(LINFO) << "ICP transformation refinement: " << std::endl << current_trans;

        // Refine the transformation.
        pcl::transformPointCloud(*cloud, *cloud, current_trans);
        pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

    }
    else if(prop_ICP_alignment_color)
    {

    }


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
}

void SOMGenerator::addViewToModel_normals() {

	CLOG(LTRACE) << "SOMGenerator::addViewToModel_normals";

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();

		// TODO if empty()

		CLOG(LDEBUG) << "cloud_xyzrgb_normals size: "<<cloud->size();
		CLOG(LDEBUG) << "cloud_xyzsift size: "<<cloud_sift->size();

		// Remove NaNs.
		std::vector<int> indices;
		cloud->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		cloud_sift->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);

		CLOG(LDEBUG) << "cloud_xyzrgb_normals size without NaN: "<<cloud->size();
		CLOG(LDEBUG) << "cloud_xyzsift size without NaN: "<<cloud_sift->size();

		CLOG(LINFO) << "view number: "<<counter;
		CLOG(LINFO) << "view cloud->size(): "<<cloud->size();
		CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift->size();


		// First cloud.
		if (counter == 0 ){
			*cloud_normal_merged = *cloud;
			*cloud_sift_merged = *cloud_sift;

			counter++;
			mean_viewpoint_features_number = cloud_sift->size();
			// Push results to output data ports.
			out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
			out_cloud_xyzrgb_normals.write(cloud_normal_merged);
			out_cloud_xyzsift.write(cloud_sift_merged);

			return;
		}

	    // SOMGenerator view count and feature numbers.
		counter++;
		total_viewpoint_features_number += cloud_sift->size();

		// Find correspondences between feature clouds.
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

		// Compute multiplicity of features (designating how many multiplicity given feature appears in all views).
		for(int i = 0; i< correspondences->size();i++){
			if (correspondences->at(i).index_query >=cloud_sift->size() || correspondences->at(i).index_match >=cloud_sift_merged->size()){
				continue;
			}
			//
			cloud_sift->at(correspondences->at(i).index_query).multiplicity = cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity + 1;
			cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity=-1;
		}


	    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
		pcl::Correspondences inliers;
		std::vector<int> inliers_c;
		Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers) ;
/*
		  pcl::SampleConsensusModelPlane<pcl::PointXYZRGBNormal>::Ptr
		    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBNormal> (cloud));

		pcl::RandomSampleConsensus<pcl::PointXYZRGBNormal> sac (model_p);
		sac.setDistanceThreshold(RanSAC_inliers_threshold) ; //property RanSAC
		sac.computeModel();
		pcl::copyPointCloud<pcl::PointXYZRGBNormal>(*cloud, inliers_c, *cloud);*/

		CLOG(LINFO) << "SAC inliers " << inliers.size();




		if (current_trans == Eigen::Matrix4f::Identity()){
			// Add clouds.

				*cloud_normal_merged += *cloud;
				*cloud_sift_merged += *cloud_sift;

				CLOG(LINFO) << "model cloud->size(): "<<cloud_normal_merged->size();
				CLOG(LINFO) << "model cloud_sift_normal->size(): "<<cloud_sift_merged->size();


				// Compute mean number of features.
				mean_viewpoint_features_number = total_viewpoint_features_number/counter;

				// Push results to output data ports.
				out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
				out_cloud_xyzrgb_normals.write(cloud_normal_merged);
				out_cloud_xyzsift.write(cloud_sift_merged);
			return;
		}//: if
	//	CLOG(LINFO) << "SAC Transformation from current view cloud to cloud_merged: " << std::endl << current_trans;
		global_trans = global_trans * current_trans;
		CLOG(LINFO) << "Transformation from current view cloud to first view: " << std::endl << global_trans << std::endl ;


		// Delete points.
		pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift_merged->begin();
		while(pt_iter!=cloud_sift_merged->end()){
			if(pt_iter->multiplicity==-1){
				pt_iter = cloud_sift_merged->erase(pt_iter);
			} else {
				++pt_iter;
			}
		}

		pcl::transformPointCloudWithNormals(*cloud, *cloud, current_trans);
		pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);


	    if(prop_ICP_alignment_normal){

	        // Use ICP to get "better" transformation.
	        pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

	        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	        icp.setMaxCorrespondenceDistance (SOMGenerator::ICP_max_correspondence_distance); //property
	        // Set the maximum number of iterations (criterion 1)
	        icp.setMaximumIterations (SOMGenerator::ICP_max_iterations); // property
	        // Set the transformation epsilon (criterion 2)
	        icp.setTransformationEpsilon (SOMGenerator::ICP_transformation_epsilon); //property
	        // Set the euclidean distance difference epsilon (criterion 3)
	        icp.setEuclideanFitnessEpsilon (0.001); // property ?

	        icp.setInputSource(cloud);

	        icp.setInputTarget(cloud_normal_merged);
	        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	        //icp.align(*Final);

	        // Run the same optimization in a loop and visualize the results
	        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	        reg_result = cloud;

	        for (int i = 0; i < 30; ++i)
	        {

	          // Estimate
		        icp.setInputSource(reg_result);

		        icp.align (*Final);
		        if(icp.converged_==false)
		        {
		        	PCL_INFO("Not enough correspondences so nothing happened\n");
		        	return;
		        }

	      		//accumulate transformation between each Iteration
	          Ti = icp.getFinalTransformation () * Ti;

	      		//if the difference between this transformation and the previous one
	      		//is smaller than the threshold, refine the process by reducing
	      		//the maximal correspondence distance
	          if (fabs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
	            icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.001);

	          prev = icp.getLastIncrementalTransformation ();

	          reg_result= Final;
	        }

	        CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

	        // Get the transformation from target to source.
	        current_trans = icp.getFinalTransformation().inverse();
	        CLOG(LINFO) << "ICP transformation refinement: " << std::endl << current_trans;

	        // Refine the transformation.

	        pcl::transformPointCloudWithNormals(*cloud, *cloud, current_trans);
	        pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

	    }

		// Add clouds.

		*cloud_normal_merged += *cloud;
		*cloud_sift_merged += *cloud_sift;

		CLOG(LINFO) << "model cloud->size(): "<<cloud_normal_merged->size();
		CLOG(LINFO) << "model cloud_sift->size(): "<<cloud_sift_merged->size();


		// Compute mean number of features.
		mean_viewpoint_features_number = total_viewpoint_features_number/counter;

		// Push results to output data ports.
		out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
		out_cloud_xyzrgb_normals.write(cloud_normal_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);

		// Push SOM - depricated.
}

} //: namespace SOMGenerator
} //: namespace Processors
