/*!
 * \file
 * \brief
 * \author Marta Lepicka
 */

#include <memory>
#include <string>

#include "OpenCloudMerge.hpp"
#include "Common/Logger.hpp"
///////////////////////////////////////////////
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//
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

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include<pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <Types/MergeUtils.hpp>


namespace Processors {
namespace OpenCloudMerge {

OpenCloudMerge::OpenCloudMerge(const std::string & name) :
	Base::Component(name),
	prop_ICP_alignment("ICP.Points", true),
	prop_ICP_alignment_normal("ICP.Normals", true),
	prop_ICP_alignment_color("ICP.Color",true),
	ICP_transformation_epsilon("ICP.Tranformation_epsilon",1e-6),
	ICP_max_correspondence_distance("ICP.Correspondence_distance", 0.1),
	ICP_max_iterations("ICP.Iterations", 2000),
	RanSAC_inliers_threshold("RanSac.Inliers_threshold", 0.01f),
	RanSAC_max_iterations("RanSac.Iterations", 2000) {

	ICP_max_iterations.addConstraint("1");
	ICP_max_iterations.addConstraint("2000");
	registerProperty (prop_ICP_alignment);
	registerProperty (prop_ICP_alignment_normal);
	registerProperty (prop_ICP_alignment_color);
	registerProperty (ICP_transformation_epsilon);
	registerProperty (ICP_max_correspondence_distance);
	registerProperty (ICP_max_iterations);
	registerProperty (RanSAC_inliers_threshold);
	registerProperty (RanSAC_max_iterations);

	properties.ICP_transformation_epsilon = ICP_transformation_epsilon;
	properties.ICP_max_iterations = ICP_max_iterations;
	properties.ICP_max_correspondence_distance = ICP_max_correspondence_distance;
	properties.RanSAC_inliers_threshold = RanSAC_inliers_threshold;
	properties.RanSAC_max_iterations = RanSAC_max_iterations;
}

OpenCloudMerge::~OpenCloudMerge() {
}

void OpenCloudMerge::prepareInterface() {
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
    h_addViewToModel.setup(boost::bind(&OpenCloudMerge::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
    addDependency("addViewToModel", &in_cloud_xyzrgb);

    h_addViewToModelNormals.setup(boost::bind(&OpenCloudMerge::addViewToModelNormals, this));
    registerHandler("addViewToModelNormals", &h_addViewToModelNormals);
    addDependency("addViewToModelNormals", &in_cloud_xyzsift);
    addDependency("addViewToModelNormals", &in_cloud_xyzrgb_normals);
}

bool OpenCloudMerge::onInit() {
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

bool OpenCloudMerge::onFinish() {
	return true;
}

bool OpenCloudMerge::onStop() {
	return true;
}

bool OpenCloudMerge::onStart() {
	return true;
}

void OpenCloudMerge::addViewToModel(){

		CLOG(LTRACE) << "OpenCloudMerged::addViewToModel";

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
		MergeUtils::computeCorrespondences(cloud_sift, cloud_sift_merged, correspondences);

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
		Eigen::Matrix4f current_trans = MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers, properties) ;

		if (current_trans.isIdentity()){
			// Add clouds.
				CLOG(LINFO) << "model cloud->size(): "<<cloud_merged->size();
				CLOG(LINFO) << "model cloud_sift->size(): "<<cloud_sift_merged->size();

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
	    	current_trans = MergeUtils::computeTransformationICP(cloud, cloud_merged, properties);
	    	CLOG(LINFO) << "ICP transformation refinement: " << current_trans;

	    	// Refine the transformation.
	    	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	    	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);
	    }
	    if(prop_ICP_alignment_color)
	    {
	    	current_trans = MergeUtils::computeTransformationICPColor(cloud, cloud_merged, properties);
	    	CLOG(LINFO) << "ICP transformation refinement: " << current_trans;

	    	// Refine the transformation.
	    	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	    	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);
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

void OpenCloudMerge::addViewToModelNormals() {

		CLOG(LTRACE) << "SOMGenerator::addViewToModelNormals";

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnormal(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		cloud_xyzrgbnormal = cloud_normal_merged;
		cloud_xyzrgb->points.resize(cloud_xyzrgbnormal->size());
		for (size_t i = 0; i < cloud_xyzrgbnormal->points.size(); i++) {
		    cloud_xyzrgb->points[i].x = cloud_xyzrgbnormal->points[i].x;
		    cloud_xyzrgb->points[i].y = cloud_xyzrgbnormal->points[i].y;
		    cloud_xyzrgb->points[i].z = cloud_xyzrgbnormal->points[i].z;
		    cloud_xyzrgb->points[i].rgb = cloud_xyzrgbnormal->points[i].rgb;
		    cloud_xyzrgb->points[i].r = cloud_xyzrgbnormal->points[i].r;
		    cloud_xyzrgb->points[i].g = cloud_xyzrgbnormal->points[i].g;
		    cloud_xyzrgb->points[i].b = cloud_xyzrgbnormal->points[i].b;
		    cloud_xyzrgb->points[i].a = cloud_xyzrgbnormal->points[i].a;
		}
		// TODO if empty()

		CLOG(LDEBUG) << "cloud_xyzrgb_normals size: "<<cloud->size();
		CLOG(LDEBUG) << "cloud_xyzsift size: "<<cloud_sift->size();
		cloud_xyzrgb->points.resize(cloud_xyzrgbnormal->size());
		// Remove NaNs.
		std::vector<int> indices;
		cloud->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		cloud_sift->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);
		cloud_xyzrgb->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, indices);

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

			cloud_xyzrgbnormal = cloud_normal_merged;

			cloud_xyzrgb->points.resize(cloud_xyzrgbnormal->size());
			for (size_t i = 0; i < cloud_xyzrgbnormal->points.size(); i++) {
			    cloud_xyzrgb->points[i].x = cloud_xyzrgbnormal->points[i].x;
			    cloud_xyzrgb->points[i].y = cloud_xyzrgbnormal->points[i].y;
			    cloud_xyzrgb->points[i].z = cloud_xyzrgbnormal->points[i].z;
			    cloud_xyzrgb->points[i].rgb = cloud_xyzrgbnormal->points[i].rgb;
			    cloud_xyzrgb->points[i].r = cloud_xyzrgbnormal->points[i].r;
			    cloud_xyzrgb->points[i].g = cloud_xyzrgbnormal->points[i].g;
			    cloud_xyzrgb->points[i].b = cloud_xyzrgbnormal->points[i].b;
			}
			out_cloud_xyzrgb.write(cloud_xyzrgb);
			return;
		}

	    // SOMGenerator view count and feature numbers.
		counter++;
		total_viewpoint_features_number += cloud_sift->size();

		// Find correspondences between feature clouds.
		// Initialize parameters.
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
		MergeUtils::computeCorrespondences(cloud_sift, cloud_sift_merged, correspondences);


	    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
		pcl::Correspondences inliers;
		Eigen::Matrix4f current_trans = MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers,properties) ;

		CLOG(LINFO) << "SAC inliers " << inliers.size();

		if (current_trans.isIdentity()){

				CLOG(LINFO) << "model cloud->size(): "<<cloud_normal_merged->size();
				CLOG(LINFO) << "model cloud_sift_normal->size(): "<<cloud_sift_merged->size();

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

	    	current_trans = MergeUtils::computeTransformationICPNormals(cloud, cloud_normal_merged, properties);
	        CLOG(LINFO) << "ICP transformation refinement: " << std::endl << current_trans;

	        // Refine the transformation.
			if (current_trans.isIdentity()){
				// Add clouds.

					CLOG(LINFO) << "model cloud->size(): "<<cloud_normal_merged->size();
					CLOG(LINFO) << "model cloud_sift_normal->size(): "<<cloud_sift_merged->size();

					// Push results to output data ports.
					out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
					out_cloud_xyzrgb_normals.write(cloud_normal_merged);
					out_cloud_xyzsift.write(cloud_sift_merged);
				return;
			}//: if
	        pcl::transformPointCloudWithNormals(*cloud, *cloud, current_trans);
	        pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);
	    }


		for (size_t i = 0; i < cloud_xyzrgbnormal->points.size(); i++) {
		    cloud_xyzrgb->points[i].x = cloud_xyzrgbnormal->points[i].x;
		    cloud_xyzrgb->points[i].y = cloud_xyzrgbnormal->points[i].y;
		    cloud_xyzrgb->points[i].z = cloud_xyzrgbnormal->points[i].z;
		    cloud_xyzrgb->points[i].rgb = cloud_xyzrgbnormal->points[i].rgb;
		    cloud_xyzrgb->points[i].r = cloud_xyzrgbnormal->points[i].r;
		    cloud_xyzrgb->points[i].g = cloud_xyzrgbnormal->points[i].g;
		    cloud_xyzrgb->points[i].b = cloud_xyzrgbnormal->points[i].b;
		    cloud_xyzrgb->points[i].a = cloud_xyzrgbnormal->points[i].a;
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
		out_cloud_xyzrgb.write(cloud_xyzrgb);

		// Push SOM - depricated.
}


} //: namespace OpenCloudMerged
} //: namespace Processors
