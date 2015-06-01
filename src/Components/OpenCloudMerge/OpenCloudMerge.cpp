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
#include <Types/HomogMatrix.hpp>


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
	ICP_max_iterations.addConstraint("20000");
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
	registerStream("in_cloud_xyzrgb1_normals", &in_cloud_xyzrgb1_normals);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb1", &in_cloud_xyzrgb1);
	registerStream("in_cloud_xyzsift1", &in_cloud_xyzsift1);
	registerStream("out_instance", &out_instance);
	registerStream("out_diff", &out_diff);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);



    // Register single handler - the "addViewToModel" function.
    h_addViewToModel.setup(boost::bind(&OpenCloudMerge::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
    addDependency("addViewToModel", &in_cloud_xyzrgb);
    addDependency("addViewToModel", &in_cloud_xyzsift1);
    addDependency("addViewToModel", &in_cloud_xyzrgb1);

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
	previous_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = in_cloud_xyzrgb1.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift1 = in_cloud_xyzsift1.read();
		// TODO if empty()

		CLOG(LDEBUG) << "cloud_xyzrgb size: "<<cloud->size();
		CLOG(LDEBUG) << "cloud_xyzsift size: "<<cloud_sift->size();



		// Remove NaNs.
		std::vector<int> indices;
		cloud->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		cloud_sift->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);
		cloud->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
		cloud_sift->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_sift1, *cloud_sift1, indices);
		CLOG(LDEBUG) << "cloud_xyzrgb size without NaN: "<<cloud->size();
		CLOG(LDEBUG) << "cloud_xyzsift size without NaN: "<<cloud_sift->size();

		CLOG(LINFO) << "view number: "<<counter;
		CLOG(LINFO) << "view cloud->size(): "<<cloud->size();
		CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift->size();
		CLOG(LINFO) << "view cloud->size(): "<<cloud1->size();
		CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift1->size();

	    // SOMGenerator view count and feature numbers.
		counter++;
		//total_viewpoint_features_number += cloud_sift->size();

		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
		MergeUtils::computeCorrespondences(cloud_sift, cloud_sift1, correspondences);

	//	 Compute multiplicity of features (designating how many multiplicity given feature appears in all views).
		for(int i = 0; i< correspondences->size();i++){
			if (correspondences->at(i).index_query >=cloud_sift->size() || correspondences->at(i).index_match >=cloud_sift1->size()){
				continue;
			}
			//
			cloud_sift->at(correspondences->at(i).index_query).multiplicity = cloud_sift1->at(correspondences->at(i).index_match).multiplicity + 1;
			cloud_sift1->at(correspondences->at(i).index_match).multiplicity=-1;
		}


	    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
		pcl::Correspondences inliers;
		Eigen::Matrix4f sac_trans= MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift1, correspondences, inliers, properties) ;
		Eigen::Matrix4f current_trans=Eigen::Matrix4f::Identity();
		Eigen::Matrix4f icp_trans;
		CLOG(LINFO) << "SAC Transformation from current view cloud to cloud_merged: " << std::endl << sac_trans;
		//global_trans = global_trans * current_trans;
		//CLOG(LINFO) << "Transformation from current view cloud to first view: " << std::endl << global_trans << std::endl ;


	//	 Delete points.
		pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift1->begin();
		while(pt_iter!=cloud_sift1->end()){
			if(pt_iter->multiplicity==-1){
				pt_iter = cloud_sift1->erase(pt_iter);
			} else {
				++pt_iter;
			}
		}

		pcl::transformPointCloud(*cloud, *cloud, sac_trans);
		pcl::transformPointCloud(*cloud_sift, *cloud_sift, sac_trans);
		current_trans=sac_trans;

	    if (prop_ICP_alignment) {
	    	icp_trans = MergeUtils::computeTransformationICP(cloud, cloud1, properties);
	    	CLOG(LINFO) << "ICP transformation refinement:\n " << icp_trans;
	    	 Eigen::Matrix<double, 4, 1> centroid_src, centroid_tgt;
	    	 pcl::PointCloud<pcl::PointXYZRGB> cloudd = *cloud;
	    	 pcl::ConstCloudIterator<pcl::PointXYZRGB>* cloud_it = new pcl::ConstCloudIterator<pcl::PointXYZRGB>(cloudd);
	    	 pcl::PointCloud<pcl::PointXYZRGB> clouds = *cloud1;
	    	 pcl::ConstCloudIterator<pcl::PointXYZRGB>* cloud_it1 = new pcl::ConstCloudIterator<pcl::PointXYZRGB>(clouds);
	    	  pcl::compute3DCentroid (*cloud_it, centroid_tgt);
	    	  pcl::compute3DCentroid (*cloud_it1, centroid_src);
	    	  CLOG(LINFO) << "CENTROID" << centroid_src;
	    	  CLOG(LINFO) << "CENTROID" << centroid_tgt;
	    	// Refine the transformation.
	    	pcl::transformPointCloud(*cloud, *cloud, icp_trans);
	    	pcl::transformPointCloud(*cloud_sift, *cloud_sift, icp_trans);
	    	current_trans*=icp_trans;
	    }
	    if(prop_ICP_alignment_color)
	    {
	    	icp_trans = MergeUtils::computeTransformationICPColor(cloud, cloud1, properties);
	    	CLOG(LINFO) << "ICP Color transformation refinement:\n " << icp_trans;

	    	// Refine the transformation.
	    	pcl::transformPointCloud(*cloud, *cloud, icp_trans);
	    	pcl::transformPointCloud(*cloud_sift, *cloud_sift, icp_trans);
	    	current_trans*=icp_trans;
	    }

		// Add clouds.
	    //  *previous_cloud = *cloud;
		*cloud1 += *cloud;
		*cloud_sift1 += *cloud_sift;

		//DEFINING PROPER TRANSFORMATION FROM TRANSFORM CLOUD

		Eigen::Matrix4f proper_trans = Eigen::Matrix4f::Identity();

		proper_trans << 0.90709,  -0.378794,  -0.183581,  0.5,
				0.290306,  0.878771,  -0.378794,  0.5,
				0.304811,  0.290306,  0.90709,  0.5,
				0,  0,  0,  1;

		Eigen::Matrix4f diff_trans = Eigen::Matrix4f::Identity();

		//diff_trans = (current_trans - proper_trans).array().abs().matrix();

		diff_trans = current_trans;
		Types::HomogMatrix hm;
		stringstream ss;

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 4; ++j) {
                hm(i, j) = diff_trans(i,j);
                ss << hm(i, j) << "  ";
			}
			ss << "\n";
		}

		CLOG(LINFO) << "Złożenie transformacji:\n" << ss.str();

		CLOG(LINFO) << "model cloud->size(): "<<cloud1->size();
		CLOG(LINFO) << "model cloud_sift->size(): "<<cloud_sift1->size();

		out_diff.write(hm);
		out_cloud_xyzrgb.write(cloud1);
		out_cloud_xyzsift.write(cloud_sift1);

		// Push SOM - depricated.
}

void OpenCloudMerge::addViewToModelNormals() {

		CLOG(LTRACE) << "SOMGenerator::addViewToModelNormals";

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1 = in_cloud_xyzrgb1_normals.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift1 = in_cloud_xyzsift1.read();

		// Remove NaNs.
		std::vector<int> indices;
		cloud->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		cloud_sift->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);
		cloud1->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
		cloud_sift1->is_dense = false;
		pcl::removeNaNFromPointCloud(*cloud_sift1, *cloud_sift1, indices);
		pcl::removeNaNNormalsFromPointCloud(*cloud1,*cloud1, indices);
		pcl::removeNaNNormalsFromPointCloud(*cloud,*cloud, indices);

		CLOG(LDEBUG) << "cloud_xyzrgb_normals size without NaN: "<<cloud->size();
		CLOG(LDEBUG) << "cloud_xyzsift size without NaN: "<<cloud_sift->size();

		CLOG(LINFO) << "view number: "<<counter;
		CLOG(LINFO) << "view cloud->size(): "<<cloud->size();
		CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift->size();

	    // SOMGenerator view count and feature numbers.
		counter++;

		// Find correspondences between feature clouds.
		// Initialize parameters.
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
		MergeUtils::computeCorrespondences(cloud_sift, cloud_sift1, correspondences);


	    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
		pcl::Correspondences inliers;
		//Eigen::Matrix4f current_trans = MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift1, correspondences, inliers,properties) ;
		Eigen::Matrix4f sac_trans= MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift1, correspondences, inliers, properties) ;
		CLOG(LINFO) << "SAC inliers " << inliers.size();
		//Eigen::Matrix4f sac_trans= MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift1, correspondences, inliers, properties) ;
		Eigen::Matrix4f current_trans=Eigen::Matrix4f::Identity();
		Eigen::Matrix4f icp_trans;
	//	CLOG(LINFO) << "SAC Transformation from current view cloud to cloud_merged: " << std::endl << current_trans;
		//global_trans = global_trans * current_trans;
		CLOG(LINFO) << "SAC transformation:  " << std::endl << sac_trans << std::endl ;

		// Delete points.
		pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift1->begin();
		while(pt_iter!=cloud_sift1->end()){
			if(pt_iter->multiplicity==-1){
				pt_iter = cloud_sift1->erase(pt_iter);
			} else {
				++pt_iter;
			}
		}

		pcl::transformPointCloudWithNormals(*cloud, *cloud, sac_trans);
		pcl::transformPointCloud(*cloud_sift, *cloud_sift, sac_trans);
		current_trans=sac_trans;

	    if(prop_ICP_alignment_normal){

	    	icp_trans = MergeUtils::computeTransformationICPNormals(cloud, cloud1, properties);
	        CLOG(LINFO) << "ICP Normal transformation refinement: " << std::endl << icp_trans;

	        pcl::transformPointCloudWithNormals(*cloud, *cloud, icp_trans);
	        pcl::transformPointCloud(*cloud_sift, *cloud_sift, icp_trans);

	        current_trans*=icp_trans;
	    }

		// Add clouds.

		*cloud1 += *cloud;
		*cloud_sift1 += *cloud_sift;

		Eigen::Matrix4f diff_trans = Eigen::Matrix4f::Identity();
		diff_trans = current_trans;
		Types::HomogMatrix hm;
		stringstream ss;

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 4; ++j) {
                hm(i, j) = diff_trans(i,j) ;
                ss << hm(i, j) << "  ";
			}
			ss << "\n";
		}

		CLOG(LINFO) << "Złożenie transformacji:\n" << ss.str();

		CLOG(LINFO) << "model cloud->size(): "<<cloud1->size();
		CLOG(LINFO) << "model cloud_sift->size(): "<<cloud_sift1->size();

		out_diff.write(hm);

		// Compute mean number of features.

		// Push results to output data ports.

		out_cloud_xyzrgb_normals.write(cloud1);
		out_cloud_xyzsift.write(cloud_sift1);


		// Push SOM - depricated.
}


} //: namespace OpenCloudMerged
} //: namespace Processors
