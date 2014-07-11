
//
#include "ClosedCloudMerge.hpp"
#include "Common/Logger.hpp"

#include <Types/MergeUtils.hpp>
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
//
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

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

namespace Processors {
namespace ClosedCloudMerge {


ClosedCloudMerge::ClosedCloudMerge(const std::string & name) :
    Base::Component(name),
    prop_ICP_alignment("ICP.Points", true),
    prop_ICP_alignment_normal("ICP.Normals",true),
    prop_ICP_alignment_color("ICP.Color",false),
    ICP_transformation_epsilon("ICP.Tranformation_epsilon",1e-6),
    ICP_max_correspondence_distance("ICP.Correspondence_distance",0.1),
    ICP_max_iterations("ICP.Iterations",2000),
    RanSAC_inliers_threshold("RanSac.Inliers_threshold",0.01f),
    RanSAC_max_iterations("RanSac.Iterations",2000),
    viewNumber("View.Number", 5),
    maxIterations("Interations.Max", 5),
    corrTreshold("Correspondenc.Treshold", 10)
{
    registerProperty(prop_ICP_alignment);
    registerProperty(prop_ICP_alignment_normal);
    registerProperty(prop_ICP_alignment_color);
    registerProperty(ICP_transformation_epsilon);
    registerProperty(ICP_max_correspondence_distance);
    registerProperty(ICP_max_iterations);
    registerProperty(RanSAC_inliers_threshold);
    registerProperty(RanSAC_max_iterations);
    registerProperty(maxIterations);
    registerProperty(viewNumber);
    registerProperty(corrTreshold);

	properties.ICP_transformation_epsilon = ICP_transformation_epsilon;
	properties.ICP_max_iterations = ICP_max_iterations;
	properties.ICP_max_correspondence_distance = ICP_max_correspondence_distance;
	properties.RanSAC_inliers_threshold = RanSAC_inliers_threshold;
	properties.RanSAC_max_iterations = RanSAC_max_iterations;
}

ClosedCloudMerge::~ClosedCloudMerge() {
}


void ClosedCloudMerge::prepareInterface() {
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
    h_addViewToModel.setup(boost::bind(&ClosedCloudMerge::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
//    addDependency("addViewToModel", &in_cloud_xyzrgb);
    addDependency("addViewToModel", &in_cloud_xyzrgb_normals);
}

bool ClosedCloudMerge::onInit() {
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

bool ClosedCloudMerge::onFinish() {
	return true;
}

bool ClosedCloudMerge::onStop() {
	return true;
}

bool ClosedCloudMerge::onStart() {
	return true;
}


void ClosedCloudMerge::addViewToModel()
{
    CLOG(LDEBUG) << "ClosedCloudMerge::addViewToModel";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = in_cloud_xyzrgb_normals.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sift = in_cloud_xyzsift.read();

	// TODO if empty()

	CLOG(LINFO) << "cloud_xyzrgb size: "<<cloudrgb->size();
	CLOG(LINFO) << "cloud_xyzrgb_normals size: "<<cloud->size();
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
	rgbn_views.push_back(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>()));

	counter++;

	// First cloud.
	if (counter == 1)
	{
		//counter++;
	//	mean_viewpoint_features_number = cloud_sift->size();
		// Push results to output data ports.
		out_mean_viewpoint_features_number.write(cloud_sift->size());

		lum_sift.addPointCloud(cloud_sift);
		*rgbn_views[0] = *cloud;


		*cloud_merged = *cloudrgb;
		*cloud_normal_merged = *cloud;
		*cloud_sift_merged = *cloud_sift;

		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzrgb_normals.write(cloud_normal_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);
		// Push SOM - depricated.
//		out_instance.write(produce());
		CLOG(LINFO) << "return ";
		return;
	}
//	 Find corespondences between feature clouds.
//	 Initialize parameters.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
	MergeUtils::computeCorrespondences(cloud_sift, cloud_sift_merged, correspondences);

    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
	pcl::Correspondences inliers;
	Eigen::Matrix4f current_trans = MergeUtils::computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers, properties);
	if (current_trans == Eigen::Matrix4f::Identity())
	{
		CLOG(LINFO) << "cloud couldn't be merged";
		counter--;
		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzrgb_normals.write(cloud_normal_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);
		// Push SOM - depricated.
//		out_instance.write(produce());
		return;
	}

	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloudrgb, *cloudrgb, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);


//	current_trans = MergeUtils::computeTransformationIPCNormals(cloud, cloud_normal_merged, properties);
//
//	pcl::transformPointCloud(*cloud, *cloud, current_trans);
//	pcl::transformPointCloud(*cloudrgb, *cloudrgb, current_trans);
//	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);


	lum_sift.addPointCloud(cloud_sift);
	*rgbn_views[counter -1] = *cloud;
	*rgb_views[counter -1] = *cloudrgb;



	int added = 0;
	for (int i = counter - 2 ; i >= 0; i--)
	{
		pcl::CorrespondencesPtr correspondences2(new pcl::Correspondences()) ;
		MergeUtils::computeCorrespondences(lum_sift.getPointCloud(counter - 1), lum_sift.getPointCloud(i), correspondences2);
		pcl::CorrespondencesPtr correspondences3(new pcl::Correspondences()) ;
		MergeUtils::computeTransformationSAC(lum_sift.getPointCloud(counter - 1), lum_sift.getPointCloud(i), correspondences2, *correspondences3, properties) ;
		//cortab[counter-1][i] = inliers2;
		CLOG(LINFO) << "  correspondences3: " << correspondences3->size() << " out of " << correspondences2->size();
		if (correspondences3->size() > corrTreshold) {
			lum_sift.setCorrespondences(counter-1, i, correspondences3);
			added++;
			for(int j = 0; j< correspondences3->size();j++){
				if (correspondences3->at(j).index_query >=lum_sift.getPointCloud(counter - 1)->size() || correspondences3->at(j).index_match >=lum_sift.getPointCloud(i)->size()){
					continue;
				}
				if (lum_sift.getPointCloud(i)->at(correspondences3->at(j).index_match).multiplicity != -1) {
					lum_sift.getPointCloud(counter - 1)->at(correspondences3->at(j).index_query).multiplicity = lum_sift.getPointCloud(i)->at(correspondences3->at(j).index_match).multiplicity + 1;
					lum_sift.getPointCloud(i)->at(correspondences3->at(j).index_match).multiplicity=-1;
				} else
					lum_sift.getPointCloud(counter - 1)->at(correspondences3->at(j).index_query).multiplicity += 1;
			}
		}
	//break;
	//CLOG(LINFO) << "computet for "<<counter-1 <<" and "<< i << "  correspondences2: " << correspondences2->size() << " out of " << correspondences2->size();
	}
	CLOG(LINFO) << "view " << counter << " have correspondences with " << added << " views";
	if (added == 0 )
		CLOG(LINFO) << endl << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<endl;


	*cloud_merged = *(rgb_views[0]);
	*cloud_normal_merged = *(rgbn_views[0]);

	if (counter > viewNumber) {
		lum_sift.setMaxIterations(maxIterations);
		lum_sift.compute();
		cloud_sift_merged = lum_sift.getConcatenatedCloud ();
		CLOG(LINFO) << "ended";
		CLOG(LINFO) << "cloud_merged from LUM ";
		for (int i = 1 ; i < viewNumber; i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB> tmprgb = *(rgb_views[i]);
			pcl::PointCloud<pcl::PointXYZRGBNormal> tmp = *(rgbn_views[i]);
			pcl::transformPointCloud(tmp, tmp, lum_sift.getTransformation (i));
			pcl::transformPointCloud(tmprgb, tmprgb, lum_sift.getTransformation (i));
			*cloud_merged += tmprgb;
			*cloud_normal_merged += tmp;
		}

		// Delete points.
		pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_sift_merged->begin();
		while(pt_iter!=cloud_sift_merged->end()){
			if(pt_iter->multiplicity==-1){
				pt_iter = cloud_sift_merged->erase(pt_iter);
			} else {
				++pt_iter;
			}
		}
	} else {
		for (int i = 1 ; i < counter; i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB> tmprgb = *(rgb_views[i]);
			pcl::PointCloud<pcl::PointXYZRGBNormal> tmp = *(rgbn_views[i]);
			pcl::transformPointCloud(tmp, tmp, lum_sift.getTransformation (i));
			pcl::transformPointCloud(tmprgb, tmprgb, lum_sift.getTransformation (i));
			*cloud_merged += tmprgb;
			*cloud_normal_merged += tmp;
		}

		cloud_sift_merged = lum_sift.getConcatenatedCloud ();
	}

		//*cloud_sift_merged += *cloud_sift;
//
	CLOG(LINFO) << "model cloud_merged->size(): "<< cloud_merged->size();
	CLOG(LINFO) << "model cloud_normal_merged->size(): "<< cloud_normal_merged->size();
	CLOG(LINFO) << "model cloud_sift_merged->size(): "<< cloud_sift_merged->size();


	// Compute mean number of features.
	//mean_viewpoint_features_number = total_viewpoint_features_number/counter;

	// Push results to output data ports.
	out_mean_viewpoint_features_number.write(total_viewpoint_features_number/counter);
	out_cloud_xyzrgb.write(cloud_merged);
	out_cloud_xyzrgb_normals.write(cloud_normal_merged);
	out_cloud_xyzsift.write(cloud_sift_merged);
}

} // namespace ClosedCloudMerge
} // namespace Processors
