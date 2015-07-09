#include "CorrespondenceMatcher.hpp"
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
namespace CorrespondenceMatcher {


CorrespondenceMatcher::CorrespondenceMatcher(const std::string & name) :
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

	properties.ICP_transformation_epsilon = ICP_transformation_epsilon;
	properties.ICP_max_iterations = ICP_max_iterations;
	properties.ICP_max_correspondence_distance = ICP_max_correspondence_distance;
	properties.RanSAC_inliers_threshold = RanSAC_inliers_threshold;
	properties.RanSAC_max_iterations = RanSAC_max_iterations;
}

CorrespondenceMatcher::~CorrespondenceMatcher() {
}


void CorrespondenceMatcher::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_first_xyzsift", &in_cloud_first_xyzsift);
	registerStream("in_cloud_sec_xyzsift", &in_cloud_sec_xyzsift);
	registerStream("out_cloud_first_xyzsift", &out_cloud_first_xyzsift);
	registerStream("out_cloud_sec_xyzsift", &out_cloud_sec_xyzsift);
	registerStream("out_correspondences", &out_correspondences);

    h_mach.setup(boost::bind(&CorrespondenceMatcher::mach, this));
    registerHandler("mach", &h_mach);
    addDependency("mach", &in_cloud_first_xyzsift);
    addDependency("mach", &in_cloud_sec_xyzsift);
}

bool CorrespondenceMatcher::onInit() {

	return true;
}

bool CorrespondenceMatcher::onFinish() {
	return true;
}

bool CorrespondenceMatcher::onStop() {
	return true;
}

bool CorrespondenceMatcher::onStart() {
	return true;
}


void CorrespondenceMatcher::mach()
{
    CLOG(LDEBUG) << "CorrespondenceMatcher::match";

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_first = in_cloud_first_xyzsift.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_sec = in_cloud_sec_xyzsift.read();

	// TODO if empty()

	// Remove NaNs.
	std::vector<int> indices;
	cloud_first->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_first, *cloud_first, indices);
	cloud_sec->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_sec, *cloud_sec, indices);


	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
	MergeUtils::computeCorrespondences(cloud_first, cloud_sec, correspondences);
	pcl::CorrespondencesPtr inliers(new pcl::Correspondences()) ;
	Eigen::Matrix4f current_trans = MergeUtils::computeTransformationSAC(cloud_first, cloud_sec, correspondences, *inliers, properties);
    CLOG(LINFO) << "  current_trans:\n " << current_trans << "\n";
	CLOG(LINFO) << "  correspondences3: " << inliers->size() << " out of " << correspondences->size();
//	pcl::transformPointCloud(*cloud_sec, *cloud_sec, current_trans);

	out_correspondences.write(inliers);
	out_cloud_first_xyzsift.write(cloud_first);
	out_cloud_sec_xyzsift.write(cloud_sec);

	CLOG(LDEBUG) << "return ";
	return;
}

} // namespace CorrespondenceMatcher
} // namespace Processors
