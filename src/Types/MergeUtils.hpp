/*
 * MergeUtils.hpp
 *
 *  Created on: 30 maj 2014
 *      Author: mlepicka
 */

#ifndef MERGEUTILS_HPP_
#define MERGEUTILS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>
#include <Types/SIFTObjectModelFactory.hpp>

#include <Types/MergeUtils.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

#include <opencv2/core/core.hpp>

class MergeUtils {
public:
	MergeUtils();
	virtual ~MergeUtils();

	struct Properties {
    	double ICP_transformation_epsilon;
		int ICP_max_iterations;
		float ICP_max_correspondence_distance;
		float RanSAC_inliers_threshold;
		float RanSAC_max_iterations;
	};

    // Computes the correspondences between two XYZSIFT clouds
    static void computeCorrespondences(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg, const pcl::CorrespondencesPtr& correspondence);

    /// Computes the transformation between two XYZSIFT clouds basing on the found correspondences.
    static Eigen::Matrix4f computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg,
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers, Properties propertie);

    static Eigen::Matrix4f computeTransformationIPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_trg, Properties propertie);

    static Eigen::Matrix4f computeTransformationIPCNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_trg, Properties propertie);

};

#endif /* MERGEUTILS_HPP_ */
