/*
 * MergeUtils.cpp
 *
 *  Created on: 30 maj 2014
 *      Author: mlepicka
 */

#include "MergeUtils.hpp"

#include <memory>
#include <string>

#include "Common/Logger.hpp"

#include <boost/bind.hpp>

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

#include <pcl/filters/filter.h>

#include<pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>


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

MergeUtils::MergeUtils() {
	// TODO Auto-generated constructor stub

}

MergeUtils::~MergeUtils() {
	// TODO Auto-generated destructor stub
}

void MergeUtils::computeCorrespondences(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg, const pcl::CorrespondencesPtr& correspondences)
{
	//CLOG(LTRACE) << "Computing Correspondences" << std::endl;
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;
	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation());
	correst.setPointRepresentation(point_representation);
	correst.setInputSource(cloud_src) ;
	correst.setInputTarget(cloud_trg) ;
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences) ;
	//CLOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_src->size() << " features";
}

Eigen::Matrix4f MergeUtils::computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg,
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers, Properties properties)
{
	//CLOG(LTRACE) << "Computing SAC" << std::endl;

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
	sac.setInputSource(cloud_src) ;
	sac.setInputTarget(cloud_trg) ;
	sac.setInlierThreshold(properties.RanSAC_inliers_threshold) ; //property RanSAC
	sac.setMaximumIterations(properties.RanSAC_max_iterations) ; //property RanSAC
	sac.setInputCorrespondences(correspondences) ;
	sac.getCorrespondences(inliers) ;

	//CLOG(LINFO) << "SAC inliers " << inliers.size();

	return sac.getBestTransformation() ;
}

Eigen::Matrix4f MergeUtils::computeTransformationICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_trg, Properties properties)
{
        // Use ICP to get "better" transformation.
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (properties.ICP_max_correspondence_distance); //property
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (properties.ICP_max_iterations); // property
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (properties.ICP_transformation_epsilon); //property
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1); // property ?

        icp.setInputSource(cloud_src);
        icp.setInputTarget(cloud_trg);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*Final);
      //  CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

        // Get the transformation from target to source.
        return icp.getFinalTransformation();//.inverse();
}

Eigen::Matrix4f MergeUtils::computeTransformationICPColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_trg, Properties properties)
{
        // Use ICP to get "better" transformation.
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (properties.ICP_max_correspondence_distance); //property
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (properties.ICP_max_iterations); // property
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (properties.ICP_transformation_epsilon); //property
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1); // property ?

        pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>::Ptr ceptr(new pcl::registration::CorrespondenceEstimationColor<pcl::PointXYZRGB, pcl::PointXYZRGB, float>);
        icp.setCorrespondenceEstimation(ceptr);

        icp.setInputSource(cloud_src);
        icp.setInputTarget(cloud_trg);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*Final);



      //  CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

        // Get the transformation from target to source.
        return icp.getFinalTransformation();//.inverse();
}




Eigen::Matrix4f MergeUtils::computeTransformationICPNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_trg, Properties properties)
{
    // Use ICP to get "better" transformation.
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (properties.ICP_max_correspondence_distance); //property
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (properties.ICP_max_iterations); // property
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (properties.ICP_transformation_epsilon); //property
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.001); // property ?

    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_trg);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    reg_result = cloud_src;

    for (int i = 0; i < 100; ++i)
    {
      // Estimate
        icp.setInputSource(reg_result);
        icp.align (*Final);
        if(icp.converged_==false)
        {
        	PCL_INFO("Not enough correspondences so nothing happened\n");
        	return Eigen::Matrix4f::Identity();
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

    // Get the transformation from target to source.
    return icp.getFinalTransformation();//.inverse();
}
