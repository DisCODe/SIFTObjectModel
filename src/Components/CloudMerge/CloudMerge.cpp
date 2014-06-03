

#include <memory>
#include <string>

#include "CloudMerge.hpp"
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



namespace Processors {
namespace CloudMerge {
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

void CloudMerge::computeCorrespondences(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg, pcl::Correspondences* correspondences)
{
	CLOG(LTRACE) << "Computing Correspondences" << std::endl;
	pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst;
	SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation());
	correst.setPointRepresentation(point_representation);
	correst.setInputSource(cloud_src) ;
	correst.setInputTarget(cloud_trg) ;
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences) ;
	CLOG(LINFO) << "Number of reciprocal correspondences: " << correspondences->size() << " out of " << cloud_src->size() << " features";
}

Eigen::Matrix4f CloudMerge::computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg,
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

Eigen::Matrix4f CloudMerge::computeTransformationIPC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_trg)
{
        // Use ICP to get "better" transformation.
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (CloudMerge::ICP_max_correspondence_distance); //property
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (CloudMerge::ICP_max_iterations); // property
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (CloudMerge::ICP_transformation_epsilon); //property
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1); // property ?

        icp.setInputSource(cloud_src);
        icp.setInputTarget(cloud_trg);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*Final);
        CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

        // Get the transformation from target to source.
        return icp.getFinalTransformation().inverse();
}

Eigen::Matrix4f CloudMerge::computeTransformationIPCNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_trg)
{
    // Use ICP to get "better" transformation.
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (CloudMerge::ICP_max_correspondence_distance); //property
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (CloudMerge::ICP_max_iterations); // property
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (CloudMerge::ICP_transformation_epsilon); //property
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.001); // property ?

    icp.setInputSource(cloud_src);

    icp.setInputTarget(cloud_trg);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    //icp.align(*Final);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    reg_result = cloud_src;

    for (int i = 0; i < 30; ++i)
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

    CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

    // Get the transformation from target to source.
    return icp.getFinalTransformation().inverse();
}

CloudMerge::CloudMerge(const std::string & name) :
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


CloudMerge::~CloudMerge() {
}

void CloudMerge::prepareInterface() {
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
    h_addViewToModel.setup(boost::bind(&CloudMerge::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
    addDependency("addViewToModel", &in_cloud_xyzrgb);
}

bool CloudMerge::onInit() {
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

bool CloudMerge::onFinish() {
	return true;
}

bool CloudMerge::onStop() {
	return true;
}

bool CloudMerge::onStart() {
	return true;
}

void addViewToModel(){

}

} // namespace CloudMerge
} // namespace Processors

