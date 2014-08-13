/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "ELECHGenerator.hpp"
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
#include <pcl/registration/elch.h>

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
namespace ELECHGenerator {



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


Eigen::Matrix4f ELECHGenerator::computeTransformationSAC(const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_src, const pcl::PointCloud<PointXYZSIFT>::ConstPtr &cloud_trg,
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


ELECHGenerator::ELECHGenerator(const std::string & name) :
    Base::Component(name),
    Elch_loop_dist("ELCH.distance", 0.05),
    Elch_rejection_threshold("ELCH.rejection", 0.001),
    ICP_max_iterations("ELCH.maxIPCiterations", 2000),
    ICP_max_correspondence_distance("ELCH.maxIPCdistance", 0.0005)
{
	registerProperty(Elch_loop_dist);
	registerProperty(Elch_rejection_threshold);
	registerProperty(ICP_max_iterations);
	registerProperty(ICP_max_correspondence_distance);
}

ELECHGenerator::~ELECHGenerator() {
}


void ELECHGenerator::prepareInterface() {
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
    h_addViewToModel.setup(boost::bind(&ELECHGenerator::addViewToModel, this));
    registerHandler("addViewToModel", &h_addViewToModel);
    addDependency("addViewToModel", &in_cloud_xyzsift);
    addDependency("addViewToModel", &in_cloud_xyzrgb);



   // h_Trigger.setup(Trigger::trigger(),this);
   // registerHandler("Trigger", &h_Trigger);
	//addDependency("Trigger", &h_Trigger);

	//h_Trigger.setup(boost::bind(&ELECHGenerator::out_trigger, this));
	//registerHandler("trigger", &h_Trigger);
	//addDependency("trigger", &out_Trigger);
	//addDependency("trigger", &in_trigger);

}

bool ELECHGenerator::onInit() {
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

bool ELECHGenerator::onFinish() {
	return true;
}

bool ELECHGenerator::onStop() {
	return true;
}

bool ELECHGenerator::onStart() {
	return true;
}

bool loopDetection (int end, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds, double dist, int &first, int &last)
{
  static double min_dist = -1;
  int state = 0;

  for (int i = end-1; i >= 0; i--)
  {
    Eigen::Vector4f cstart, cend;
    pcl::compute3DCentroid (*(clouds[i]), cstart);
    pcl::compute3DCentroid (*(clouds[end]), cend);
    Eigen::Vector4f diff = cend - cstart;
    double norm = diff.norm ();

    std::cout << endl << "distance between " << i << " and " << end << " is " << norm << " state is " << state << endl;

    if (state == 0 && norm > dist)
    {
      state = 1;
      //std::cout << "state 1" << std::endl;
    }
    if (state > 0 && norm < dist)
    {
      state = 2;
      std::cout << "loop detected between scan " << i << " and scan " << end  << endl;
      if (min_dist < 0 || norm < min_dist)
      {
        min_dist = norm;
        first = i;
        last = end;
        break;
      }
    }
  }
  //std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
   if (min_dist > 0 && (state < 2 || end == int (clouds.size ()) - 1)) //TODO
   {
     min_dist = -1;
     return true;
   }
   return false;
 }

void ELECHGenerator::addViewToModel() {
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

	counter++;

	// First cloud.
	if (counter == 1)
	{
		//counter++;
		mean_viewpoint_features_number = cloud_sift->size();
		// Push results to output data ports.
		out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);

		lum_sift.addPointCloud(cloud_sift);
		*rgb_views[0] = *cloud;
		elch_rgb.addPointCloud(rgb_views[0]);


		*cloud_merged = *cloud;
		*cloud_sift_merged = *cloud_sift;

		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);
		// Push SOM - depricated.
//		out_instance.write(produce());
		CLOG(LINFO) << "return ";
		return;
	}

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


    // Compute transformation between clouds and SOMGenerator global transformation of cloud.
	pcl::Correspondences inliers;
	Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers) ;

	int i=0;

	while ( (inliers.size()) < 10 && (i <50)) //ICP property ?
	{

		Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_sift, cloud_sift_merged, correspondences, inliers) ;
		i++;
	}

	cout<<"i: "<<i<<endl;

	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

	lum_sift.addPointCloud(cloud_sift);
	*rgb_views[counter -1] = *cloud;
	elch_rgb.addPointCloud(rgb_views[counter -1]);
//	*cloud_sift_merged += *cloud_sift;




	int first, last;
	if (loopDetection(counter-1, rgb_views, Elch_loop_dist, first, last))
	{
		CLOG(LINFO) << "loop beetween " << first << " and " << last;
		elch_rgb.setLoopStart(first);
		elch_rgb.setLoopEnd(last);
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
		icp->setMaximumIterations(ICP_max_iterations);
		icp->setMaxCorrespondenceDistance(ICP_max_correspondence_distance);
		icp->setRANSACOutlierRejectionThreshold(Elch_rejection_threshold);
		elch_rgb.setReg(icp);
		elch_rgb.compute();
	}

	*cloud_merged = *(rgb_views[0]);
	for (int i = 1 ; i < counter; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB> tmp = *(rgb_views[i]);
	//	pcl::transformPointCloud(tmp, tmp, elch_sift.getTransformation (i));
		*cloud_merged += tmp;
	}

	cloud_sift_merged = lum_sift.getConcatenatedCloud ();


		//*cloud_sift_merged += *cloud_sift;
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

void ELECHGenerator::out_trigger(){

}


} //: namespace ELECHGenerator
} //: namespace Processors
