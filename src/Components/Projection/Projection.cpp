/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "Projection.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace Projection {

Projection::Projection(const std::string & name) :
        Base::Component(name),
        icp_max_iter("icp_max_iter", 5),
        icp_corr_distance("icp_corr_distance", 0.005f) {

}

Projection::~Projection() {
}

void Projection::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzsift_scene", &in_cloud_xyzsift_scene);
	registerStream("in_cloud_xyzsift_model", &in_cloud_xyzsift_model);
	registerStream("in_rototranslations", &in_rototranslations);
	registerStream("out_registered_instances", &out_registered_instances);
	// Register handlers
    registerHandler("project", boost::bind(&Projection::project, this));
    addDependency("project", &in_cloud_xyzsift_scene);
    addDependency("project", &in_cloud_xyzsift_model);
    addDependency("project", &in_rototranslations);

}

bool Projection::onInit() {

	return true;
}

bool Projection::onFinish() {
	return true;
}

bool Projection::onStop() {
	return true;
}

bool Projection::onStart() {
	return true;
}

void Projection::project() {
    CLOG(LTRACE) << "Projection::project";
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = in_rototranslations.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr scene = in_cloud_xyzsift_scene.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr model = in_cloud_xyzsift_model.read();

    /**
     * Generates clouds for each instances found
     */
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> instances;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::PointCloud<PointXYZSIFT>::Ptr rotated_model (new pcl::PointCloud<PointXYZSIFT> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
      instances.push_back (rotated_model);
    }

    /**
     * ICP
     */
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> registered_instances;
    CLOG(LINFO) << "ICP";
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
        pcl::IterativeClosestPoint<PointXYZSIFT, PointXYZSIFT> icp;
        icp.setMaximumIterations (icp_max_iter);
        icp.setMaxCorrespondenceDistance (icp_corr_distance);
        icp.setInputTarget (scene);
        icp.setInputSource (instances[i]);
        pcl::PointCloud<PointXYZSIFT>::Ptr registered (new pcl::PointCloud<PointXYZSIFT>);
        icp.align (*registered);
        registered_instances.push_back (registered);
        CLOG(LINFO) << "Instance " << i << " ";
        if (icp.hasConverged ())
        {
            CLOG(LINFO) << "Aligned!" << endl;
        }
        else
        {
            CLOG(LINFO) << "Not Aligned!" << endl;
        }
    }

    out_registered_instances.write(registered_instances);

}



} //: namespace Projection
} //: namespace Processors
