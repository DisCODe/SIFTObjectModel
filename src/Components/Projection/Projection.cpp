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
        icp_corr_distance("icp_corr_distance", 0.005f),
        bounding_box_epsilon("bounding_box_epsilon", 0.005f),
        use_icp("use_icp", false){
            registerProperty(icp_max_iter);
            registerProperty(icp_corr_distance);
            registerProperty(bounding_box_epsilon);
            registerProperty(use_icp);
}

Projection::~Projection() {
}

void Projection::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_cloud_xyz_scene", &in_cloud_xyz_scene);
    registerStream("in_cloud_xyz_model", &in_cloud_xyz_model);
    registerStream("in_cloud_xyzrgb_scene", &in_cloud_xyzrgb_scene);
    registerStream("in_cloud_xyzrgb_model", &in_cloud_xyzrgb_model);
	registerStream("in_cloud_xyzsift_scene", &in_cloud_xyzsift_scene);
	registerStream("in_cloud_xyzsift_model", &in_cloud_xyzsift_model);
	registerStream("in_rototranslations", &in_rototranslations);
    registerStream("out_registered_instances_xyz", &out_registered_instances_xyz);
    registerStream("out_registered_instances_xyzrgb", &out_registered_instances_xyzrgb);
    registerStream("out_registered_instances_xyzsift", &out_registered_instances_xyzsift);
    registerStream("out_parts_of_scene_xyzrgb", &out_parts_of_scene_xyzrgb);
	// Register handlers
    registerHandler("project", boost::bind(&Projection::project, this));
    addDependency("project", &in_cloud_xyz_scene);
    addDependency("project", &in_cloud_xyz_model);
    addDependency("project", &in_rototranslations);

    registerHandler("project_xyzrgb", boost::bind(&Projection::project_xyzrgb, this));
    addDependency("project_xyzrgb", &in_cloud_xyzrgb_scene);
    addDependency("project_xyzrgb", &in_cloud_xyzrgb_model);
    addDependency("project_xyzrgb", &in_rototranslations);

    registerHandler("project_xyzsift", boost::bind(&Projection::project_xyzsift, this));
    addDependency("project_xyzsift", &in_cloud_xyzsift_scene);
    addDependency("project_xyzsift", &in_cloud_xyzsift_model);
    addDependency("project_xyzsift", &in_rototranslations);


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
    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = in_cloud_xyz_scene.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr model = in_cloud_xyz_model.read();

    /**
     * Generates clouds for each instances found
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> parts_of_scene;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      //rotate model
      pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
      instances.push_back (rotated_model);

      //cut part of scene
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D(*rotated_model, minPt, maxPt);
      minPt.x -= bounding_box_epsilon;
      minPt.y -= bounding_box_epsilon;
      minPt.z -= bounding_box_epsilon;
      maxPt.x += bounding_box_epsilon;
      maxPt.y += bounding_box_epsilon;
      maxPt.z += bounding_box_epsilon;
      pcl::PointCloud<pcl::PointXYZ>::Ptr part_of_scene (new pcl::PointCloud<pcl::PointXYZ> ());
      *part_of_scene = *scene;
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (part_of_scene);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (minPt.x, maxPt.x);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (minPt.y, maxPt.y);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (minPt.z, maxPt.z);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      parts_of_scene.push_back(part_of_scene);
      CLOG(LDEBUG) << "part of scene " << i << " points " << part_of_scene->size();
    }

    /**
     * ICP
     */
    if(use_icp){
        std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> registered_instances;
        CLOG(LINFO) << "ICP";
        for (size_t i = 0; i < rototranslations.size (); ++i)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setMaximumIterations (icp_max_iter);
            icp.setMaxCorrespondenceDistance (icp_corr_distance);
            icp.setInputTarget (scene);
            icp.setInputSource (instances[i]);
            pcl::PointCloud<pcl::PointXYZ>::Ptr registered (new pcl::PointCloud<pcl::PointXYZ>);
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
        out_registered_instances_xyz.write(registered_instances);
    }
    else
        out_registered_instances_xyz.write(instances);

}


void Projection::project_xyzsift() {
    CLOG(LTRACE) << "Projection::project_xyzsift";
    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr scene = in_cloud_xyzsift_scene.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr model = in_cloud_xyzsift_model.read();

    /**
     * Generates clouds for each instances found
     */
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> instances;
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> parts_of_scene;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      //rotate model
      pcl::PointCloud<PointXYZSIFT>::Ptr rotated_model (new pcl::PointCloud<PointXYZSIFT> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
      instances.push_back (rotated_model);

      //cut part of scene
      PointXYZSIFT minPt, maxPt;
      pcl::getMinMax3D(*rotated_model, minPt, maxPt);
      minPt.x -= bounding_box_epsilon;
      minPt.y -= bounding_box_epsilon;
      minPt.z -= bounding_box_epsilon;
      maxPt.x += bounding_box_epsilon;
      maxPt.y += bounding_box_epsilon;
      maxPt.z += bounding_box_epsilon;
      pcl::PointCloud<PointXYZSIFT>::Ptr part_of_scene (new pcl::PointCloud<PointXYZSIFT> ());
      *part_of_scene = *scene;
      pcl::PassThrough<PointXYZSIFT> pass;
      pass.setInputCloud (part_of_scene);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (minPt.x, maxPt.x);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (minPt.y, maxPt.y);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (minPt.z, maxPt.z);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      parts_of_scene.push_back(part_of_scene);
      CLOG(LDEBUG) << "part of scene " << i << " points " << part_of_scene->size();
    }

    /**
     * ICP
     */
    if(use_icp){
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
        out_registered_instances_xyzsift.write(registered_instances);
    }
    else
        out_registered_instances_xyzsift.write(instances);

}


void Projection::project_xyzrgb() {
    CLOG(LTRACE) << "Projection::project_xyzrgb";
    std::vector<Types::HomogMatrix> rototranslations = in_rototranslations.read();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene = in_cloud_xyzrgb_scene.read();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model = in_cloud_xyzrgb_model.read();

    CLOG(LDEBUG) << "Scene points " << scene->size();
    /**
     * Generates clouds for each instances found
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> instances;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> parts_of_scene;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
      //rotate model
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_model (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
      instances.push_back (rotated_model);

      //cut part of scene
      pcl::PointXYZRGB minPt, maxPt;
      pcl::getMinMax3D(*rotated_model, minPt, maxPt);
      minPt.x -= bounding_box_epsilon;
      minPt.y -= bounding_box_epsilon;
      minPt.z -= bounding_box_epsilon;
      maxPt.x += bounding_box_epsilon;
      maxPt.y += bounding_box_epsilon;
      maxPt.z += bounding_box_epsilon;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr part_of_scene (new pcl::PointCloud<pcl::PointXYZRGB> ());
      *part_of_scene = *scene;
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (part_of_scene);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (minPt.x, maxPt.x);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (minPt.y, maxPt.y);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (minPt.z, maxPt.z);
      pass.setFilterLimitsNegative (false);
      pass.filter (*part_of_scene);
      parts_of_scene.push_back(part_of_scene);
      CLOG(LDEBUG) << "part of scene " << i << " points " << part_of_scene->size();
    }

    /**
     * ICP
     */
    if(use_icp){
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> registered_instances;
        CLOG(LINFO) << "ICP";
        for (size_t i = 0; i < rototranslations.size (); ++i)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setMaximumIterations (icp_max_iter);
            icp.setMaxCorrespondenceDistance (icp_corr_distance);
            icp.setInputTarget (parts_of_scene[i]);
            icp.setInputSource (instances[i]);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered (new pcl::PointCloud<pcl::PointXYZRGB>);
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
        out_registered_instances_xyzrgb.write(registered_instances);
    }
    else
        out_registered_instances_xyzrgb.write(instances);

    //link parts to one cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene1(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i < parts_of_scene.size(); i++){
        *scene1 += *(parts_of_scene[i]);
    }

    out_parts_of_scene_xyzrgb.write(scene1);

}

} //: namespace Projection
} //: namespace Processors
