/*!
 * \file
 * \brief
 * \author Michał Laszkowski
 */

#include <memory>
#include <string>

#include "ProjectionGrouping.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ProjectionGrouping {

ProjectionGrouping::ProjectionGrouping(const std::string & name) :
		Base::Component(name)  {

}

ProjectionGrouping::~ProjectionGrouping() {
}

void ProjectionGrouping::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzrgb_model", &in_cloud_xyzrgb_model);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzsift_model", &in_cloud_xyzsift_model);
	registerStream("in_clustered_correspondences", &in_clustered_correspondences);
	registerStream("in_rototranslations", &in_rototranslations);
    registerStream("out_projections", &out_projections);
    registerStream("out_model_bounding_box", &out_model_bounding_box);
	// Register handlers
    registerHandler("group", boost::bind(&ProjectionGrouping::group, this));
    //addDependency("group", &in_cloud_xyzsift);
	addDependency("group", &in_cloud_xyzsift_model);
	addDependency("group", &in_clustered_correspondences);
	addDependency("group", &in_rototranslations);

}

bool ProjectionGrouping::onInit() {

	return true;
}

bool ProjectionGrouping::onFinish() {
	return true;
}

bool ProjectionGrouping::onStop() {
	return true;
}

bool ProjectionGrouping::onStart() {
	return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectionGrouping::getBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    CLOG(LTRACE) << "ProjectionGrouping::getBoundingBox(PointXYZ)";
    pcl::PointCloud<pcl::PointXYZ>::Ptr bounding_box (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ minPt, maxPt, tmpPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    bounding_box->push_back(minPt);
    bounding_box->push_back(maxPt);
    float x = maxPt.x - minPt.x;
    float y = maxPt.y - minPt.y;
    float z = maxPt.z - minPt.z;

    tmpPt.x = minPt.x + x;
    tmpPt.y = minPt.y;
    tmpPt.z = minPt.z;
    bounding_box->push_back(tmpPt);
    tmpPt.x = minPt.x;
    tmpPt.y = minPt.y + y;
    tmpPt.z = minPt.z;
    bounding_box->push_back(tmpPt);
    tmpPt.x = minPt.x;
    tmpPt.y = minPt.y;
    tmpPt.z = minPt.z + z;
    bounding_box->push_back(tmpPt);
    tmpPt.x = minPt.x + x;
    tmpPt.y = minPt.y + y;
    tmpPt.z = minPt.z;
    bounding_box->push_back(tmpPt);
    tmpPt.x = minPt.x + x;
    tmpPt.y = minPt.y;
    tmpPt.z = minPt.z + z;
    bounding_box->push_back(tmpPt);
    tmpPt.x = minPt.x;
    tmpPt.y = minPt.y + y;
    tmpPt.z = minPt.z + z;
    bounding_box->push_back(tmpPt);

    return bounding_box;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectionGrouping::getBoundingBox(pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift){
    CLOG(LTRACE) << "ProjectionGrouping::getBoundingBox(PointXYZSIFT)";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*cloud_xyzsift, *cloud_xyz);
    return getBoundingBox(cloud_xyz);
}

void ProjectionGrouping::group() {
    CLOG(LTRACE) << "ProjectionGrouping::group";
    //Read data streams
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_model;
    //    if(!in_cloud_xyzrgb.empty())
    //        cloud_xyzrgb = in_cloud_xyzrgb.read();
    //    if(!in_cloud_xyzrgb_model.empty())
    //        cloud_xyzrgb_model = in_cloud_xyzrgb_model.read();
    //pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift_model = in_cloud_xyzsift_model.read();
    std::vector<pcl::Correspondences> clustered_correspondences = in_clustered_correspondences.read();
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = in_rototranslations.read();

    if(clustered_correspondences.size() != rototranslations.size()){
        CLOG(LERROR) << "Different number of clusters and translations";
        return;
    }

    //Get 8 points XYZ cloud bounding box of model
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_bounding_box = getBoundingBox(cloud_xyzsift_model);
    //Get model projections by transformations of model bounding box
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> model_projections;
    for(int i=0; i < rototranslations.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*model_bounding_box, *tmp, rototranslations[i]);
        model_projections.push_back(tmp);
    }

    //Get clusters projections by transformations of their bounding boxes
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_projections;
    for(int i=0; i < rototranslations.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
    //TODO get clusters xyz clouds
        for(int j=0; j < clustered_correspondences[i].size(); j++){
            pcl::PointXYZ tmpPt;
//            tmpPt.x = cloud_xyzsift_model[clustered_correspondences[i].at(j).index_query]->x;
//            tmpPt.y = cloud_xyzsift_model[clustered_correspondences[i].at(j).index_query]->y;
//            tmpPt.z = cloud_xyzsift_model[clustered_correspondences[i].at(j).index_query]->z;
//            tmp->push_back(tmpPt);
        }

        pcl::transformPointCloud(*tmp, *tmp, rototranslations[i]);
        clusters_projections.push_back(tmp);
    }

    out_model_bounding_box.write(model_bounding_box);
    out_projections.write(model_projections);
}



} //: namespace ProjectionGrouping
} //: namespace Processors
