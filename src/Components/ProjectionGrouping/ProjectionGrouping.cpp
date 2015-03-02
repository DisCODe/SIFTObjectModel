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
        Base::Component(name),
        mcn("mcn", 1000) {
            registerProperty(mcn);
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


    float x = maxPt.x - minPt.x;
    float y = maxPt.y - minPt.y;
    float z = maxPt.z - minPt.z;

    bounding_box->push_back(minPt); //0
    tmpPt.x = minPt.x + x;
    tmpPt.y = minPt.y;
    tmpPt.z = minPt.z;
    bounding_box->push_back(tmpPt); //1
    tmpPt.x = minPt.x + x;
    tmpPt.y = minPt.y + y;
    tmpPt.z = minPt.z;
    bounding_box->push_back(tmpPt); //2
    tmpPt.x = minPt.x;
    tmpPt.y = minPt.y + y;
    tmpPt.z = minPt.z;
    bounding_box->push_back(tmpPt); //3
    tmpPt.x = minPt.x;
    tmpPt.y = minPt.y;
    tmpPt.z = minPt.z + z;
    bounding_box->push_back(tmpPt); //4
    tmpPt.x = minPt.x + x;
    tmpPt.y = minPt.y;
    tmpPt.z = minPt.z + z;
    bounding_box->push_back(tmpPt); //5
    bounding_box->push_back(maxPt); //6
    tmpPt.x = minPt.x;
    tmpPt.y = minPt.y + y;
    tmpPt.z = minPt.z + z;
    bounding_box->push_back(tmpPt); //7

    return bounding_box;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectionGrouping::getBoundingBox(pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift){
    CLOG(LTRACE) << "ProjectionGrouping::getBoundingBox(PointXYZSIFT)";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*cloud_xyzsift, *cloud_xyz);
    return getBoundingBox(cloud_xyz);
}

float ProjectionGrouping::cuboidIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid1, pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid2){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cuboids (new pcl::PointCloud<pcl::PointXYZ>);
    *cuboids = *cuboid1 + *cuboid2;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cuboids, minPt, maxPt);

    int inc1 = 0;
    int inc2 = 0;
    int inboth = 0;
    float x, y, z;
    //Monte Carlo method
    srand(time(NULL));
    for(int i=0; i < mcn; i++){
        x = minPt.x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxPt.x-minPt.x)));
        y = minPt.y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxPt.y-minPt.y)));
        z = minPt.z + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxPt.z-minPt.z)));
    }

    return 0;
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
    for(int i=0; i < clustered_correspondences.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
        for(int j=0; j < clustered_correspondences[i].size(); j++){
            pcl::PointXYZ tmpPt;
            int iq = clustered_correspondences[i][j].index_query;
            //cout << "iq " << iq<<endl;
            tmpPt.x = cloud_xyzsift_model->at(iq).x;
            tmpPt.y = cloud_xyzsift_model->at(iq).y;
            tmpPt.z = cloud_xyzsift_model->at(iq).z;
            tmp->push_back(tmpPt);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_bounding_box = getBoundingBox(tmp);
        pcl::transformPointCloud(*cluster_bounding_box, *cluster_bounding_box, rototranslations[i]);
        clusters_projections.push_back(cluster_bounding_box);
    }

    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> projections;
    //model projections
    projections.insert(projections.end(), model_projections.begin(), model_projections.end());
    //clusters projections
    projections.insert(projections.end(), clusters_projections.begin(), clusters_projections.end());

    out_model_bounding_box.write(model_bounding_box);
    out_projections.write(projections);
}



} //: namespace ProjectionGrouping
} //: namespace Processors
