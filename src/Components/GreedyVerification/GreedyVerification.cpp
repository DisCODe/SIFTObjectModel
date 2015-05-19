/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "GreedyVerification.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace GreedyVerification {

GreedyVerification::GreedyVerification(const std::string & name) :
		Base::Component(name) , 
        resolution("resolution", 0.005f),
        inlier_treshold("inlier_treshold", 0.005f),
        lambda("lambda", 1.5f){
	registerProperty(resolution);
	registerProperty(inlier_treshold);
    registerProperty(lambda);

}

GreedyVerification::~GreedyVerification() {
}

void GreedyVerification::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_aligned_hypotheses_xyzsift", &in_aligned_hypotheses_xyzsift);
    registerStream("in_cloud_xyzsift_scene", &in_cloud_xyzsift_scene);
    registerStream("out_verified_hypotheses_xyzsift", &out_verified_hypotheses_xyzsift);
    registerStream("in_aligned_hypotheses_xyzrgb", &in_aligned_hypotheses_xyzrgb);
    registerStream("in_cloud_xyzrgb_scene", &in_cloud_xyzrgb_scene);
    registerStream("out_verified_hypotheses_xyzrgb", &out_verified_hypotheses_xyzrgb);
    registerStream("in_aligned_hypotheses_xyz", &in_aligned_hypotheses_xyz);
    registerStream("in_cloud_xyz_scene", &in_cloud_xyz_scene);
    registerStream("out_verified_hypotheses_xyz", &out_verified_hypotheses_xyz);
	// Register handlers
    registerHandler("verify_xyzsift", boost::bind(&GreedyVerification::verify_xyzsift, this));
    addDependency("verify_xyzsift", &in_aligned_hypotheses_xyzsift);
    addDependency("verify_xyzsift", &in_cloud_xyzsift_scene);
    registerHandler("verify_xyzrgb", boost::bind(&GreedyVerification::verify_xyzrgb, this));
    addDependency("verify_xyzrgb", &in_aligned_hypotheses_xyzrgb);
    addDependency("verify_xyzrgb", &in_cloud_xyzrgb_scene);
    registerHandler("verify", boost::bind(&GreedyVerification::verify, this));
    addDependency("verify", &in_aligned_hypotheses_xyz);
    addDependency("verify", &in_cloud_xyz_scene);
}

bool GreedyVerification::onInit() {

	return true;
}

bool GreedyVerification::onFinish() {
	return true;
}

bool GreedyVerification::onStop() {
	return true;
}

bool GreedyVerification::onStart() {
	return true;
}


void GreedyVerification::verify() {
    CLOG(LTRACE) << "GreedyVerification::verify";
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = in_cloud_xyz_scene.read();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses_xyz.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "GreedyVerification No hypotheses available";
        return;
    }

    pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> greedy_hv(lambda);
    greedy_hv.setResolution (resolution);
    greedy_hv.setInlierThreshold (inlier_treshold);
    greedy_hv.setSceneCloud (scene);
    greedy_hv.addModels (aligned_hypotheses, true);
    greedy_hv.verify ();
    std::vector<bool> mask_hv;
    greedy_hv.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "GreedyVerification wrong vector size";
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses_xyz.write(verified_hypotheses);
}

void GreedyVerification::verify_xyzsift() {
    CLOG(LTRACE) << "GreedyVerification::verify_xyzsift";
    pcl::PointCloud<PointXYZSIFT>::Ptr scene = in_cloud_xyzsift_scene.read();
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses_xyzsift.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "GreedyVerification No hypotheses available";
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*scene, *scene_xyz);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> aligned_hypotheses_xyz;
    for(int i = 0; i < aligned_hypotheses.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(aligned_hypotheses[i]), *cloud );
        aligned_hypotheses_xyz.push_back(cloud);
    }

    pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> greedy_hv(lambda);
    greedy_hv.setResolution (resolution);
    greedy_hv.setInlierThreshold (inlier_treshold);
    greedy_hv.setSceneCloud (scene_xyz);
    greedy_hv.addModels (aligned_hypotheses_xyz, true);
    greedy_hv.verify ();
    std::vector<bool> mask_hv;
    greedy_hv.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "GreedyVerification wrong vector size";
    }
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses_xyzsift.write(verified_hypotheses);
}

void GreedyVerification::verify_xyzrgb() {
    CLOG(LTRACE) << "GreedyVerification::verify_xyzrgb";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene = in_cloud_xyzrgb_scene.read();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses_xyzrgb.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "GreedyVerification No hypotheses available";
        return;
    }

    pcl::GreedyVerification<pcl::PointXYZRGB, pcl::PointXYZRGB> greedy_hv(lambda);
    greedy_hv.setResolution (resolution);
    greedy_hv.setInlierThreshold (inlier_treshold);
    greedy_hv.setSceneCloud (scene);
    greedy_hv.addModels (aligned_hypotheses, true);
    greedy_hv.verify ();
    std::vector<bool> mask_hv;
    greedy_hv.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "GreedyVerification wrong vector size";
    }
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "GreedyVerification: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses_xyzrgb.write(verified_hypotheses);
}


} //: namespace GreedyVerification
} //: namespace Processors
