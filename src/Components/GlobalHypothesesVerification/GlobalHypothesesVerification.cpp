/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "GlobalHypothesesVerification.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace GlobalHypothesesVerification {

GlobalHypothesesVerification::GlobalHypothesesVerification(const std::string & name) :
		Base::Component(name) , 
		resolution("resolution", 0.005f), 
		inlier_threshold("inlier_threshold", 0.005f), 
		radius_clutter("radius_clutter", 0.04f), 
		regularizer("regularizer", 3.f), 
		clutter_regularizer("clutter_regularizer", 5.f), 
		detect_clutter("detect_clutter", true) {
	registerProperty(resolution);
	registerProperty(inlier_threshold);
	registerProperty(radius_clutter);
	registerProperty(regularizer);
	registerProperty(clutter_regularizer);
	registerProperty(detect_clutter);

}

GlobalHypothesesVerification::~GlobalHypothesesVerification() {
}

void GlobalHypothesesVerification::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_aligned_hypotheses", &in_aligned_hypotheses);
	registerStream("in_cloud_xyzsift_scene", &in_cloud_xyzsift_scene);
    registerStream("out_verified_hypotheses", &out_verified_hypotheses);
    // Register handlers
	registerHandler("verify", boost::bind(&GlobalHypothesesVerification::verify, this));
	addDependency("verify", &in_aligned_hypotheses);
	addDependency("verify", &in_cloud_xyzsift_scene);

}

bool GlobalHypothesesVerification::onInit() {

	return true;
}

bool GlobalHypothesesVerification::onFinish() {
	return true;
}

bool GlobalHypothesesVerification::onStop() {
	return true;
}

bool GlobalHypothesesVerification::onStart() {
	return true;
}

void GlobalHypothesesVerification::verify() {
    CLOG(LTRACE) << "GlobalHypothesesVerification::verify";
    pcl::PointCloud<PointXYZSIFT>::Ptr scene = in_cloud_xyzsift_scene.read();
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "GlobalHypothesesVerification No hypotheses available";
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

    pcl::GlobalHypothesesVerification<pcl::PointXYZ, pcl::PointXYZ> go;
    go.setResolution (resolution);
    go.setInlierThreshold (inlier_threshold);
    go.setRadiusClutter (radius_clutter);
    go.setRegularizer (regularizer);
    go.setClutterRegularizer (clutter_regularizer);
    go.setDetectClutter (detect_clutter);
    go.setSceneCloud (scene_xyz);
    go.addModels (aligned_hypotheses_xyz, true);
    go.verify ();
    std::vector<bool> mask_hv;
    go.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "GlobalHypothesesVerification wrong vector size";
    }
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "GlobalHypothesesVerification: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "GlobalHypothesesVerification: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses.write(verified_hypotheses);

}



} //: namespace GlobalHypothesesVerification
} //: namespace Processors
