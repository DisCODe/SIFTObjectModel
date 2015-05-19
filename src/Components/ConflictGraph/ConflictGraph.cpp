/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "ConflictGraph.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ConflictGraph {

ConflictGraph::ConflictGraph(const std::string & name) :
		Base::Component(name) , 
		resolution("resolution", 0.005f), 
		inlier_treshold("inlier_treshold", 0.005f), 
		support_threshold("support_threshold", 0.08f), 
		penalty_threshold("penalty_threshold", 0.05f), 
		conflict_threshold("conflict_threshold", 0.02f) {
	registerProperty(resolution);
	registerProperty(inlier_treshold);
	registerProperty(support_threshold);
	registerProperty(penalty_threshold);
	registerProperty(conflict_threshold);

}

ConflictGraph::~ConflictGraph() {
}

void ConflictGraph::prepareInterface() {
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
    registerHandler("verify_xyzsift", boost::bind(&ConflictGraph::verify_xyzsift, this));
    addDependency("verify_xyzsift", &in_aligned_hypotheses_xyzsift);
    addDependency("verify_xyzsift", &in_cloud_xyzsift_scene);

    registerHandler("verify_xyzrgb", boost::bind(&ConflictGraph::verify_xyzrgb, this));
    addDependency("verify_xyzrgb", &in_aligned_hypotheses_xyzrgb);
    addDependency("verify_xyzrgb", &in_cloud_xyzrgb_scene);

    registerHandler("verify", boost::bind(&ConflictGraph::verify, this));
    addDependency("verify", &in_aligned_hypotheses_xyz);
    addDependency("verify", &in_cloud_xyz_scene);
}

bool ConflictGraph::onInit() {

	return true;
}

bool ConflictGraph::onFinish() {
	return true;
}

bool ConflictGraph::onStop() {
	return true;
}

bool ConflictGraph::onStart() {
	return true;
}

void ConflictGraph::verify() {
    CLOG(LTRACE) << "ConflictGraph::verify";
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = in_cloud_xyz_scene.read();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses_xyz.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "ConflictGraph No hypotheses available";
        return;
    }


    pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> papazov;
    papazov.setResolution (resolution);
    papazov.setInlierThreshold (inlier_treshold);
    papazov.setSupportThreshold (support_threshold);
    papazov.setPenaltyThreshold (penalty_threshold);
    papazov.setConflictThreshold (conflict_threshold);
    papazov.setSceneCloud (scene);
    papazov.addModels (aligned_hypotheses, true);
    papazov.verify ();
    std::vector<bool> mask_hv;
    papazov.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "ConflictGraph wrong vector size";
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "ConflictGraph: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "ConflictGraph: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses_xyz.write(verified_hypotheses);
}


void ConflictGraph::verify_xyzsift() {
    CLOG(LTRACE) << "ConflictGraph::verify_xyzsift";
    pcl::PointCloud<PointXYZSIFT>::Ptr scene = in_cloud_xyzsift_scene.read();
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses_xyzsift.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "ConflictGraph No hypotheses available";
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

    pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> papazov;
    papazov.setResolution (resolution);
    papazov.setInlierThreshold (inlier_treshold);
    papazov.setSupportThreshold (support_threshold);
    papazov.setPenaltyThreshold (penalty_threshold);
    papazov.setConflictThreshold (conflict_threshold);
    papazov.setSceneCloud (scene_xyz);
    papazov.addModels (aligned_hypotheses_xyz, true);
    papazov.verify ();
    std::vector<bool> mask_hv;
    papazov.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "ConflictGraph wrong vector size";
    }
    std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "ConflictGraph: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "ConflictGraph: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses_xyzsift.write(verified_hypotheses);
}

void ConflictGraph::verify_xyzrgb() {
    CLOG(LTRACE) << "ConflictGraph::verify_xyzrgb";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene = in_cloud_xyzrgb_scene.read();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_hypotheses = in_aligned_hypotheses_xyzrgb.read();

    if(aligned_hypotheses.size() == 0){
        CLOG(LINFO) << "ConflictGraph No hypotheses available";
        return;
    }


    pcl::PapazovHV<pcl::PointXYZRGB, pcl::PointXYZRGB> papazov;
    papazov.setResolution (resolution);
    papazov.setInlierThreshold (inlier_treshold);
    papazov.setSupportThreshold (support_threshold);
    papazov.setPenaltyThreshold (penalty_threshold);
    papazov.setConflictThreshold (conflict_threshold);
    papazov.setSceneCloud (scene);
    papazov.addModels (aligned_hypotheses, true);
    papazov.verify ();
    std::vector<bool> mask_hv;
    papazov.getMask (mask_hv);

    if(mask_hv.size() != aligned_hypotheses.size()){
       CLOG(LERROR) << "ConflictGraph wrong vector size";
    }
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> verified_hypotheses;
    for(int i = 0; i < mask_hv.size(); i++){
        if(mask_hv[i]){
            verified_hypotheses.push_back(aligned_hypotheses[i]);
            CLOG(LINFO) << "ConflictGraph: Hypothese " << i << " is CORRECT";
        }
        else{
            CLOG(LINFO) << "ConflictGraph: Hypothese " << i << " is NOT correct";
        }

    }

    out_verified_hypotheses_xyzrgb.write(verified_hypotheses);
}

} //: namespace ConflictGraph
} //: namespace Processors
