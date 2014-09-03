/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>
#include <iomanip>

#include "SIFTObjectMatcher.hpp"
#include "Common/Logger.hpp"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h> 
#include <pcl/search/impl/kdtree.hpp>
#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Types/Features.hpp"
#include <pcl/recognition/cg/hough_3d.h>

//
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

namespace Processors {
namespace SIFTObjectMatcher {

class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
    using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;
    public:
    SIFTFeatureRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 128 ;
        trivial_ = false ;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointXYZSIFT &p, float * out) const
    {
        //This representation is only for determining correspondences (not for use in Kd-tree for example - so use only SIFT part of the point
        for (register int i = 0; i < 128 ; i++)
            out[i] = p.descriptor[i];//p.descriptor.at<float>(0, i) ;
        //std::cout << "SIFTFeatureRepresentation:copyToFloatArray()" << std::endl ;
    }
};

SIFTObjectMatcher::SIFTObjectMatcher(const std::string & name) :
		Base::Component(name),
		threshold("threshold", 0.75f),
        inlier_threshold("inlier_threshold", 0.001f),
        max_distance("max_distance", 150),
        cg_thresh("cg_thresh", 5.0f){
			registerProperty(threshold);
			registerProperty(inlier_threshold);
            registerProperty(max_distance);
            registerProperty(cg_thresh);
}

SIFTObjectMatcher::~SIFTObjectMatcher() {
}

void SIFTObjectMatcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_models", &in_models);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb_model", &out_cloud_xyzrgb_model);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzsift_model", &out_cloud_xyzsift_model);
    registerStream("out_correspondences", &out_correspondences);
    registerStream("out_good_correspondences", &out_good_correspondences);
    registerStream("out_clustered_correspondences", &out_clustered_correspondences);

	// Register handlers
	h_readModels.setup(boost::bind(&SIFTObjectMatcher::readModels, this));
	registerHandler("readModels", &h_readModels);
	addDependency("readModels", &in_models);
	h_match.setup(boost::bind(&SIFTObjectMatcher::match, this));
	registerHandler("match", &h_match);
	addDependency("match", &in_cloud_xyzsift);
	addDependency("match", &in_cloud_xyzrgb);

}

bool SIFTObjectMatcher::onInit() {

	return true;
}

bool SIFTObjectMatcher::onFinish() {
	return true;
}

bool SIFTObjectMatcher::onStop() {
	return true;
}

bool SIFTObjectMatcher::onStart() {
	return true;
}

void SIFTObjectMatcher::readModels() {
    CLOG(LTRACE) << "readModels()" << endl;
	for( int i = 0 ; i<models.size(); i++){
		delete models[i];
	}
	models.clear();
	std::vector<AbstractObject*> abstractObjects = in_models.read();
	for( int i = 0 ; i<abstractObjects.size(); i++){
        CLOG(LTRACE)<<"Name: "<<abstractObjects[i]->name<<endl;
		SIFTObjectModel *model = dynamic_cast<SIFTObjectModel*>(abstractObjects[i]);
		if(model!=NULL)
			models.push_back(model);
		else
            CLOG(LTRACE) << "niepoprawny model" << endl;
	}
    CLOG(LTRACE) << models.size() << " modeli" << endl;
}

void SIFTObjectMatcher::match() {
	CLOG(LTRACE) << "SIFTObjectMatcher::match()"<<endl;
	if(models.empty()){
		cout<<"No models available" <<endl;
		return;
	}
			
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();	
	

		for (int i = 0 ; i<models.size(); i++){
			CLOG(LTRACE) << "liczba cech modelu "<<i<<" "<<models[i]->name<<": " <<
				models[i]->cloud_xyzsift->size()<<endl; 	
		}
		CLOG(LTRACE) << "liczba cech instancji : " <<
			cloud_xyzsift->size()<<endl; 


        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences()) ;
        pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst ;

        //SIFTFeatureRepresentation point_representation ;
        //correst.setPointRepresentation (point_representation.makeShared()); //NEVER do like this, makeShared will return DefaultFeatureRepresentation<PointDefault>!
        SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
        correst.setPointRepresentation(point_representation) ;
        for (int i = 0 ; i<models.size(); i++){

            pcl::KdTreeFLANN<PointXYZSIFT> match_search;
            match_search.setPointRepresentation(point_representation);
            match_search.setInputCloud (models[i]->cloud_xyzsift);

            //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
            for (size_t j = 0; j < cloud_xyzsift->size (); ++j)
            {
              std::vector<int> neigh_indices (1);
              std::vector<float> neigh_sqr_dists (1);
              if (!pcl_isfinite (cloud_xyzsift->at (j).descriptor[0])) //skipping NaNs
              {
                continue;
              }
              int found_neighs = match_search.nearestKSearch (cloud_xyzsift->at (j), 1, neigh_indices, neigh_sqr_dists);
              if(found_neighs == 1 && neigh_sqr_dists[0] < max_distance) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
              {
                pcl::Correspondence corr (neigh_indices[0], static_cast<int> (j), neigh_sqr_dists[0]);
//                pcl::Correspondence corr2 (static_cast<int> (j), neigh_indices[0], neigh_sqr_dists[0]);
                correspondences->push_back (corr);
              }
            }
            std::cout << "Correspondences found: " << correspondences->size () << std::endl;
//            correst.setInputSource(cloud_xyzsift) ;
//            correst.setInputTarget(models[i]->cloud_xyzsift) ;
//            correst.determineReciprocalCorrespondences(*correspondences, max_distance) ;
//			//ransac - niepoprawne dopasowania
//            pcl::CorrespondencesPtr inliers (new pcl::Correspondences());
//        	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> sac ;
//			sac.setInputSource(cloud_xyzsift) ;
//			sac.setInputTarget(models[i]->cloud_xyzsift) ;
//			sac.setInlierThreshold(inlier_threshold) ;
//			sac.setMaximumIterations(2000) ;
//			sac.setInputCorrespondences(correspondences) ;
//            sac.getCorrespondences(*inliers) ;
//			//std::cout << "SAC inliers " << inliers.size() << std::endl ;
//            Eigen::Matrix4f  trans = sac.getBestTransformation();
//            if (trans==Eigen::Matrix4f::Identity()){
//                CLOG(LTRACE)  << "0 good correspondences!!" <<endl;
//            }
//            else{
//                CLOG(LTRACE) << setprecision(2) << fixed;
//                float percent = (float)inliers->size()/(float)models[i]->mean_viewpoint_features_number;
//                CLOG(LTRACE)  << "\nNumber of reciprocal correspondences: " << correspondences->size() <<". Good correspondences:" << inliers->size() ;
//                CLOG(LTRACE)  << " out of " << cloud_xyzsift->size() << " keypoints of instance, = " ;
//                CLOG(LTRACE)  << percent << ". "<< models[i]->cloud_xyzsift->size() << " keypoints of model "<< models[i]->name << std::endl ;
//                if (percent > threshold)
//                    std::cout <<"Rozpoznano model "<< models[i]->name<<endl;
//            }
//        /////////////////////////////
        out_cloud_xyzrgb.write(cloud_xyzrgb);
        out_cloud_xyzrgb_model.write(models[i]->cloud_xyzrgb);
        out_cloud_xyzsift.write(cloud_xyzsift);
        out_cloud_xyzsift_model.write(models[i]->cloud_xyzsift);
        out_correspondences.write(correspondences);//wszystkie dopasowania
//        out_good_correspondences.write(inliers);


        //Algorithm params
        float rf_rad_ (0.015f);
        float cg_size_ (inlier_threshold);//(0.01f);
        float cg_thresh_ (cg_thresh);
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;
//        clustered_corrs.resize(10000);
//        rototranslations.resize(10000);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_model (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_instance (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(models[i]->cloud_xyzsift),*cloud_xyz_model);
        pcl::copyPointCloud(*cloud_xyzsift,*cloud_xyz_instance);
        //  Clustering
        if(false){
            pcl::Hough3DGrouping<PointXYZSIFT, PointXYZSIFT, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
            clusterer.setHoughBinSize (cg_size_);
            clusterer.setHoughThreshold (cg_thresh_);
            clusterer.setUseInterpolation (true);
            clusterer.setUseDistanceWeight (false);

            clusterer.setLocalRfSearchRadius(rf_rad_);
            clusterer.setInputCloud (models[i]->cloud_xyzsift);
            //clusterer.setInputRf (model_rf);
            clusterer.setSceneCloud (cloud_xyzsift);
            //clusterer.setSceneRf (scene_rf);
            clusterer.setModelSceneCorrespondences (correspondences);

            clusterer.cluster (clustered_corrs);//tu sie wywala
    //      clusterer.recognize (rototranslations, clustered_corrs);//tu sie wywala

            std::cout << "Model instances found: " << rototranslations.size () << std::endl;
            for (size_t i = 0; i < rototranslations.size (); ++i)
            {
                Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
                Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
                if(rotation != Eigen::Matrix3f::Identity()){
                    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
                    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

                  // Print the rotation matrix and translation vector
                    printf ("\n");
                    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
                    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
                    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
                    printf ("\n");
                    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
                }
            }
        }
///////////////////////////
        else{
        // Using GeometricConsistency0
            pcl::GeometricConsistencyGrouping<PointXYZSIFT, PointXYZSIFT> gc_clusterer;
            gc_clusterer.setGCSize (cg_size_);
            gc_clusterer.setGCThreshold (cg_thresh_);
            cout<< "model " << cloud_xyz_model->size() <<" scena "<<cloud_xyz_instance->size()<<" corrs "<<correspondences->size()<<endl;
            gc_clusterer.setInputCloud (models[i]->cloud_xyzsift);//cloud_xyz_model);
            gc_clusterer.setSceneCloud (cloud_xyzsift);//cloud_xyz_instance);
            gc_clusterer.setModelSceneCorrespondences (correspondences);

            //gc_clusterer.cluster (clustered_corrs);
            gc_clusterer.recognize (rototranslations, clustered_corrs);
            cout<<"clustered_corrs "<< clustered_corrs.size()<<endl;
            for(int j=0; j<clustered_corrs.size(); j++){
                for(int k =0; k<clustered_corrs[j].size();k++){
                    cout<<clustered_corrs[j][k].index_query<<"-"<<clustered_corrs[j][k].index_match<<", ";
                }
                cout<<endl<<"-----------"<<endl;
            }
                for (size_t k = 0; k < rototranslations.size (); ++k){
                      Eigen::Matrix3f rotation = rototranslations[k].block<3,3>(0, 0);
                      Eigen::Vector3f translation = rototranslations[k].block<3,1>(0, 3);
                      if(rotation != Eigen::Matrix3f::Identity()){
                          std::cout << "\n    Instance " << k + 1 << ":" << std::endl;
                          std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[k].size () << std::endl;
                          //Print the rotation matrix and translation vector
                          printf ("\n");
                          printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
                          printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
                          printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
                          printf ("\n");
                          printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
                      }
                }
            }

            out_clustered_correspondences.write(clustered_corrs);
        }
}



} //: namespace SIFTObjectMatcher
} //: namespace Processors
