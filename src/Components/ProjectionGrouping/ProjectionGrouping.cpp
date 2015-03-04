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
        mcn("mcn", 10000) {
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

void ProjectionGrouping::threePointsToPlane (const pcl::PointXYZ &point_a,
                            const pcl::PointXYZ &point_b,
                            const pcl::PointXYZ &point_c,
                            const pcl::ModelCoefficients::Ptr plane){

  // Create Eigen plane through 3 points
  Eigen::Hyperplane<float, 3> eigen_plane =
    Eigen::Hyperplane<float, 3>::Through (point_a.getArray3fMap (),
                                                          point_b.getArray3fMap (),
                                                          point_c.getArray3fMap ());

  plane->values.resize (4);

  for (int i = 0; i < plane->values.size (); i++)
    plane->values[i] = eigen_plane.coeffs ()[i];
}


float ProjectionGrouping::cuboidIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid1, pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid2){
    CLOG(LTRACE) << "ProjectionGrouping::cuboidIntersection";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cuboids (new pcl::PointCloud<pcl::PointXYZ>);
    *cuboids = *cuboid1 + *cuboid2;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cuboids, minPt, maxPt);

    //Planes
    //0123 - 4567
    //0374 - 1265
    //0154 - 2376
    pcl::ModelCoefficients::Ptr plane11(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane12(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane13(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane14(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane15(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane16(new pcl::ModelCoefficients);
    threePointsToPlane(cuboid1->at(0), cuboid1->at(1), cuboid1->at(2), plane11);
    threePointsToPlane(cuboid1->at(4), cuboid1->at(5), cuboid1->at(6), plane12);
    threePointsToPlane(cuboid1->at(0), cuboid1->at(3), cuboid1->at(7), plane13);
    threePointsToPlane(cuboid1->at(1), cuboid1->at(2), cuboid1->at(6), plane14);
    threePointsToPlane(cuboid1->at(0), cuboid1->at(1), cuboid1->at(5), plane15);
    threePointsToPlane(cuboid1->at(2), cuboid1->at(3), cuboid1->at(7), plane16);
    pcl::ModelCoefficients::Ptr plane21(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane22(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane23(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane24(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane25(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane26(new pcl::ModelCoefficients);
    threePointsToPlane(cuboid2->at(0), cuboid2->at(1), cuboid2->at(2), plane21);
    threePointsToPlane(cuboid2->at(4), cuboid2->at(5), cuboid2->at(6), plane22);
    threePointsToPlane(cuboid2->at(0), cuboid2->at(3), cuboid2->at(7), plane23);
    threePointsToPlane(cuboid2->at(1), cuboid2->at(2), cuboid2->at(6), plane24);
    threePointsToPlane(cuboid2->at(0), cuboid2->at(1), cuboid2->at(5), plane25);
    threePointsToPlane(cuboid2->at(2), cuboid2->at(3), cuboid2->at(7), plane26);

    int inc2 = 0;
    int inboth = 0;
    float x, y, z;
    //Monte Carlo method
    srand(time(NULL));
    for(int i=0; i < mcn; i++){
        x = minPt.x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxPt.x-minPt.x)));
        y = minPt.y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxPt.y-minPt.y)));
        z = minPt.z + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxPt.z-minPt.z)));

        bool in2 = false;
        float dd21 = (plane21->values[0] * x) + (plane21->values[1] * y) + (plane21->values[2] * z) + (plane21->values[3]);
        float dd22 = (plane22->values[0] * x) + (plane22->values[1] * y) + (plane22->values[2] * z) + (plane22->values[3]);
        if( (dd21 >= 0 && dd22 <= 0) || (dd21 <= 0 && dd22 >= 0) ){ //is between planes 1 and 2
            float dd23 = (plane23->values[0] * x) + (plane23->values[1] * y) + (plane23->values[2] * z) + (plane23->values[3]);
            float dd24 = (plane24->values[0] * x) + (plane24->values[1] * y) + (plane24->values[2] * z) + (plane24->values[3]);
            if( (dd23 >= 0 && dd24 <= 0) || (dd23 <= 0 && dd24 >= 0)){ //is between planes 3 and 4
                float dd25 = (plane25->values[0] * x) + (plane25->values[1] * y) + (plane25->values[2] * z) + (plane25->values[3]);
                float dd26 = (plane26->values[0] * x) + (plane26->values[1] * y) + (plane26->values[2] * z) + (plane26->values[3]);
                if( (dd25 >= 0 && dd26 <= 0) || (dd25 <= 0 && dd26 >= 0) ){ //is between planes 5 and 6
                    inc2++;
                    in2 = true;
                }
            }
        }
        if(in2){
            float dd11 = (plane11->values[0] * x) + (plane11->values[1] * y) + (plane11->values[2] * z) + (plane11->values[3]);
            float dd12 = (plane12->values[0] * x) + (plane12->values[1] * y) + (plane12->values[2] * z) + (plane12->values[3]);
            if( (dd11 >= 0 && dd12 <= 0) || (dd11 <= 0 && dd12 >= 0) ){ //is between planes 1 and 2
                float dd13 = (plane13->values[0] * x) + (plane13->values[1] * y) + (plane13->values[2] * z) + (plane13->values[3]);
                float dd14 = (plane14->values[0] * x) + (plane14->values[1] * y) + (plane14->values[2] * z) + (plane14->values[3]);
                if( (dd13 >= 0 && dd14 <= 0) || (dd13 <= 0 && dd14 >= 0)){ //is between planes 3 and 4
                    float dd15 = (plane15->values[0] * x) + (plane15->values[1] * y) + (plane15->values[2] * z) + (plane15->values[3]);
                    float dd16 = (plane16->values[0] * x) + (plane16->values[1] * y) + (plane16->values[2] * z) + (plane16->values[3]);
                    if( (dd15 >= 0 && dd16 <= 0) || (dd15 <= 0 && dd16 >= 0) ){ //is between planes 5 and 6
                        inboth++;
                    }
                }
            }
        }//if(in2)


    }//for
    CLOG(LTRACE) << "ProjectionGrouping::cuboidIntersection: " << mcn << " random points, " << inc2 << " points in cuboid2, " << inboth << " points in both cuboids.";
    if(inc2 == 0)
        return 0;
    //% of cuboid2 is in cuboid1
    float r = (float)inboth / (float)inc2;
    return r;
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
            tmpPt.x = cloud_xyzsift_model->at(iq).x;
            tmpPt.y = cloud_xyzsift_model->at(iq).y;
            tmpPt.z = cloud_xyzsift_model->at(iq).z;
            tmp->push_back(tmpPt);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_bounding_box = getBoundingBox(tmp);
        pcl::transformPointCloud(*cluster_bounding_box, *cluster_bounding_box, rototranslations[i]);
        clusters_projections.push_back(cluster_bounding_box);
    }

    if(model_projections.size() >=2 && clusters_projections.size() >= 2){
        float f1 = cuboidIntersection(model_projections[0], clusters_projections[1]);
        cout << "wynik m0 c1: " <<f1<< endl;
        float f2 = cuboidIntersection(model_projections[1], clusters_projections[0]);
        cout << "wynik m1 c0: " <<f2<< endl;
        float f3 = cuboidIntersection(model_projections[1], model_projections[0]);
        cout << "wynik m1 m0: " <<f3<< endl;
    }
    else{
        cout<<"za mało danych" <<endl;
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
