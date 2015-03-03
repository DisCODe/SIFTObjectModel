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

    // \/A^2 +B^2 +C^2
    float p11 = sqrt( (plane11->values[0] * plane11->values[0]) + (plane11->values[1] * plane11->values[1]) + (plane11->values[2] * plane11->values[2]) );
    float p12 = sqrt( (plane12->values[0] * plane12->values[0]) + (plane12->values[1] * plane12->values[1]) + (plane12->values[2] * plane12->values[2]) );
    float p13 = sqrt( (plane13->values[0] * plane13->values[0]) + (plane13->values[1] * plane13->values[1]) + (plane13->values[2] * plane13->values[2]) );
    float p14 = sqrt( (plane14->values[0] * plane14->values[0]) + (plane14->values[1] * plane14->values[1]) + (plane14->values[2] * plane14->values[2]) );
    float p15 = sqrt( (plane15->values[0] * plane15->values[0]) + (plane15->values[1] * plane15->values[1]) + (plane15->values[2] * plane15->values[2]) );
    float p16 = sqrt( (plane16->values[0] * plane16->values[0]) + (plane16->values[1] * plane16->values[1]) + (plane16->values[2] * plane16->values[2]) );
    float p21 = sqrt( (plane21->values[0] * plane21->values[0]) + (plane21->values[1] * plane21->values[1]) + (plane21->values[2] * plane21->values[2]) );
    float p22 = sqrt( (plane22->values[0] * plane22->values[0]) + (plane22->values[1] * plane22->values[1]) + (plane22->values[2] * plane22->values[2]) );
    float p23 = sqrt( (plane23->values[0] * plane23->values[0]) + (plane23->values[1] * plane23->values[1]) + (plane23->values[2] * plane23->values[2]) );
    float p24 = sqrt( (plane24->values[0] * plane24->values[0]) + (plane24->values[1] * plane24->values[1]) + (plane24->values[2] * plane24->values[2]) );
    float p25 = sqrt( (plane25->values[0] * plane25->values[0]) + (plane25->values[1] * plane25->values[1]) + (plane25->values[2] * plane25->values[2]) );
    float p26 = sqrt( (plane26->values[0] * plane26->values[0]) + (plane26->values[1] * plane26->values[1]) + (plane26->values[2] * plane26->values[2]) );

    //różnica między płaszczyznami
    float diff112 = abs( plane11->values[3] - plane12->values[3]);
    float diff134 = abs( plane13->values[3] - plane14->values[3]);
    float diff156 = abs( plane15->values[3] - plane16->values[3]);
    float diff212 = abs( plane21->values[3] - plane22->values[3]);
    float diff234 = abs( plane23->values[3] - plane24->values[3]);
    float diff256 = abs( plane25->values[3] - plane26->values[3]);

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

        bool in1 = false;

        float dd11 = abs( (plane11->values[0] * x) + (plane11->values[1] * y) + (plane11->values[2] * z) + (plane11->values[3]));
        float dd12 = abs( (plane12->values[0] * x) + (plane12->values[1] * y) + (plane12->values[2] * z) + (plane12->values[3]));
        if( (dd11/p11) + (dd12/p12) - diff112 < 0.001){ //czy jest pomiedzy plaszczyznami 1 i 2
            float dd13 = abs( (plane13->values[0] * x) + (plane13->values[1] * y) + (plane13->values[2] * z) + (plane13->values[3]));
            float dd14 = abs( (plane14->values[0] * x) + (plane14->values[1] * y) + (plane14->values[2] * z) + (plane14->values[3]));
            if( (dd13/p13) + (dd14/p14) - diff134 < 0.001){
                float dd15 = abs( (plane15->values[0] * x) + (plane15->values[1] * y) + (plane13->values[2] * z) + (plane15->values[3]));
                float dd16 = abs( (plane16->values[0] * x) + (plane16->values[1] * y) + (plane14->values[2] * z) + (plane16->values[3]));
                if( (dd15/p15) + (dd16/p16) - diff156 < 0.001){
                    inc1++;
                    in1 = true;
                }
            }
        }

        float dd21 = abs( (plane21->values[0] * x) + (plane21->values[1] * y) + (plane21->values[2] * z) + (plane21->values[3]));
        float dd22 = abs( (plane22->values[0] * x) + (plane22->values[1] * y) + (plane22->values[2] * z) + (plane22->values[3]));
        if( (dd21/p21) + (dd22/p22) - diff212 < 0.001){
            float dd23 = abs( (plane23->values[0] * x) + (plane23->values[1] * y) + (plane23->values[2] * z) + (plane23->values[3]));
            float dd24 = abs( (plane24->values[0] * x) + (plane24->values[1] * y) + (plane24->values[2] * z) + (plane24->values[3]));
            if( (dd23/p23) + (dd24/p24) - diff234 < 0.001){
                float dd25 = abs( (plane25->values[0] * x) + (plane25->values[1] * y) + (plane23->values[2] * z) + (plane25->values[3]));
                float dd26 = abs( (plane26->values[0] * x) + (plane26->values[1] * y) + (plane24->values[2] * z) + (plane26->values[3]));
                if( (dd25/p25) + (dd26/p26) - diff256 < 0.001){
                    inc2++;
                    if(in1)
                        inboth++;
                }
            }
        }
    }

    cout<< "inc1 " <<inc1 << " inc2 " << inc2 << " inboth " << inboth <<endl;
    if(inc1 == 0)
        return -1;

    float r = inboth / inc1;
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
