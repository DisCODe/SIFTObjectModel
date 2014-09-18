/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CuboidModelGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#if (CV_MAJOR_VERSION == 2)
#if (CV_MINOR_VERSION > 3)
#include <opencv2/nonfree/features2d.hpp>
#endif
#endif


using boost::property_tree::ptree;
using boost::property_tree::read_json;

namespace Processors {
namespace CuboidModelGenerator {

CuboidModelGenerator::CuboidModelGenerator(const std::string & name) :
		Base::Component(name) , 
        dataJSONname("dataJSONname", std::string("./")) {
    registerProperty(dataJSONname);
    generateModel_flag = false;
}

CuboidModelGenerator::~CuboidModelGenerator() {
}

void CuboidModelGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_model", &out_model);
    registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);

    // Register handlers
    h_generateModel.setup(boost::bind(&CuboidModelGenerator::generateModelPressed, this));
    registerHandler("generateModel", &h_generateModel);

    h_returnModel.setup(boost::bind(&CuboidModelGenerator::returnModel, this));
    registerHandler("returnModel", &h_returnModel);
    addDependency("returnModel", NULL);
}

bool CuboidModelGenerator::onInit() {
    CLOG(LTRACE) << "CuboidModelGenerator::onInit";

    generate_top = generate_bottom = generate_left = generate_right = generate_front = generate_back =
        mask_top = mask_bottom = mask_left = mask_right = mask_front = mask_back = false;
    cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	return true;
}

bool CuboidModelGenerator::onFinish() {
	return true;
}

bool CuboidModelGenerator::onStop() {
	return true;
}

bool CuboidModelGenerator::onStart() {
	return true;
}

void CuboidModelGenerator::sift(cv::Mat input, cv::Mat &descriptors, Types::Features &features) {
    CLOG(LTRACE) << "CuboidModelGenerator::sift";
    try {
        //-- Step 1: Detect the keypoints.
        cv::SiftFeatureDetector detector;
        std::vector<cv::KeyPoint> keypoints;
        detector.detect(input, keypoints);

        //-- Step 2: Calculate descriptors (feature vectors).
        cv::SiftDescriptorExtractor extractor;
        extractor.compute( input, keypoints, descriptors);

        features = Types::Features(keypoints);
    } catch (...) {
        CLOG(LERROR) << "CuboidModelGenerator::sift() failed\n";
    }
}

void CuboidModelGenerator::loadData(){
    CLOG(LTRACE) << "CuboidModelGenerator::loadData";
    ptree ptree_file;
    std::string model_name;
    std::string left_name;
    std::string right_name;
    std::string top_name;
    std::string bottom_name;
    std::string front_name;
    std::string back_name;
    std::string left_mask_name;
    std::string right_mask_name;
    std::string top_mask_name;
    std::string bottom_mask_name;
    std::string front_mask_name;
    std::string back_mask_name;
    try{
        // Open JSON file and load it to ptree.
        read_json(dataJSONname, ptree_file);
        // Read JSON properties.
        model_name = ptree_file.get("name","");
        left_name = ptree_file.get("left","");
        right_name = ptree_file.get("right","");
        top_name = ptree_file.get("top","");
        bottom_name = ptree_file.get("bottom","");
        front_name = ptree_file.get("front","");
        back_name = ptree_file.get("back","");
        left_mask_name = ptree_file.get("left","");
        right_mask_name = ptree_file.get("right","");
        top_mask_name = ptree_file.get("top","");
        bottom_mask_name = ptree_file.get("bottom","");
        front_mask_name = ptree_file.get("front","");
        back_mask_name = ptree_file.get("back","");

        a = ptree_file.get<int>("a");
        b = ptree_file.get<int>("b");
        c = ptree_file.get<int>("c");
    }//: try
    catch(std::exception const& e){
        LOG(LERROR) << "SOMJSONReader: file "<< dataJSONname <<" not found or invalid\n";
        return;
    }//: catch
    if(left_name!=""){
        left = cv::imread(left_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        generate_left = true;
    }
    if(right_name!=""){
        right = cv::imread(right_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        generate_right = true;
    }
    if(top_name!=""){
        top = cv::imread(top_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        generate_top = true;
    }
    if(bottom_name!=""){
        bottom = cv::imread(bottom_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        generate_bottom = true;
    }
    if(front_name!=""){
        front = cv::imread(front_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        generate_front =  true;
    }
    if(back_name!=""){
        back = cv::imread(back_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        generate_back = true;
    }
    if(left_mask_name!=""){
        left_mask = cv::imread(left_mask_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        mask_left = true;
    }
    if(right_mask_name!=""){
        right_mask = cv::imread(right_mask_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        mask_right = true;
    }
    if(top_mask_name!=""){
        top_mask = cv::imread(top_mask_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        mask_top = true;
    }
    if(bottom_mask_name!=""){
        bottom_mask = cv::imread(bottom_mask_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        mask_bottom = true;
    }
    if(front_mask_name!=""){
        front_mask = cv::imread(front_mask_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        mask_front =  true;
    }
    if(back_mask_name!=""){
        back_mask = cv::imread(back_mask_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        mask_back = true;
    }
}


void CuboidModelGenerator::returnModel() {
    CLOG(LTRACE) << "CuboidModelGenerator::returnModel()";
    // Generate model if required.
    if (generateModel_flag) {
        generateModel();
        generateModel_flag = false;
    }

    // Write to output clouds and SOM.
    out_cloud_xyzrgb.write(cloud_xyzrgb);
    out_cloud_xyzsift.write(cloud_xyzsift);
    out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);

    SIFTObjectModel* model;
    model = dynamic_cast<SIFTObjectModel*>(produce());
    out_model.write(model);
}


void CuboidModelGenerator::generateModelPressed() {
    CLOG(LTRACE) << "CuboidModelGenerator::generateModelPressed";
    generateModel_flag = true;
}


void CuboidModelGenerator::generateModel() {
    CLOG(LTRACE) << "CuboidModelGenerator::generateModel";
    loadData();

    // Clear clouds.
    cloud_xyzrgb->clear();
    cloud_xyzsift->clear();

    int x,y,z;
    //front
    if(generate_front){
        y=0;//stałe
        CLOG(LTRACE) <<"front " << front.cols << " x " <<front.rows << endl;
        for(x = 0; x < a; x++){
            for(z = 0; z < c; z++){
                pcl::PointXYZRGB point;
                point.x = float(a-x)/1000;
                point.y = float(y)/1000;
                point.z = float(c-z)/1000;
                //pozycja w obrazie
                int xx = 0 + (x-0)*(front.cols-1-0)/(a-1-0);
                int zz = 0 + (z-0)*(front.rows-1-0)/(c-1-0);
                if (mask_front && front_mask.at<float>(zz,xx)==0) {
                        continue;
                }
                cv::Vec3b bgr = front.at<cv::Vec3b>(zz, xx);
                point.r = bgr[2];
                point.g = bgr[1];
                point.b = bgr[0];
                cloud_xyzrgb->push_back(point);
            }

        }
    }
    //back
    if(generate_back){
        y=-b;//stale
        CLOG(LTRACE) <<"back " << back.cols << " x " <<back.rows << endl;
        for(x = 0; x < a; x++){
            for(z = 0; z < c; z++){
                pcl::PointXYZRGB point;
                point.x = float(x)/1000;
                point.y = float(y)/1000;
                point.z = float(c-z)/1000;
                int xx = 0 + (x-0)*(back.cols-1-0)/(a-1-0);
                int zz = 0 + (z-0)*(back.rows-1-0)/(c-1-0);
                if (mask_back && back_mask.at<float>(zz, xx)==0) {
                        continue;
                }
                cv::Vec3b bgr = back.at<cv::Vec3b>(zz, xx);
                point.r = bgr[2];
                point.g = bgr[1];
                point.b = bgr[0];
                cloud_xyzrgb->push_back(point);
            }
        }
    }
    //top
    if(generate_top){
        z= c;//stale
        CLOG(LTRACE) <<"top " << top.cols << " x " <<top.rows << endl;
        for(x = 0; x < a; x++){
            for(y = 0; y < b; y++){
                pcl::PointXYZRGB point;
                point.x = float(a-x)/1000;
                point.y = float(-b+y)/1000;
                point.z = float(z)/1000;
                int xx = 0 + (x-0)*(top.cols-1-0)/(a-1-0);
                int yy = 0 + (y-0)*(top.rows-1-0)/(b-1-0);
                if (mask_top && top_mask.at<float>(yy, xx)==0) {
                        continue;
                }
                cv::Vec3b bgr = top.at<cv::Vec3b>(yy, xx);
                point.r = bgr[2];
                point.g = bgr[1];
                point.b = bgr[0];
                cloud_xyzrgb->push_back(point);
            }
        }
    }
    //bottom
    if(generate_bottom){
        z= 0;//stale
        CLOG(LTRACE) <<"bottom " << bottom.cols << " x " <<bottom.rows << endl;
        for(x = 0; x < a; x++){
            for(y = 0; y < b; y++){
                pcl::PointXYZRGB point;
                point.x = float(x)/1000;
                point.y = float(-b+y)/1000;
                point.z = float(z)/1000;
                int xx = 0 + (x-0)*(bottom.cols-1-0)/(a-1-0);
                int yy = 0 + (y-0)*(bottom.rows-1-0)/(b-1-0);
                if (mask_bottom && bottom_mask.at<float>(yy, xx)==0) {
                        continue;
                }
                cv::Vec3b bgr = bottom.at<cv::Vec3b>(yy, xx);
                point.r = bgr[2];
                point.g = bgr[1];
                point.b = bgr[0];
                cloud_xyzrgb->push_back(point);
            }
        }
    }
    //left
    if(generate_left){
        x= a;//stale
        CLOG(LTRACE) <<"left " << left.cols << " x " <<left.rows << endl;
        for(y = 0; y < b; y++){
            for(z = 0; z < c; z++){
                pcl::PointXYZRGB point;
                point.x = float(x)/1000;
                point.y = float(-b+y)/1000;
                point.z = float(c-z)/1000;
                int yy = 0 + (y-0)*(left.cols-1-0)/(b-1-0);
                int zz = 0 + (z-0)*(left.rows-1-0)/(c-1-0);
                if (mask_left && left_mask.at<float>(zz, yy)==0) {
                        continue;
                }
                cv::Vec3b bgr = left.at<cv::Vec3b>(zz, yy);
                point.r = bgr[2];
                point.g = bgr[1];
                point.b = bgr[0];
                cloud_xyzrgb->push_back(point);
            }
        }
    }
    //right
    if(generate_right){
        x= 0;//stale
        CLOG(LTRACE) <<"right " << right.cols << " x " <<right.rows << endl;
        for(y = 0; y < b; y++){
            for(z = 0; z < c; z++){
                pcl::PointXYZRGB point;
                point.x = float(x)/1000;
                point.y = float(-y)/1000;
                point.z = float(c-z)/1000;
                int yy = 0 + (y-0)*(right.cols-1-0)/(b-1-0);
                int zz = 0 + (z-0)*(right.rows-1-0)/(c-1-0);
                if (mask_right && right_mask.at<float>(zz, yy)==0) {
                        continue;
                }
                cv::Vec3b bgr = right.at<cv::Vec3b>(zz, yy);

                point.r = bgr[2];
                point.g = bgr[1];
                point.b = bgr[0];
                cloud_xyzrgb->push_back(point);
            }
        }
    }


    //SIFT
    int f = 0;
    cv::Mat descriptors;
    Types::Features features;
    //front
        if(generate_front){
        sift(front,descriptors,features);
        CLOG(LTRACE)<<"SIFT front " << features.features.size() <<endl;
        f+=features.features.size();
        for(int i=0; i < features.features.size(); i++){
            PointXYZSIFT point;
            int u = round(features.features[i].pt.x);
            int v = round(features.features[i].pt.y);
            if (mask_front && front_mask.at<float>(v, u)==0) {
                    continue;
            }

            int xx = 0 + (u-0)*(a-1-0)/(front.cols-1-0);
            int zz = 0 + (v-0)*(c-1-0)/(front.rows-1-0);

            point.x = float(a-xx)/1000;
            point.y = float(0)/1000;
            point.z = float(c-zz)/1000;
            for(int j=0; j<descriptors.cols;j++){
                point.descriptor[j] = descriptors.row(i).at<float>(j);
            }
            point.multiplicity = 1;
            cloud_xyzsift->push_back(point);
        }
    }
    //back
    if(generate_back){
        sift(back,descriptors,features);
        CLOG(LTRACE)<<"SIFT back " << features.features.size() <<endl;
        f+=features.features.size();
        for(int i=0; i < features.features.size(); i++){
            PointXYZSIFT point;
            int u = round(features.features[i].pt.x);
            int v = round(features.features[i].pt.y);
            if (mask_back && back_mask.at<float>(v, u)==0) {
                    continue;
            }

            int xx = 0 + (u-0)*(a-1-0)/(back.cols-1-0);
            int zz = 0 + (v-0)*(c-1-0)/(back.rows-1-0);

            point.x = float(xx)/1000;
            point.y = float(-b)/1000;
            point.z = float(c-zz)/1000;
            for(int j=0; j<descriptors.cols;j++){
                point.descriptor[j] = descriptors.row(i).at<float>(j);
            }
            point.multiplicity = 1;
            cloud_xyzsift->push_back(point);
        }
    }

    //top
    if(generate_top){
        sift(top,descriptors,features);
        CLOG(LTRACE)<<"SIFT top " << features.features.size() <<endl;
        f+=features.features.size();
        for(int i=0; i < features.features.size(); i++){
            PointXYZSIFT point;
            int u = round(features.features[i].pt.x);
            int v = round(features.features[i].pt.y);
            if (mask_top && top_mask.at<float>(v, u)==0) {
                    continue;
            }

            int xx = 0 + (u-0)*(a-1-0)/(top.cols-1-0);
            int yy = 0 + (v-0)*(b-1-0)/(top.rows-1-0);

            point.x = float(a-xx)/1000;
            point.y = float(-b+yy)/1000;
            point.z = float(c)/1000;
            for(int j=0; j<descriptors.cols;j++){
                point.descriptor[j] = descriptors.row(i).at<float>(j);
            }
            point.multiplicity = 1;
            cloud_xyzsift->push_back(point);
        }
    }
    //bottom
        if(generate_bottom){
        sift(bottom,descriptors,features);
        CLOG(LTRACE)<<"SIFT bottom " << features.features.size() <<endl;
        f+=features.features.size();
        for(int i=0; i < features.features.size(); i++){
            PointXYZSIFT point;
            int u = round(features.features[i].pt.x);
            int v = round(features.features[i].pt.y);
            if (mask_bottom && bottom_mask.at<float>(v, u)==0) {
                    continue;
            }

            int xx = 0 + (u-0)*(a-1-0)/(bottom.cols-1-0);
            int yy = 0 + (v-0)*(b-1-0)/(bottom.rows-1-0);

            point.x = float(xx)/1000;
            point.y = float(-b+yy)/1000;
            point.z = float(0)/1000;
            for(int j=0; j<descriptors.cols;j++){
                point.descriptor[j] = descriptors.row(i).at<float>(j);
            }
            point.multiplicity = 1;
            cloud_xyzsift->push_back(point);
        }
    }
    //left
    if(generate_left){
        sift(left,descriptors,features);
        CLOG(LTRACE)<<"SIFT left " << features.features.size() <<endl;
        f+=features.features.size();
        for(int i=0; i < features.features.size(); i++){
            PointXYZSIFT point;
            int u = round(features.features[i].pt.x);
            int v = round(features.features[i].pt.y);
            if (mask_left && left_mask.at<float>(v, u)==0) {
                    continue;
            }

            int yy = 0 + (u-0)*(b-1-0)/(left.cols-1-0);
            int zz = 0 + (v-0)*(c-1-0)/(left.rows-1-0);

            point.x = float(a)/1000;
            point.y = float(-b+yy)/1000;
            point.z = float(c-zz)/1000;
            for(int j=0; j<descriptors.cols;j++){
                point.descriptor[j] = descriptors.row(i).at<float>(j);
            }
            point.multiplicity = 1;
            cloud_xyzsift->push_back(point);
        }
    }
    //right
    if(generate_right){
        sift(right,descriptors,features);
        CLOG(LTRACE)<<"SIFT right " << features.features.size() <<endl;
        f+=features.features.size();
        for(int i=0; i < features.features.size(); i++){
            PointXYZSIFT point;
            int u = round(features.features[i].pt.x);
            int v = round(features.features[i].pt.y);

            int yy = 0 + (u-0)*(b-1-0)/(right.cols-1-0);
            int zz = 0 + (v-0)*(c-1-0)/(right.rows-1-0);
            if (mask_right && right_mask.at<float>(v, u)==0) {
                    continue;
            }

            point.x = float(0)/1000;
            point.y = float(-yy)/1000;
            point.z = float(c-zz)/1000;
            for(int j=0; j<descriptors.cols;j++){
                point.descriptor[j] = descriptors.row(i).at<float>(j);
            }
            point.multiplicity = 1;
            cloud_xyzsift->push_back(point);
        }
    }
    mean_viewpoint_features_number = f/6;

}



} //: namespace CuboidModelGenerator
} //: namespace Processors
