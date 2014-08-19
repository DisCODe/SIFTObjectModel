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

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace CuboidModelGenerator {

CuboidModelGenerator::CuboidModelGenerator(const std::string & name) :
		Base::Component(name) , 
        dataJSONname("dataJSONname", std::string("./")) {
    registerProperty(dataJSONname);

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
	h_generate.setup(boost::bind(&CuboidModelGenerator::generate, this));
	registerHandler("generate", &h_generate);

}

bool CuboidModelGenerator::onInit() {
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
    LOG(LTRACE) << "CuboidModelGenerator::sift()\n";
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
        LOG(LERROR) << "CuboidModelGenerator::sift() failed\n";
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
    try{
        // Open JSON file and load it to ptree.
        read_json(dataJSONname, ptree_file);
        // Read JSON properties.
        model_name = ptree_file.get<std::string>("name");
        left_name = ptree_file.get<std::string>("left");
        right_name = ptree_file.get<std::string>("right");
        top_name = ptree_file.get<std::string>("top");
        bottom_name = ptree_file.get<std::string>("bottom");
        front_name = ptree_file.get<std::string>("front");
        back_name = ptree_file.get<std::string>("back");
        a = ptree_file.get<int>("a");
        b = ptree_file.get<int>("b");
        c = ptree_file.get<int>("c");
    }//: try
    catch(std::exception const& e){
        LOG(LERROR) << "SOMJSONReader: file "<< dataJSONname <<" not found or invalid\n";
        return;
    }//: catch

    left = cv::imread(left_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    right = cv::imread(right_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    top = cv::imread(top_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    bottom = cv::imread(bottom_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    front = cv::imread(front_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    back = cv::imread(back_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

}

void CuboidModelGenerator::generate() {
    CLOG(LTRACE) << "CuboidModelGenerator::generate";
    loadData();

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    int x,y,z;
    //front
    y=0;//stałe
    for(x = 0; x < a; x++){
        for(z = 0; z < c; z++){
            pcl::PointXYZRGB point;
            point.x = float(a-x)/1000;
            point.y = float(y)/1000;
            point.z = float(c-z)/1000;
            //pozycja w obrazie
            int xx = 0 + (x-0)*(front.cols-1-0)/(a-1-0);
            int zz = 0 + (z-0)*(front.rows-1-0)/(c-1-0);
            cv::Vec3b bgr = front.at<cv::Vec3b>(zz, xx);
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            cloud_xyzrgb->push_back(point);
        }

    }
    //back
    y=-b;//stale
    cout <<"back " << back.cols << " x " <<back.rows << endl;
    for(x = 0; x < a; x++){
        for(z = 0; z < c; z++){
            pcl::PointXYZRGB point;
            point.x = float(x)/1000;
            point.y = float(y)/1000;
            point.z = float(c-z)/1000;
            int xx = 0 + (x-0)*(back.cols-1-0)/(a-1-0);
            int zz = 0 + (z-0)*(back.rows-1-0)/(c-1-0);
            cv::Vec3b bgr = back.at<cv::Vec3b>(zz, xx);
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            cloud_xyzrgb->push_back(point);
        }
    }

    //top
    z= c;//stale
    cout <<"top " << top.cols << " x " <<top.rows << endl;
    for(x = 0; x < a; x++){
        for(y = 0; y < b; y++){
            pcl::PointXYZRGB point;
            point.x = float(a-x)/1000;
            point.y = float(-b+y)/1000;
            point.z = float(z)/1000;
            int xx = 0 + (x-0)*(top.cols-1-0)/(a-1-0);
            int yy = 0 + (y-0)*(top.rows-1-0)/(b-1-0);
            cv::Vec3b bgr = top.at<cv::Vec3b>(yy, xx);
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            cloud_xyzrgb->push_back(point);
        }
    }

    //bottom
    z= 0;//stale
    cout <<"bottom " << bottom.cols << " x " <<bottom.rows << endl;
    for(x = 0; x < a; x++){
        for(y = 0; y < b; y++){
            pcl::PointXYZRGB point;
            point.x = float(x)/1000;
            point.y = float(-b+y)/1000;
            point.z = float(z)/1000;
            int xx = 0 + (x-0)*(bottom.cols-1-0)/(a-1-0);
            int yy = 0 + (y-0)*(bottom.rows-1-0)/(b-1-0);
            cv::Vec3b bgr = bottom.at<cv::Vec3b>(yy, xx);
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            cloud_xyzrgb->push_back(point);
        }
    }

    //left
    x= a;//stale
    cout <<"left " << left.cols << " x " <<left.rows << endl;
    for(y = 0; y < b; y++){
        for(z = 0; z < c; z++){
            pcl::PointXYZRGB point;
            point.x = float(x)/1000;
            point.y = float(-b+y)/1000;
            point.z = float(c-z)/1000;
            int yy = 0 + (y-0)*(left.cols-1-0)/(b-1-0);
            int zz = 0 + (z-0)*(left.rows-1-0)/(c-1-0);
            cv::Vec3b bgr = left.at<cv::Vec3b>(zz, yy);
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            cloud_xyzrgb->push_back(point);
        }
    }

    //right
    x= 0;//stale
    cout <<"left " << right.cols << " x " <<right.rows << endl;
    for(y = 0; y < b; y++){
        for(z = 0; z < c; z++){
            pcl::PointXYZRGB point;
            point.x = float(x)/1000;
            point.y = float(-y)/1000;
            point.z = float(c-z)/1000;
            int yy = 0 + (y-0)*(right.cols-1-0)/(b-1-0);
            int zz = 0 + (z-0)*(right.rows-1-0)/(c-1-0);
            cv::Vec3b bgr = right.at<cv::Vec3b>(zz, yy);
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            cloud_xyzrgb->push_back(point);
        }
    }

    out_cloud_xyzrgb.write(cloud_xyzrgb);


    //SIFT
    //pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>());
    int f = 0;
    cv::Mat descriptors;
    Types::Features features;
    //front
    sift(front,descriptors,features);
    CLOG(LTRACE)<<"SIFT front " << features.features.size() <<endl;
    f+=features.features.size();
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

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
    //back
    sift(back,descriptors,features);
    CLOG(LTRACE)<<"SIFT back " << features.features.size() <<endl;
    f+=features.features.size();
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

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
    //top
    sift(top,descriptors,features);
    CLOG(LTRACE)<<"SIFT top " << features.features.size() <<endl;
    f+=features.features.size();
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

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

    //bottom
    sift(bottom,descriptors,features);
    CLOG(LTRACE)<<"SIFT bottom " << features.features.size() <<endl;
    f+=features.features.size();
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

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

    //left
    sift(left,descriptors,features);
    CLOG(LTRACE)<<"SIFT left " << features.features.size() <<endl;
    f+=features.features.size();
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

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

    //right
    sift(right,descriptors,features);
    CLOG(LTRACE)<<"SIFT right " << features.features.size() <<endl;
    f+=features.features.size();
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

        int yy = 0 + (u-0)*(b-1-0)/(right.cols-1-0);
        int zz = 0 + (v-0)*(c-1-0)/(right.rows-1-0);

        point.x = float(0)/1000;
        point.y = float(-yy)/1000;
        point.z = float(c-zz)/1000;
        for(int j=0; j<descriptors.cols;j++){
            point.descriptor[j] = descriptors.row(i).at<float>(j);
        }
        point.multiplicity = 1;
        cloud_xyzsift->push_back(point);
    }

    out_cloud_xyzsift.write(cloud_xyzsift);

    mean_viewpoint_features_number = f/6;
    out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
    SIFTObjectModel* model;
    model = dynamic_cast<SIFTObjectModel*>(produce());
    out_model.write(model);
}



} //: namespace CuboidModelGenerator
} //: namespace Processors
