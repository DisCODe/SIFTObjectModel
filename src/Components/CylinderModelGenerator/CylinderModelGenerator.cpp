/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "CylinderModelGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#if (CV_MAJOR_VERSION == 2)
#if (CV_MINOR_VERSION > 3)
#include <opencv2/nonfree/features2d.hpp>
#endif
#endif

#include <math.h>

using boost::property_tree::ptree;
using boost::property_tree::read_json;

namespace Processors {
namespace CylinderModelGenerator {

CylinderModelGenerator::CylinderModelGenerator(const std::string & name) :
		Base::Component(name) , 
        dataJSONname("dataJSONname", std::string("./")),
        step("step", 1.0){
	registerProperty(dataJSONname);
    registerProperty(step);

}

CylinderModelGenerator::~CylinderModelGenerator() {
}

void CylinderModelGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	// Register handlers
    h_generateModel.setup(boost::bind(&CylinderModelGenerator::generateModelPressed, this));
	registerHandler("generateModel", &h_generateModel);

	h_returnModel.setup(boost::bind(&CylinderModelGenerator::returnModel, this));
	registerHandler("returnModel", &h_returnModel);
	addDependency("returnModel", NULL);

}

bool CylinderModelGenerator::onInit() {
    CLOG(LTRACE) << "CylinderModelGenerator::onInit";
    cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
	return true;
}

bool CylinderModelGenerator::onFinish() {
	return true;
}

bool CylinderModelGenerator::onStop() {
	return true;
}

bool CylinderModelGenerator::onStart() {
	return true;
}

void CylinderModelGenerator::sift(cv::Mat input, cv::Mat &descriptors, Types::Features &features) {
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

void CylinderModelGenerator::loadData(){
    CLOG(LTRACE) << "CylinderModelGenerator::loadData";
    ptree ptree_file;
    std::string model_name;
    std::string top_name;
    std::string bottom_name;
    std::string side_name;
    try{
        cout<<dataJSONname<<endl;
        // Open JSON file and load it to ptree.
        read_json(dataJSONname, ptree_file);
        // Read JSON properties.
        model_name = ptree_file.get<std::string>("name");
        side_name = ptree_file.get<std::string>("side");
        top_name = ptree_file.get<std::string>("top");
        bottom_name = ptree_file.get<std::string>("bottom");

        h = ptree_file.get<int>("h");
        r = ptree_file.get<int>("r");

    }//: try
    catch(std::exception const& e){
        LOG(LERROR) << "CylinderModelGenerator: file "<< dataJSONname <<" not found or invalid\n";
        return;
    }//: catch

    top = cv::imread(top_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    bottom = cv::imread(bottom_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
    side = cv::imread(side_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

}

void CylinderModelGenerator::generateModelPressed() {
    CLOG(LTRACE) << "CylinderModelGenerator::generateModelPressed";
    generateModel_flag = true;
}

void CylinderModelGenerator::generateModel() {
    CLOG(LTRACE) << "CylinderModelGenerator::generateModel";
    loadData();

    // Clear clouds.
    cloud_xyzrgb->clear();
    cloud_xyzsift->clear();
    //width
    float w = 2 * M_PI * r;


    //side
    for (float i=0; i<h; i+=step){
        for(float j=0; j<w; j+=step){
            // Get image coordinates.
            pcl::PointXYZRGB point;
            int v = 0 + (j-0)*(side.cols-1-0)/(w-1-0);
            int u = 0 + (i-0)*(side.rows-1-0)/(h-1-0);
            cv::Vec3b bgr = side.at<cv::Vec3b>(u, v);
            // Set point colour.
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            // Compute cylindric parameters.
            float z = i;
            float fi = -j*360/(w-1);
            // Set cartesian coordinates of the cylinder side.
            point.x = float(r) * cos(fi * M_PI /180.0) /1000;
            point.y = float(r) * sin(fi * M_PI /180.0) /1000;
            point.z = float(z)/1000;
            // Add point to cloud.
            cloud_xyzrgb->push_back(point);
        }
    }

    //top
    for (float i=0; i<2*r; i+=step){
        for(float j=0; j<2*r; j+=step){
            if(sqrt(abs(i-r)*abs(i-r)+abs(j-r)*abs(j-r)) > r)
                continue;
            // Get image coordinates.
            pcl::PointXYZRGB point;
            int v = 0 + (j-0)*(top.cols-1-0)/(2*r-1-0);
            int u = 0 + (i-0)*(top.rows-1-0)/(2*r-1-0);
            cv::Vec3b bgr = top.at<cv::Vec3b>(u, v);
            // Set point colour.
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            // Set cartesian coordinates of the cylinder top.
            point.x = (i-float(r))/1000;
            point.y = (j-float(r))/1000;
            point.z = float(h)/1000;
            // Add point to cloud.
            cloud_xyzrgb->push_back(point);
        }
    }

    //bottom
    for (float i=0; i<2*r; i+=step){
        for(float j=0; j<2*r; j+=step){
            if(sqrt(abs(i-r)*abs(i-r)+abs(j-r)*abs(j-r)) > r)
                continue;
            // Get image coordinates.
            pcl::PointXYZRGB point;
            int v = 0 + (j-0)*(bottom.cols-1-0)/(2*r-1-0);
            int u = 0 + (i-0)*(bottom.rows-1-0)/(2*r-1-0);
            cv::Vec3b bgr = bottom.at<cv::Vec3b>(u, v);
            // Set point colour.
            point.r = bgr[2];
            point.g = bgr[1];
            point.b = bgr[0];
            // Set cartesian coordinates of the cylinder top.
            point.x = (float(r)-i)/1000;
            point.y = (j-float(r))/1000;
            point.z = 0/1000;
            // Add point to cloud.
            cloud_xyzrgb->push_back(point);
        }
    }

    //SIFT
    int f = 0;
    cv::Mat descriptors;
    Types::Features features;
    //side
    sift(side,descriptors,features);
    CLOG(LTRACE)<<"SIFT side " << features.features.size() <<endl;

    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        // Get image coordinates.
        float u = features.features[i].pt.x;
        float v = features.features[i].pt.y;
        // Compute cylindric parameters.
        float z = v*h/side.rows;
        float fi = -(u*w/side.cols)*360/(w-1);
        // Set cartesian coordinates of the cylinder side.
        point.x = float(r) * cos(fi * M_PI /180.0) /1000;
        point.y = float(r) * sin(fi * M_PI /180.0) /1000;
        point.z = float(z)/1000;
        //Set SIFT descriptor
        for(int j=0; j<descriptors.cols;j++){
            point.descriptor[j] = descriptors.row(i).at<float>(j);
        }
        point.multiplicity = 1;
        cloud_xyzsift->push_back(point);
    }

    //top
    sift(top,descriptors,features);
    CLOG(LTRACE)<<"SIFT top " << features.features.size() <<endl;
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        // Get image coordinates.
        float u = features.features[i].pt.x;
        float v = features.features[i].pt.y;
        if(sqrt(abs((u*2*r/top.cols)-r)*abs((u*2*r/top.cols)-r)+abs((v*2*r/top.rows)-r)*abs((v*2*r/top.rows)-r)) > r)
            continue;
        // Set cartesian coordinates of the cylinder side.
        point.y = ((u*2*r/top.cols)-float(r)) /1000;
        point.x = ((v*2*r/top.rows)-float(r)) /1000;
        point.z = float(h)/1000;
        //Set SIFT descriptor
        for(int j=0; j<descriptors.cols;j++){
            point.descriptor[j] = descriptors.row(i).at<float>(j);
        }
        point.multiplicity = 1;
        cloud_xyzsift->push_back(point);
    }

    //bottom
    sift(bottom,descriptors,features);
    CLOG(LTRACE)<<"SIFT bottom " << features.features.size() <<endl;
    for(int i=0; i < features.features.size(); i++){
        PointXYZSIFT point;
        // Get image coordinates.
        float u = features.features[i].pt.x;
        float v = features.features[i].pt.y;
        if(sqrt(abs((u*2*r/bottom.cols)-r)*abs((u*2*r/bottom.cols)-r)+abs((v*2*r/bottom.rows)-r)*abs((v*2*r/bottom.rows)-r)) > r)
            continue;
        // Set cartesian coordinates of the cylinder side.
        point.y = ((u*2*r/bottom.cols)-float(r)) /1000;
        point.x = (float(r)-(v*2*r/bottom.rows)) /1000;
        point.z = 0/1000;
        //Set SIFT descriptor
        for(int j=0; j<descriptors.cols;j++){
            point.descriptor[j] = descriptors.row(i).at<float>(j);
        }
        point.multiplicity = 1;
        cloud_xyzsift->push_back(point);
    }
}

void CylinderModelGenerator::returnModel() {
    CLOG(LTRACE) << "CylinderModelGenerator::returnModel";
    // Generate model if required.
    if (generateModel_flag) {
        generateModel();
        generateModel_flag = false;
    }

    // Write to output clouds.
    out_cloud_xyzrgb.write(cloud_xyzrgb);
    out_cloud_xyzsift.write(cloud_xyzsift);
}



} //: namespace CylinderModelGenerator
} //: namespace Processors
