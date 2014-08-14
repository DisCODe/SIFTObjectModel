/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "FeatureCloudConverter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FeatureCloudConverter {

FeatureCloudConverter::FeatureCloudConverter(const std::string & name) :
		Base::Component(name)  {

}

FeatureCloudConverter::~FeatureCloudConverter() {
}

void FeatureCloudConverter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_depth", &in_depth);
	registerStream("in_mask", &in_mask);
	registerStream("in_features", &in_features);
	registerStream("in_descriptors", &in_descriptors);
	registerStream("in_camera_info", &in_camera_info);
    registerStream("in_depth_xyz", &in_depth_xyz);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);

	// Register handlers
	h_process.setup(boost::bind(&FeatureCloudConverter::process, this));
	registerHandler("process", &h_process);
	addDependency("process", &in_depth);
	addDependency("process", &in_features);
	addDependency("process", &in_descriptors);
	addDependency("process", &in_camera_info);
	h_process_mask.setup(boost::bind(&FeatureCloudConverter::process_mask, this));
	registerHandler("process_mask", &h_process_mask);
	addDependency("process_mask", &in_depth);
	addDependency("process_mask", &in_mask);
	addDependency("process_mask", &in_features);
	addDependency("process_mask", &in_descriptors);
	addDependency("process_mask", &in_camera_info);
    h_process_depth_xyz.setup(boost::bind(&FeatureCloudConverter::process_depth_xyz, this));
    registerHandler("process_depth_xyz", &h_process_depth_xyz);
    addDependency("process_depth_xyz", &in_features);
    addDependency("process_depth_xyz", &in_descriptors);
    addDependency("proces_depth_xyzs", &in_depth_xyz);
    h_process_depth_xyz_mask.setup(boost::bind(&FeatureCloudConverter::process_depth_xyz_mask, this));
    registerHandler("process_depth_xyz_mask", &h_process_depth_xyz_mask);
    addDependency("process_depth_xyz_mask", &in_mask);
    addDependency("process_depth_xyz_mask", &in_features);
    addDependency("process_depth_xyz_mask", &in_descriptors);
    addDependency("process_depth_xyz_mask", &in_depth_xyz);

}

bool FeatureCloudConverter::onInit() {

	return true;
}

bool FeatureCloudConverter::onFinish() {
	return true;
}

bool FeatureCloudConverter::onStop() {
	return true;
}

bool FeatureCloudConverter::onStart() {
	return true;
}

void FeatureCloudConverter::process() {
	CLOG(LTRACE) << "FeatureCloudConverter::process";
	cv::Mat depth = in_depth.read();
	depth.convertTo(depth, CV_32F);
	cv::Mat descriptors = in_descriptors.read();
	Types::Features features = in_features.read();
	Types::CameraInfo camera_info = in_camera_info.read();

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud (new pcl::PointCloud<PointXYZSIFT>());
	
	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();
	
	for(int i=0; i < features.features.size(); i++){
		
		PointXYZSIFT point;
		int u = round(features.features[i].pt.x);
		int v = round(features.features[i].pt.y);
		//cout<<features.features[i].pt.x<<" -> "<<u<<endl;
		//cout<<features.features[i].pt.y<<" -> "<<v<<endl;
	
		//const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[v]);
		//uint16_t d = depth_row[u];
		float d = depth.at<float>(v, u);
		if (d == 0) {
				continue;
		}
		
		point.x = (u - cx_d) * d * fx_d;
		point.y = (v - cy_d) * d * fy_d;
		point.z = d * 0.001;

		
		for(int j=0; j<descriptors.cols;j++){
			point.descriptor[j] = descriptors.row(i).at<float>(j);	
		}
		
		point.multiplicity = 1;
		
		cloud->push_back(point);
	}
	
	out_cloud_xyzsift.write(cloud);
}


void FeatureCloudConverter::process_mask() {
	CLOG(LTRACE) << "FeatureCloudConverter::process_mask";
	cv::Mat depth = in_depth.read();
	depth.convertTo(depth, CV_32F);
	cv::Mat mask = in_mask.read();
	mask.convertTo(mask, CV_32F);
	cv::Mat descriptors = in_descriptors.read();
	Types::Features features = in_features.read();
	Types::CameraInfo camera_info = in_camera_info.read();

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud (new pcl::PointCloud<PointXYZSIFT>());
	
	double fx_d = 0.001 / camera_info.fx();
	double fy_d = 0.001 / camera_info.fy();
	double cx_d = camera_info.cx();
	double cy_d = camera_info.cy();
	
	for(int i=0; i < features.features.size(); i++){
		
		PointXYZSIFT point;
		int u = round(features.features[i].pt.x);
		int v = round(features.features[i].pt.y);
		//cout<<features.features[i].pt.x<<" -> "<<u<<endl;
		//cout<<features.features[i].pt.y<<" -> "<<v<<endl;
	
		//const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth.data[v]);
		//uint16_t d = depth_row[u];
		float d = depth.at<float>(v, u);
		if (d == 0 || mask.at<float>(v, u)==0) {
				continue;
		}
		
		point.x = (u - cx_d) * d * fx_d;
		point.y = (v - cy_d) * d * fy_d;
		point.z = d * 0.001;

		for(int j=0; j<descriptors.cols;j++){
			point.descriptor[j] = descriptors.row(i).at<float>(j);	
		}
		
		point.multiplicity = 1;
		
		cloud->push_back(point);
	}
	
	out_cloud_xyzsift.write(cloud);
}

void FeatureCloudConverter::process_depth_xyz() {
    CLOG(LTRACE) << "FeatureCloudConverter::process";
    cv::Mat depth_xyz = in_depth_xyz.read();
    cv::Mat descriptors = in_descriptors.read();
    Types::Features features = in_features.read();

    pcl::PointCloud<PointXYZSIFT>::Ptr cloud (new pcl::PointCloud<PointXYZSIFT>());


    for(int i=0; i < features.features.size(); i++){

        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);

        cv::Vec3f p = depth_xyz.at<cv::Vec3f>(v, u);


        point.x = p[0];
        point.y = p[1];
        point.z = p[2];


        for(int j=0; j<descriptors.cols;j++){
            point.descriptor[j] = descriptors.row(i).at<float>(j);
        }

        point.multiplicity = 1;

        cloud->push_back(point);
    }

    out_cloud_xyzsift.write(cloud);
}

void FeatureCloudConverter::process_depth_xyz_mask() {
    CLOG(LTRACE) << "FeatureCloudConverter::process";
    cv::Mat depth_xyz = in_depth_xyz.read();
    cv::Mat descriptors = in_descriptors.read();
    Types::Features features = in_features.read();
    cv::Mat mask = in_mask.read();
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud (new pcl::PointCloud<PointXYZSIFT>());


    for(int i=0; i < features.features.size(); i++){

        PointXYZSIFT point;
        int u = round(features.features[i].pt.x);
        int v = round(features.features[i].pt.y);
        if (mask.at<float>(v, u)==0) {
                continue;
        }
        cv::Vec3f p = depth_xyz.at<cv::Vec3f>(v, u);


        point.x = p[0];
        point.y = p[1];
        point.z = p[2];


        for(int j=0; j<descriptors.cols;j++){
            point.descriptor[j] = descriptors.row(i).at<float>(j);
        }

        point.multiplicity = 1;

        cloud->push_back(point);
    }

    out_cloud_xyzsift.write(cloud);
}

} //: namespace FeatureCloudConverter
} //: namespace Processors
