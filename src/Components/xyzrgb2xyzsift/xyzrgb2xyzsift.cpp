/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "xyzrgb2xyzsift.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#if CV_MAJOR_VERSION == 2
#if CV_MINOR_VERSION > 3
#include <opencv2/nonfree/features2d.hpp>
#endif
#elif CV_MAJOR_VERSION == 3
#include <opencv2/nonfree/features2d.hpp>
#endif

namespace Processors {
namespace xyzrgb2xyzsift {

xyzrgb2xyzsift::xyzrgb2xyzsift(const std::string & name) :
		Base::Component(name)  {

}

xyzrgb2xyzsift::~xyzrgb2xyzsift() {
}

void xyzrgb2xyzsift::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	// Register handlers
	h_compute.setup(boost::bind(&xyzrgb2xyzsift::compute, this));
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud_xyzrgb);

}

bool xyzrgb2xyzsift::onInit() {

	return true;
}

bool xyzrgb2xyzsift::onFinish() {
	return true;
}

bool xyzrgb2xyzsift::onStop() {
	return true;
}

bool xyzrgb2xyzsift::onStart() {
	return true;
}

void xyzrgb2xyzsift::compute() {
    CLOG(LTRACE) << "xyzrgb2xyzsift::compute";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

    cloud->is_dense = true ;

    cv::Mat rgbimg(cloud->height, cloud->width, CV_8UC3, cv::Scalar::all(0)) ;
    for (register int row = 0; row  < cloud->height; row++)
        for (register int col = 0; col < cloud->width ; col++) {
            pcl::PointXYZRGB& pt = cloud->at(col, row) ;
            cv::Vec3b &rgbvec = rgbimg.at<cv::Vec3b>(row, col) ;
            rgbvec[2] = pt.r ; //Creating image for all pixels: normal and NaN
            rgbvec[1] = pt.g ;
            rgbvec[0] = pt.b ;
    }


    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    try {
        //-- Step 1: Detect the keypoints.
        cv::SiftFeatureDetector detector;

        detector.detect(rgbimg, keypoints);

        //-- Step 2: Calculate descriptors (feature vectors).
        cv::SiftDescriptorExtractor extractor;

        extractor.compute( rgbimg, keypoints, descriptors);
    } catch (...) {
        LOG(LERROR) << "sdasdas\n";
    }

    pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift(new pcl::PointCloud<PointXYZSIFT>);

    //Create output point cloud (this time - unorganized)
    cloud_xyzsift->height = 1 ;
    cloud_xyzsift->width = keypoints.size() ;
    cloud_xyzsift->is_dense = false ;
    cloud_xyzsift->points.resize(cloud_xyzsift->height * cloud_xyzsift->width) ;



    //Convert SIFT keypoints/descriptors into PointXYZRGBSIFT cloud
    pcl::PointCloud<PointXYZSIFT>::iterator pt_iter = cloud_xyzsift->begin();
    for (register int i = 0; i < keypoints.size() ; i++) {

        //std::cout << " " << keypoints[i].pt.x << " " << keypoints[i].pt.y << std::endl ;
        int keyx = std::min(std::max(((unsigned int) round(keypoints[i].pt.x)), 0u), cloud->width) ;
        int keyy = std::min(std::max(((unsigned int) round(keypoints[i].pt.y)), 0u), cloud->height) ;
        //std::cout << keyx << " " << keyy << std::endl ;
        cv::Mat desc = descriptors(cv::Range(i,i + 1), cv::Range::all()) ;
        //std::cout << "descriptor " << desc << std::endl ;

        //Make XYZSIFT point
        PointXYZSIFT& pt_out = *pt_iter++ ;
        pcl::PointXYZRGB& pt_in = cloud->at(keyx, keyy) ;
        pt_out.x = pt_in.x; pt_out.y = pt_in.y; pt_out.z = pt_in.z ;
        for(int j=0; j<descriptors.cols;j++){
            pt_out.descriptor[j] = descriptors.row(i).at<float>(j);
        }
    }

    out_cloud_xyzsift.write(cloud_xyzsift);
}



} //: namespace xyzrgb2xyzsift
} //: namespace Processors
