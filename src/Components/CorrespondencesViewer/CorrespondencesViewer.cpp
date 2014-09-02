/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>
#include <sstream>
#include "CorrespondencesViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
//#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/filter.h>
namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name),
		prop_window_name("window_name", std::string("Correspondences Viewer")),
		cloud_xyzsift1_point_size("cloud_xyzsift1_point_size", 5), 
		cloud_xyzsift2_point_size("cloud_xyzsift2_point_size", 5),
		clouds_colours("clouds_colours", cv::Mat(cv::Mat::zeros(2, 3, CV_8UC1))),
		correspondences_colours("correspondences_colours", cv::Mat(cv::Mat::zeros(1, 3, CV_8UC1))),
		display_cloud_xyzrgb1("display_cloud_xyzrgb1", true),
		display_cloud_xyzrgb2("display_cloud_xyzrgb2", true),
		display_cloud_xyzsift1("display_cloud_xyzsift1", true),
		display_cloud_xyzsift2("display_cloud_xyzsift2", true),
		display_correspondences("display_correspondences", true),
		display_good_correspondences("display_good_correspondences", true),
		prop_coordinate_system("coordinate_system", false),
		tx("tx", 0.3f),
		ty("ty", 0.0f),
		tz("tz", 0.0f){
			registerProperty(prop_window_name);
			registerProperty(cloud_xyzsift1_point_size);
			registerProperty(cloud_xyzsift2_point_size);
			registerProperty(clouds_colours);
			registerProperty(correspondences_colours);
			registerProperty(display_cloud_xyzrgb1);
			registerProperty(display_cloud_xyzrgb2);
			registerProperty(display_cloud_xyzsift1);
			registerProperty(display_cloud_xyzsift2);
			registerProperty(display_correspondences);
			registerProperty(display_good_correspondences);
			registerProperty(prop_coordinate_system);
			registerProperty(tx);
			registerProperty(ty);
			registerProperty(tz);
			
			  // Set red as default.
			((cv::Mat)clouds_colours).at<uchar>(0,0) = 255;
			((cv::Mat)clouds_colours).at<uchar>(0,1) = 0;
			((cv::Mat)clouds_colours).at<uchar>(0,2) = 0;
			((cv::Mat)clouds_colours).at<uchar>(1,0) = 255;
			((cv::Mat)clouds_colours).at<uchar>(1,1) = 0;
			((cv::Mat)clouds_colours).at<uchar>(1,2) = 0;
			((cv::Mat)correspondences_colours).at<uchar>(0,0) = 255;
			((cv::Mat)correspondences_colours).at<uchar>(0,1) = 0;
			((cv::Mat)correspondences_colours).at<uchar>(0,2) = 0;
}

CorrespondencesViewer::~CorrespondencesViewer() {
}

void CorrespondencesViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_cloud_xyzsift1", &in_cloud_xyzsift1);
registerStream("in_cloud_xyzsift2", &in_cloud_xyzsift2);
registerStream("in_cloud_xyzrgb1", &in_cloud_xyzrgb1);
registerStream("in_cloud_xyzrgb2", &in_cloud_xyzrgb2);
registerStream("in_correspondences", &in_correspondences);
registerStream("in_good_correspondences", &in_good_correspondences);
registerStream("in_clustered_correspondences", &in_clustered_correspondences);

	// Register handlers
	h_on_clouds.setup(boost::bind(&CorrespondencesViewer::on_clouds, this));
	registerHandler("on_clouds", &h_on_clouds);
	addDependency("on_clouds", &in_cloud_xyzsift1);
	addDependency("on_clouds", &in_cloud_xyzsift2);
	addDependency("on_clouds", &in_cloud_xyzrgb1);
    //addDependency("on_clouds", &in_correspondences);
	addDependency("on_clouds", &in_cloud_xyzrgb2);	
	// Register spin handler.
	h_on_spin.setup(boost::bind(&CorrespondencesViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CorrespondencesViewer::onInit() {
	LOG(LTRACE) << "CorrespondencesViewer::onInit";
	viewer = new pcl::visualization::PCLVisualizer (prop_window_name);
	viewer->setBackgroundColor (0, 0, 0);

	//viewer->addCoordinateSystem (1.0);

	viewer->initCameraParameters ();
	//cloud_view_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    clusters = 0;
	return true;
}

bool CorrespondencesViewer::onFinish() {
	return true;
}

bool CorrespondencesViewer::onStop() {
	return true;
}

bool CorrespondencesViewer::onStart() {
	return true;
}

void CorrespondencesViewer::on_clouds() {
	LOG(LTRACE) << "CorrespondencesViewer::on_clouds()";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb1 = in_cloud_xyzrgb1.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb2 = in_cloud_xyzrgb2.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift1 = in_cloud_xyzsift1.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift2 = in_cloud_xyzsift2.read();
    //pcl::CorrespondencesPtr correspondences = in_correspondences.read();
	
	//*cloud_view_xyzsift = *cloud_xyzsift1;
	
	//Define a small translation between clouds	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
	//float tx = 0.3f, ty = 0.0f, tz = 0.0f ;
	trans(0, 3) = tx ; trans(1, 3) = ty ; trans(2, 3) = tz ;

	//Transform one of the clouds	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb2trans(new pcl::PointCloud<pcl::PointXYZRGB>) ;
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift2trans(new pcl::PointCloud<PointXYZSIFT>) ;
	pcl::transformPointCloud(*cloud_xyzrgb2, *cloud_xyzrgb2trans, trans) ;
	pcl::transformPointCloud(*cloud_xyzsift2, *cloud_xyzsift2trans, trans) ;
	
	//Display clouds
	if(prop_coordinate_system)
		viewer->addCoordinateSystem (1.0);
	
	viewer->removePointCloud("viewcloud1") ;
	if(display_cloud_xyzrgb1){
		std::vector<int> indices;
		cloud_xyzrgb1->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud_xyzrgb1, *cloud_xyzrgb1, indices);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution1(cloud_xyzrgb1);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_xyzrgb1, color_distribution1, "viewcloud1") ;
	}
	viewer->removePointCloud("siftcloud1") ;
	if(display_cloud_xyzsift1){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz1(new pcl::PointCloud<pcl::PointXYZ>); 
		pcl::copyPointCloud(*cloud_xyzsift1,*cloud_xyz1);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz1, "siftcloud1") ;
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud_xyzsift1_point_size, "siftcloud1");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
			((cv::Mat)clouds_colours).at<uchar>(0, 0),
			((cv::Mat)clouds_colours).at<uchar>(0, 1),
			((cv::Mat)clouds_colours).at<uchar>(0, 2), 
			"siftcloud1");
	}
	
	viewer->removePointCloud("viewcloud2") ;
	if(display_cloud_xyzrgb2){
		std::vector<int> indices;
		cloud_xyzrgb2->is_dense = false; 
		pcl::removeNaNFromPointCloud(*cloud_xyzrgb2, *cloud_xyzrgb2, indices);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_distribution2(cloud_xyzrgb2trans);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud_xyzrgb2trans, color_distribution2, "viewcloud2") ;
	}
	viewer->removePointCloud("siftcloud2") ;
	if(display_cloud_xyzsift2){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz2(new pcl::PointCloud<pcl::PointXYZ>); 
		pcl::copyPointCloud(*cloud_xyzsift2trans,*cloud_xyz2);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_xyz2, "siftcloud2") ;
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud_xyzsift2_point_size, "siftcloud2");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
			((cv::Mat)clouds_colours).at<uchar>(1, 0),
			((cv::Mat)clouds_colours).at<uchar>(1, 1),
			((cv::Mat)clouds_colours).at<uchar>(1, 2), 
			"siftcloud2");
	}
	//Display correspondences
	viewer->removeCorrespondences("correspondences") ;
    if(display_correspondences && !in_correspondences.empty()){
        pcl::CorrespondencesPtr correspondences = in_correspondences.read();
		viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift1, cloud_xyzsift2trans, *correspondences, "correspondences") ;
		viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 
			((cv::Mat)correspondences_colours).at<uchar>(0, 0),
			((cv::Mat)correspondences_colours).at<uchar>(0, 1),
			((cv::Mat)correspondences_colours).at<uchar>(0, 2), 
			"correspondences") ;
	}
	
	//Display good correspondences
	viewer->removeCorrespondences("good_correspondences") ;
	if (display_good_correspondences && !in_good_correspondences.empty()){
		pcl::CorrespondencesPtr good_correspondences = in_good_correspondences.read();
		if(display_correspondences){
			viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift1, cloud_xyzsift2trans, *good_correspondences, "good_correspondences") ;
			viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "good_correspondences") ;
		}
	}

    //Display clustered correspondences
    for(int i = 0; i <clusters; i++){
        ostringstream ss;
        ss << i;
        string str = ss.str();
        viewer->removeCorrespondences(std::string("correspondences")+str) ;
    }
    if(display_correspondences && !in_clustered_correspondences.empty()){
        std::vector<pcl::Correspondences> clustered_corrs = in_clustered_correspondences.read();
        clusters = clustered_corrs.size();
        for(int i = 0; i< clustered_corrs.size(); i++){
            int r,g,b;
            r = g = b = 0;
            int ii = 60 + i*(255-60)/clustered_corrs.size();
            int lb = ii & 0xff;
            if (ii > 50)
                switch (ii>>6) {
                case 0:
                    b = 255;
                    g = 255-lb;
                    r = 255-lb;
                    break;
                case 1:
                    b = 255;
                    g = lb;
                    r = 0;
                    break;
                case 2:
                    b = 255-lb;
                    g = 255;
                    r = 0;
                    break;
                case 3:
                    b = 0;
                    g = 255;
                    r = lb;
                    break;
                case 4:
                    b = 0;
                    g = 255-lb;
                    r = 255;
                    break;
                case 5:
                    b = 0;
                    g = 0;
                    r = 255-lb;
                    break;
                default:
                    r = g = b = 0;
                    break;
                }

            cout << "cloud_xyzsift1 " <<cloud_xyzsift1->size() << " cloud_xyzsift2trans " << cloud_xyzsift2trans->size()<< " cluster " << clustered_corrs[i].size() <<endl;
            ostringstream ss;
            ss << i;
            string str = ss.str();
//            viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift1, cloud_xyzsift2trans, clustered_corrs[i], "correspondences"+str) ;
            viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift2trans, cloud_xyzsift1, clustered_corrs[i], "correspondences"+str) ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                r,
                g,
                b,
                "correspondences"+str) ;
            cout<<"ii "<< ii <<" "<< (ii>>6) <<" rgb " <<r<<" "<<g<<" "<<b<<endl;
        }

    }
}


void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}


} //: namespace CorrespondencesViewer
} //: namespace Processors
