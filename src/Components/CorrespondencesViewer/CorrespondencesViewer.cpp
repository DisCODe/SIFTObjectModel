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
#include <pcl/common/common.h>

namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name),
		prop_window_name("window_name", std::string("Correspondences Viewer")),
		prop_coordinate_system("coordinate_system",boost::bind(&CorrespondencesViewer::onCSShowClick, this, _2), true),
		prop_background_color("background_color", boost::bind(&CorrespondencesViewer::onBackgroundColorChange, this, _2), std::string("0,0,0")),
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
        display_bounding_box("display_bounding_box", false),
		tx("tx", 0.3f),
		ty("ty", 0.0f),
        tz("tz", 0.0f),
        display_one_cluster("display_one_cluster", false),
        display_cluster("display_cluster", 0),
        display_clusters("display_clusters", false),
        display_clusters_bounding_boxes("display_clusters_bounding_boxes", false)
{
	registerProperty(prop_window_name);
	registerProperty(prop_coordinate_system);
	registerProperty(prop_background_color);

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
	registerProperty(display_bounding_box);
	registerProperty(tx);
	registerProperty(ty);
	registerProperty(tz);
	registerProperty(display_one_cluster);
	registerProperty(display_cluster);
    registerProperty(display_clusters);
    registerProperty(display_clusters_bounding_boxes);

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

	// Initialize viewer to NULL!
	viewer = NULL;
}

CorrespondencesViewer::~CorrespondencesViewer() {
}

void CorrespondencesViewer::onCSShowClick(const bool new_show_cs_){
    CLOG(LDEBUG) << "CorrespondencesViewer::onCSShowClick show="<<new_show_cs_;

    if (!viewer)
    	return;

    if(new_show_cs_) {
	if (new_show_cs_) {
#if PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->addCoordinateSystem ();
#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
	} else {
#if PCL_VERSION_COMPARE(>=,1,7,1)
		viewer->removeCoordinateSystem ();
#endif
	// TODO: Currently only 1.7.1 is available in the 012/031 laboratories.
	// TODO: Fix for other versions of PCL.
	}
    }

    prop_coordinate_system = new_show_cs_;
}

void CorrespondencesViewer::onBackgroundColorChange(std::string color_) {
	CLOG(LDEBUG) << "CorrespondencesViewer::onBackgroundColorChange color=" << color_;
	try {
		// Parse string.
		vector<std::string> strs;
		boost::split(strs, color_, boost::is_any_of(","));
		if (strs.size() != 3)
			throw std::exception();

		// Try to cast to double and divide by 255.
		double r = boost::lexical_cast<double>(strs[0]) /255;
		double g = boost::lexical_cast<double>(strs[1]) /255;
		double b = boost::lexical_cast<double>(strs[2]) /255;

		CLOG(LINFO) << "CorrespondencesViewer::onBackgroundColorChange r=" << r << " g=" << g << " b=" << b;
		// Change background color.
		if (viewer)
			viewer->setBackgroundColor(r, g, b);
	} catch (...) {
		CLOG(LWARNING)
				<< "CorrespondencesViewer::onBackgroundColorChange failed - invalid color format. Accepted format: r,g,b";
	}

}
void CorrespondencesViewer::displayCorrespondences(){
    CLOG(LTRACE) << "CorrespondencesViewer::displayCorrespondences";
    //Display clustered correspondences
    for(int i = 0; i < clusters; i++){
        ostringstream ss;
        ss << i;
        string str = ss.str();
        viewer->removeCorrespondences(std::string("correspondences")+str) ;
    }
    clusters = clustered_corrs.size();

    //Remove bounding boxes
    viewer->removeAllShapes();

    //If no clustered corrs display corrs from in_correspondences and in_good_correspondences
    if(clusters == 0){
        CLOG(LTRACE) << "CorrespondencesViewer no clusters";
        //Display correspondences
        if(!in_correspondences.empty())
            correspondences = in_correspondences.read();
        viewer->removeCorrespondences("correspondences");
        if(display_correspondences){
            viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift1, cloud_xyzsift2trans, *correspondences, "correspondences") ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                ((cv::Mat)correspondences_colours).at<uchar>(0, 0),
                ((cv::Mat)correspondences_colours).at<uchar>(0, 1),
                ((cv::Mat)correspondences_colours).at<uchar>(0, 2),
                "correspondences") ;
        }
        //Display good correspondences
        if(!in_good_correspondences.empty())
            good_correspondences = in_good_correspondences.read();
        viewer->removeCorrespondences("good_correspondences");
        if (display_good_correspondences){
            viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift1, cloud_xyzsift2trans, *good_correspondences, "good_correspondences") ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "good_correspondences") ;
        }
    }

    //Display clustered corrs
    else if(display_correspondences){
        CLOG(LTRACE) << "CorrespondencesViewer Display clusters";
        viewer->removeCorrespondences("correspondences");
        viewer->removeCorrespondences("good_correspondences");
        //Display only one choosen cluster
        if(display_one_cluster){
            int display_cluster_ = display_cluster;
            if(display_cluster >= clusters){
                CLOG(LTRACE) << "Less than "<< display_cluster+1 << " clusters! Displayed cluster 0";
                display_cluster_ = 0;
            }
            viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift2trans, cloud_xyzsift1, clustered_corrs[display_cluster_], "correspondences0") ;
            viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                    ((cv::Mat)correspondences_colours).at<uchar>(0, 0),
                    ((cv::Mat)correspondences_colours).at<uchar>(0, 1),
                    ((cv::Mat)correspondences_colours).at<uchar>(0, 2),
                    "correspondences0") ;
            //Display Bounding Box
            if(display_bounding_box){
                CLOG(LTRACE) << "CorrespondencesViewer Display Bounding Box";
                vector<int> indices;
                for(int i = 0; i < clustered_corrs[display_cluster_].size(); i++){
                    indices.push_back(clustered_corrs[display_cluster_][i].index_match);
                }
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D(*cloud_xyzsift1, indices, min_pt, max_pt);
                viewer->addCube (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], 255, 255, 255);

            }
        }
        //Display all clusters
        else{
            for(int i = 0; i< clustered_corrs.size(); i++){
//            	// Random colors for given cluster.
//                int c[3], index, r,g,b;
//            	index = rand()%3;
//                c[index+i] = 255;
//                c[(index+1+i)%3] = 0;
//                c[(index+2+i)%3] = 100 + rand()%155;
//                r=c[0];g=c[1];b=c[2];
//                //std::cout<<r<<","<<g<<","<<b<<endl;

                ostringstream ss;
                ss << i;
                string str = ss.str();
                viewer->addCorrespondences<PointXYZSIFT>(cloud_xyzsift2trans, cloud_xyzsift1, clustered_corrs[i], "correspondences"+str) ;
                viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                    colors[i][0],
                    colors[i][1],
                    colors[i][2],
                    "correspondences"+str) ;
                //Display Bounding Box
                if(display_bounding_box){
                    CLOG(LTRACE) << "CorrespondencesViewer Display Bounding Box";
                    vector<int> indices;
                    for(int j = 0; j < clustered_corrs[i].size(); j++){
                        indices.push_back(clustered_corrs[i][j].index_match);
                    }
                    Eigen::Vector4f min_pt, max_pt;
                    pcl::getMinMax3D(*cloud_xyzsift1, indices, min_pt, max_pt);
                    viewer->addCube (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], colors[i][0], colors[i][1], colors[i][2], "cube"+str);

                }
            }
        }
    }
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
    registerStream("in_projections", &in_projections);
    registerStream("in_clusters", &in_clusters);

	// Register handlers
    registerHandler("on_clouds", boost::bind(&CorrespondencesViewer::on_clouds, this));
	addDependency("on_clouds", &in_cloud_xyzsift1);
	addDependency("on_clouds", &in_cloud_xyzsift2);
	addDependency("on_clouds", &in_cloud_xyzrgb1);
    //addDependency("on_clouds", &in_correspondences);
	addDependency("on_clouds", &in_cloud_xyzrgb2);	
    registerHandler("on_projections", boost::bind(&CorrespondencesViewer::on_projections, this));
    addDependency("on_projections", &in_projections);
    registerHandler("on_clusters", boost::bind(&CorrespondencesViewer::on_clusters, this));
    addDependency("on_clusters", &in_clusters);
	// Register spin handler.
    registerHandler("on_spin", boost::bind(&CorrespondencesViewer::on_spin, this));
	addDependency("on_spin", NULL);
}

bool CorrespondencesViewer::onInit() {
	LOG(LTRACE) << "CorrespondencesViewer::onInit";

    cloud_xyzrgb1 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_xyzrgb2 = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_xyzsift1 = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    cloud_xyzsift2 = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    cloud_xyzrgb2trans = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_xyzsift2trans = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());

    correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences());
    good_correspondences = pcl::CorrespondencesPtr(new pcl::Correspondences());

	viewer = new pcl::visualization::PCLVisualizer (prop_window_name);

	// Try to change background color.
	onBackgroundColorChange(prop_background_color);

	// Display/hide coordinate system.
	onCSShowClick(prop_coordinate_system);


	viewer->initCameraParameters ();
	//cloud_view_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
    clusters = 0;
    projections_number = 0;
    clusters_clouds = 0;

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
    cloud_xyzrgb1 = in_cloud_xyzrgb1.read();
    cloud_xyzrgb2 = in_cloud_xyzrgb2.read();
    cloud_xyzsift1 = in_cloud_xyzsift1.read();
    cloud_xyzsift2 = in_cloud_xyzsift2.read();
    //pcl::CorrespondencesPtr correspondences = in_correspondences.read();
	
	//*cloud_view_xyzsift = *cloud_xyzsift1;
	
	//Define a small translation between clouds	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity() ;
	//float tx = 0.3f, ty = 0.0f, tz = 0.0f ;
	trans(0, 3) = tx ; trans(1, 3) = ty ; trans(2, 3) = tz ;

	//Transform one of the clouds	
	pcl::transformPointCloud(*cloud_xyzrgb2, *cloud_xyzrgb2trans, trans) ;
	pcl::transformPointCloud(*cloud_xyzsift2, *cloud_xyzsift2trans, trans) ;
	
	//Display clouds	
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
        cloud_xyzrgb2trans->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud_xyzrgb2trans, *cloud_xyzrgb2trans, indices);
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

    if(in_clustered_correspondences.empty()){
        clustered_corrs.clear();
    }
    else{
        clustered_corrs = in_clustered_correspondences.read();
    }

    //Display correspondences
    displayCorrespondences();
}

void CorrespondencesViewer::on_projections() {
    LOG(LTRACE) << "ClustersViewer::on_projections";
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> projections = in_projections.read();
    //viewer->removeAllShapes();
    for(int i = 0; i < projections_number; i++){
        char id = '0' + i;
        viewer->removeShape(std::string("line") + id + std::string("01"));
        viewer->removeShape(std::string("line") + id + std::string("12"));
        viewer->removeShape(std::string("line") + id + std::string("23"));
        viewer->removeShape(std::string("line") + id + std::string("30"));
        viewer->removeShape(std::string("line") + id + std::string("04"));
        viewer->removeShape(std::string("line") + id + std::string("15"));
        viewer->removeShape(std::string("line") + id + std::string("26"));
        viewer->removeShape(std::string("line") + id + std::string("37"));
        viewer->removeShape(std::string("line") + id + std::string("45"));
        viewer->removeShape(std::string("line") + id + std::string("56"));
        viewer->removeShape(std::string("line") + id + std::string("67"));
        viewer->removeShape(std::string("line") + id + std::string("74"));
    }
    projections_number = projections.size();

    for(int i = 0; i < projections.size() && i < 10; i++){
        LOG(LTRACE) << "draw projection "<< i <<endl;
        char id = '0' + i;

        viewer->addLine(projections[i]->at(0), projections[i]->at(1), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("01"));
        viewer->addLine(projections[i]->at(1), projections[i]->at(2), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("12"));
        viewer->addLine(projections[i]->at(2), projections[i]->at(3), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("23"));
        viewer->addLine(projections[i]->at(3), projections[i]->at(0), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("30"));
        viewer->addLine(projections[i]->at(0), projections[i]->at(4), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("04"));
        viewer->addLine(projections[i]->at(1), projections[i]->at(5), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("15"));
        viewer->addLine(projections[i]->at(2), projections[i]->at(6), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("26"));
        viewer->addLine(projections[i]->at(3), projections[i]->at(7), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("37"));
        viewer->addLine(projections[i]->at(4), projections[i]->at(5), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("45"));
        viewer->addLine(projections[i]->at(5), projections[i]->at(6), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("56"));
        viewer->addLine(projections[i]->at(6), projections[i]->at(7), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("67"));
        viewer->addLine(projections[i]->at(7), projections[i]->at(4), colors[i][0], colors[i][1], colors[i][2], std::string("line") + id + std::string("74"));
    }
}

void CorrespondencesViewer::on_clusters() {
    LOG(LTRACE) << "CorrespondencesViewer::on_clusters";
    //TODO display bounding boxes
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = in_clusters.read();
    if(!display_clusters){
        for(int i = 0; i < clusters_clouds; i++){
            char id = '0' + i;
            viewer->removePointCloud(std::string("cluster") + id);
            LOG(LTRACE) << "remove cluster "<< i <<endl;
        }
        clusters_clouds = 0;
        return;
    }

    if (clusters.size() >  clusters_clouds)
        for(int i = clusters_clouds; i < clusters.size() && i<10; i++){
            char id = '0' + i;
            viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), std::string("cluster") + id);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, std::string("cluster") + id);
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                        colors[i][0],
                                                        colors[i][1],
                                                        colors[i][2],
                                                        std::string("cluster") + id);
            LOG(LTRACE) << "add cluster cloud "<< i <<endl;
        }
    else if (clusters.size()<clusters_clouds)
        for(int i = clusters.size(); i < clusters_clouds; i++){
            char id = '0' + i;
            viewer->removePointCloud(std::string("cluster") + id);
            LOG(LTRACE) << "remove cluster "<< i <<endl;
        }

    clusters_clouds = clusters.size();
    if (clusters_clouds > 10)
        clusters_clouds = 10;


    for(int i = 0; i < clusters_clouds; i++){
        char id = '0' + i;
        viewer->updatePointCloud(clusters[i],std::string("cluster") + id);
        LOG(LTRACE) << "update cluster "<< i <<endl;
    }
}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}


} //: namespace CorrespondencesViewer
} //: namespace Processors
