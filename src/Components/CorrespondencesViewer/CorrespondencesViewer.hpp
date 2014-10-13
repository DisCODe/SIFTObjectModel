/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef CORRESPONDENCESVIEWER_HPP_
#define CORRESPONDENCESVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <Types/PointXYZSIFT.hpp> 

#include <Types/MatrixTranslator.hpp>
#include <opencv2/core/core.hpp>


//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

//#include <pcl/visualization/point_cloud_geometry_handlers.h>

namespace Processors {
namespace CorrespondencesViewer {

/*!
 * \class CorrespondencesViewer
 * \brief CorrespondencesViewer processor class.
 *
 * CorrespondencesViewer processor.
 */
class CorrespondencesViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CorrespondencesViewer(const std::string & name = "CorrespondencesViewer");

	/*!
	 * Destructor
	 */
	virtual ~CorrespondencesViewer();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	pcl::visualization::PCLVisualizer * viewer;
	//pcl::PointCloud<PointXYZSIFT>::Ptr cloud_view_xyzsift;

// Input data streams

	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift1;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift2;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb1;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb2;
	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondences;
	Base::DataStreamIn<pcl::CorrespondencesPtr> in_good_correspondences;
    Base::DataStreamIn<std::vector<pcl::Correspondences> > in_clustered_correspondences;
// Output data streams

	// Handlers
    Base::EventHandler2 h_on_clouds;
	Base::EventHandler2 h_on_spin;
	
	// Handlers
    void on_clouds();
	void on_spin();
	
	
	Base::Property<std::string> prop_window_name;
	Base::Property<int> cloud_xyzsift1_point_size;
	Base::Property<int> cloud_xyzsift2_point_size;
	/// Property for setting the colours of clouds. From default it will be set to 2 rows with 255, 0, 0 (red).
	Base::Property<cv::Mat, Types::MatrixTranslator> clouds_colours; 
	/// Property for setting the colour of correspondences. From default it will be set to 1 row with 255, 0, 0 (red).
	Base::Property<cv::Mat, Types::MatrixTranslator> correspondences_colours;
	
	Base::Property<bool> display_cloud_xyzrgb1;
	Base::Property<bool> display_cloud_xyzrgb2;
	Base::Property<bool> display_cloud_xyzsift1;
	Base::Property<bool> display_cloud_xyzsift2;
	Base::Property<bool> display_correspondences;
	Base::Property<bool> display_good_correspondences;
	
	Base::Property<bool> prop_coordinate_system;
	
	Base::Property<float> tx;
	Base::Property<float> ty;
	Base::Property<float> tz;

    Base::Property<bool> display_one_cluster;
    Base::Property<int> display_cluster;

    int clusters;
	
};

} //: namespace CorrespondencesViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondencesViewer", Processors::CorrespondencesViewer::CorrespondencesViewer)

#endif /* CORRESPONDENCESVIEWER_HPP_ */
