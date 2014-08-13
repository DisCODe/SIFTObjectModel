/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef FEATURECLOUDCONVERTER_HPP_
#define FEATURECLOUDCONVERTER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/CameraInfo.hpp>
#include <Types/Features.hpp> 
#include <Types/PointXYZSIFT.hpp> 

#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace FeatureCloudConverter {

/*!
 * \class FeatureCloudConverter
 * \brief FeatureCloudConverter processor class.
 *
 * FeatureCloudConverter processor.
 */
class FeatureCloudConverter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FeatureCloudConverter(const std::string & name = "FeatureCloudConverter");

	/*!
	 * Destructor
	 */
	virtual ~FeatureCloudConverter();

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


	// Input data streams
	Base::DataStreamIn<cv::Mat> in_depth;
	Base::DataStreamIn<cv::Mat> in_mask;
	Base::DataStreamIn<Types::Features> in_features;
	Base::DataStreamIn<cv::Mat> in_descriptors;
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;
    Base::DataStreamIn<cv::Mat> in_depth_xyz;

	/// Output data stream containing resulting feature cloud.
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Handlers
	Base::EventHandler2 h_process;
	Base::EventHandler2 h_process_mask;
    Base::EventHandler2 h_process_depth_xyz;
    Base::EventHandler2 h_process_depth_xyz_mask;
	// Handlers
	void process();
	void process_mask();
    void process_depth_xyz();
    void process_depth_xyz_mask();


};

} //: namespace FeatureCloudConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FeatureCloudConverter", Processors::FeatureCloudConverter::FeatureCloudConverter)

#endif /* FEATURECLOUDCONVERTER_HPP_ */
