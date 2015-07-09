/*!
 * \file
 * \brief 
 * \author Mort
 */

#ifndef GAUSSCLOUD_HPP_
#define GAUSSCLOUD_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include "Types/HomogMatrix.hpp"
#include <pcl/visualization/pcl_visualizer.h>

#include <Types/PointXYZSIFT.hpp>


namespace Processors {
namespace GaussCloud {

/*!
 * \class GaussCloud
 * \brief GaussCloud processor class.
 *
 * 
 */
class GaussCloud: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	GaussCloud(const std::string & name = "GaussCloud");

	/*!
	 * Destructor
	 */
	virtual ~GaussCloud();

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
	Base::DataStreamIn< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > in_cloud_xyzrgb;
	Base::DataStreamIn< pcl::PointCloud<PointXYZSIFT>::Ptr > in_cloud_xyzsift;

	Base::DataStreamOut< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;
	Base::DataStreamOut< pcl::PointCloud<PointXYZSIFT>::Ptr > out_cloud_xyzsift;

	// Input data streams

	// Output data streams

	// Handlers
	const static int q;
	const static float c1;
	const static float c2;
	const static float c3;
	// Properties

	//http://teleinfo.pb.edu.pl/krashan/articles/gauss/
	// Handlers
	Base::EventHandler2 h_makeNoisyCloud;
	void makeNoisyCloud();
	float generateNumber(float, float);
};

} //: namespace GaussCloud
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GaussCloud", Processors::GaussCloud::GaussCloud)

#endif /* GAUSSCLOUD_HPP_ */
