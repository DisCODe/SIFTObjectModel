/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef GLOBALHYPOTHESESVERIFICATION_HPP_
#define GLOBALHYPOTHESESVERIFICATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <pcl/recognition/hv/hv_go.h>

namespace Processors {
namespace GlobalHypothesesVerification {

/*!
 * \class GlobalHypothesesVerification
 * \brief GlobalHypothesesVerification processor class.
 *
 * 
 */
class GlobalHypothesesVerification: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	GlobalHypothesesVerification(const std::string & name = "GlobalHypothesesVerification");

	/*!
	 * Destructor
	 */
	virtual ~GlobalHypothesesVerification();

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
    Base::DataStreamIn<std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> > in_aligned_hypotheses_xyzsift;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift_scene;
    Base::DataStreamIn<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> > in_aligned_hypotheses_xyzrgb;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb_scene;
    Base::DataStreamIn<std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> > in_aligned_hypotheses_xyz;
    Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz_scene;
	// Output data streams
    Base::DataStreamOut<std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> > out_verified_hypotheses_xyzsift;
    Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> > out_verified_hypotheses_xyzrgb;
    Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> > out_verified_hypotheses_xyz;

	// Properties
	Base::Property<float> resolution;
	Base::Property<float> inlier_threshold;
	Base::Property<float> radius_clutter;
	Base::Property<float> regularizer;
	Base::Property<float> clutter_regularizer;
	Base::Property<bool> detect_clutter;

	
	// Handlers
	void verify();
    void verify_xyzrgb();
    void verify_xyzsift();

};

} //: namespace GlobalHypothesesVerification
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GlobalHypothesesVerification", Processors::GlobalHypothesesVerification::GlobalHypothesesVerification)

#endif /* GLOBALHYPOTHESESVERIFICATION_HPP_ */
