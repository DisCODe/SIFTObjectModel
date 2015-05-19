/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef GREEDYVERIFICATION_HPP_
#define GREEDYVERIFICATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <pcl/recognition/hv/greedy_verification.h>

namespace Processors {
namespace GreedyVerification {

/*!
 * \class GreedyVerification
 * \brief GreedyVerification processor class.
 *
 * 
 */
class GreedyVerification: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	GreedyVerification(const std::string & name = "GreedyVerification");

	/*!
	 * Destructor
	 */
	virtual ~GreedyVerification();

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
	Base::Property<float> inlier_treshold;
    Base::Property<float> lambda;

	
	// Handlers
	void verify();
    void verify_xyzrgb();
    void verify_xyzsift();
};

} //: namespace GreedyVerification
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GreedyVerification", Processors::GreedyVerification::GreedyVerification)

#endif /* GREEDYVERIFICATION_HPP_ */
