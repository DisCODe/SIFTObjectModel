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
#include <pcl/recognition/impl/hv/greedy_verification.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>

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
    Base::DataStreamIn<std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> > in_aligned_hypotheses;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift_scene;

	// Output data streams

	// Properties
	Base::Property<float> resolution;
	Base::Property<float> inlier_treshold;
    Base::Property<float> lambda;

	
	// Handlers
	void verify();

};

} //: namespace GreedyVerification
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("GreedyVerification", Processors::GreedyVerification::GreedyVerification)

#endif /* GREEDYVERIFICATION_HPP_ */
