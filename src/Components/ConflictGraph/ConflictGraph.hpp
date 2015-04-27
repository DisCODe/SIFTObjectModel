/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CONFLICTGRAPH_HPP_
#define CONFLICTGRAPH_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointXYZSIFT.hpp>
#include <pcl/recognition/hv/hv_papazov.h>

namespace Processors {
namespace ConflictGraph {

/*!
 * \class ConflictGraph
 * \brief ConflictGraph processor class.
 *
 * 
 */
class ConflictGraph: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ConflictGraph(const std::string & name = "ConflictGraph");

	/*!
	 * Destructor
	 */
	virtual ~ConflictGraph();

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
    Base::DataStreamOut<std::vector<pcl::PointCloud<PointXYZSIFT>::ConstPtr> > out_verified_hypotheses;

	// Properties
	Base::Property<float> resolution;
	Base::Property<float> inlier_treshold;
	Base::Property<float> support_threshold;
	Base::Property<float> penalty_threshold;
	Base::Property<float> conflict_threshold;

	
	// Handlers
	void verify();

};

} //: namespace ConflictGraph
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ConflictGraph", Processors::ConflictGraph::ConflictGraph)

#endif /* CONFLICTGRAPH_HPP_ */
