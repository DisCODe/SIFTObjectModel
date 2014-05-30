/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SIFTCLUSTEREXTRACTION_HPP_
#define SIFTCLUSTEREXTRACTION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Types/PointXYZSIFT.hpp> 

namespace Processors {
namespace SIFTClusterExtraction {

/*!
 * \class SIFTClusterExtraction
 * \brief SIFTClusterExtraction processor class.
 *
 * SIFTClusterExtraction processor.
 */
class SIFTClusterExtraction: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTClusterExtraction(const std::string & name = "SIFTClusterExtraction");

	/*!
	 * Destructor
	 */
	virtual ~SIFTClusterExtraction();

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

		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

// Output data streams

		Base::DataStreamOut<std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> > out_clusters;
		Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_clusters_xyz;
	// Handlers
	Base::EventHandler2 h_extract;
		Base::Property<float> clusterTolerance;
		Base::Property<int> minClusterSize;
		Base::Property<int> maxClusterSize;

	
	// Handlers
	void extract();

};

} //: namespace SIFTClusterExtraction
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTClusterExtraction", Processors::SIFTClusterExtraction::SIFTClusterExtraction)

#endif /* SIFTCLUSTEREXTRACTION_HPP_ */
