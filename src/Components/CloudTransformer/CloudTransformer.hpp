/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CLOUDTRANSFORMER_HPP_
#define CLOUDTRANSFORMER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>
#include <Types/HomogMatrix.hpp>

namespace Processors {
namespace CloudTransformer {

/*!
 * \class CloudTransformer
 * \brief CloudTransformer processor class.
 *
 * 
 */
class CloudTransformer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CloudTransformer(const std::string & name = "CloudTransformer");

	/*!
	 * Destructor
	 */
	virtual ~CloudTransformer();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;
    Base::DataStreamIn<SIFTObjectModel*, Base::DataStreamBuffer::Newest> in_som;
    Base::DataStreamIn<Types::HomogMatrix> in_hm;
	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
    Base::DataStreamOut<SIFTObjectModel*> out_som;

	// Handlers
	Base::EventHandler2 h_fransform_xyz;
	Base::EventHandler2 h_transform_xyzrgb;
	Base::EventHandler2 h_transform_xyzsift;
    Base::EventHandler2 h_transform_som;

	// Properties

	
	// Handlers
	void fransform_xyz();
	void transform_xyzrgb();
	void transform_xyzsift();
    void transform_som();

};

} //: namespace CloudTransformer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CloudTransformer", Processors::CloudTransformer::CloudTransformer)

#endif /* CLOUDTRANSFORMER_HPP_ */
