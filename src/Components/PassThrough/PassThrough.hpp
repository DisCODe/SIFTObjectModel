/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef PASSTHROUGH2_HPP_
#define PASSTHROUGH2_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


#include <Types/PointXYZSIFT.hpp>
#include <Types/SIFTObjectModel.hpp>


namespace Processors {
namespace PassThrough {

/*!
 * \class PassThrough
 * \brief PassThrough processor class.
 *
 * PassThrough processor.
 */
class PassThrough: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	PassThrough(const std::string & name = "PassThrough");

	/*!
	 * Destructor
	 */
	virtual ~PassThrough();

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
    Base::DataStreamIn<SIFTObjectModel*> in_som;

// Output data streams
    Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
    Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
    Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
    Base::DataStreamOut<SIFTObjectModel*> out_som;

	// Handlers
    Base::EventHandler2 h_filter_xyz;
    Base::EventHandler2 h_filter_xyzrgb;
    Base::EventHandler2 h_filter_xyzsift;
    Base::EventHandler2 h_filter_som;

    //Properties
    Base::Property<float> xa;
    Base::Property<float> xb;
    Base::Property<float> ya;
    Base::Property<float> yb;
    Base::Property<float> za;
    Base::Property<float> zb;
    Base::Property<bool> negative_x;
    Base::Property<bool> negative_y;
    Base::Property<bool> negative_z;

	
	// Handlers
    void filter_xyz();
    void filter_xyzrgb();
    void filter_xyzsift();
    void filter_som();

    void applyFilter (pcl::PointCloud<PointXYZSIFT>::Ptr input, pcl::PointCloud<PointXYZSIFT> &output, std::string filter_field_name, float min, float max, bool negative);
    void applyFilterIndices (std::vector<int> &indices, pcl::PointCloud<PointXYZSIFT>::Ptr input, std::string filter_field_name, float min, float max, bool negative);
};

} //: namespace PassThrough
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("PassThrough", Processors::PassThrough::PassThrough)

#endif /* PASSTHROUGH2_HPP_ */
