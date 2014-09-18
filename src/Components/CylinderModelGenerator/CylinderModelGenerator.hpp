/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CYLINDERMODELGENERATOR_HPP_
#define CYLINDERMODELGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include "Types/Features.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace CylinderModelGenerator {

/*!
 * \class CylinderModelGenerator
 * \brief CylinderModelGenerator processor class.
 *
 * 
 */
class CylinderModelGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CylinderModelGenerator(const std::string & name = "CylinderModelGenerator");

	/*!
	 * Destructor
	 */
	virtual ~CylinderModelGenerator();

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

    void loadData();
    void sift(cv::Mat input, cv::Mat &descriptors, Types::Features &features);

	// Input data streams

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;

	// Handlers
	Base::EventHandler2 h_generateModel;
	Base::EventHandler2 h_returnModel;

	// Properties
    /// JSON file containing geometric properties of the cuboid and list of files with textures.
    Base::Property<std::string> dataJSONname;

    Base::Property<float> step;

    Base::Property<bool> generate_on_init;
	
	// Handlers
    /// Function responsible for generation of SOM cuboid model.
    void generateModel();

    /// Function setting the generateModel flag.
    void generateModelPressed();

    /// Function responsible for cyclic return of SOM cuboid model.
    void returnModel();


    /// Image with top side texture.
    cv::Mat top;
    /// Image with bottom side texture.
    cv::Mat bottom;
    /// Image with side texture.
    cv::Mat side;

    /// Image with top side mask.
    cv::Mat top_mask;
    /// Image with bottom side mask.
    cv::Mat bottom_mask;
    /// Image with side mask.
    cv::Mat side_mask;

    /// Flag indicating that the user pressed the generateModelButton
    bool generateModel_flag;

    /// Sizes of the cuboid.
    int h,r;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;
    pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;

    bool generate_top, generate_bottom, generate_side;
    bool mask_top, mask_bottom, mask_side;

};

} //: namespace CylinderModelGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CylinderModelGenerator", Processors::CylinderModelGenerator::CylinderModelGenerator)

#endif /* CYLINDERMODELGENERATOR_HPP_ */
