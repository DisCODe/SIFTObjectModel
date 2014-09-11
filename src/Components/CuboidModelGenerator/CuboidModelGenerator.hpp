/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef CUBOIDMODELGENERATOR_HPP_
#define CUBOIDMODELGENERATOR_HPP_

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
#include <Types/SIFTObjectModel.hpp>
#include <Types/SIFTObjectModelFactory.hpp>

namespace Processors {
namespace CuboidModelGenerator {

/*!
 * \class CuboidModelGenerator
 * \brief CuboidModelGenerator processor class.
 *
 * 
 */
class CuboidModelGenerator: public Base::Component, SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	CuboidModelGenerator(const std::string & name = "CuboidModelGenerator");

	/*!
	 * Destructor
	 */
	virtual ~CuboidModelGenerator();

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
	Base::DataStreamOut<AbstractObject*> out_model;
    Base::DataStreamOut<int> out_mean_viewpoint_features_number;
	// Handlers
	Base::EventHandler2 h_generate;

	// Properties
	Base::Property<std::string> dataJSONname;


	
	// Handlers
	void generate();


    cv::Mat left;
    cv::Mat right;
    cv::Mat top;
    cv::Mat bottom;
    cv::Mat front;
    cv::Mat back;
    int a,b,c;

};

} //: namespace CuboidModelGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CuboidModelGenerator", Processors::CuboidModelGenerator::CuboidModelGenerator)

#endif /* CUBOIDMODELGENERATOR_HPP_ */
