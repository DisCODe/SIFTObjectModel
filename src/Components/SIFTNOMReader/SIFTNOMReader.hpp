/*!
 * \file
 * \brief 
 * \author Marta Lepicka
 */

#ifndef SIFTNOMREADER_HPP_
#define SIFTNOMREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/SIFTObjectModel.hpp>
#include <Types/SIFTObjectModelFactory.hpp>

namespace Processors {
namespace SIFTNOMReader {

/*!
 * \class SIFTNOMReader
 * \brief SIFTNOMReader processor class.
 *
 * SIFTNOMReader processor.
 */
class SIFTNOMReader: public Base::Component, SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	SIFTNOMReader(const std::string & name = "SIFTNOMReader");

	/*!
	 * Destructor
	 */
	virtual ~SIFTNOMReader();

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

	/// Output data stream containing models.
	Base::DataStreamOut<std::vector<AbstractObject*> > out_models;
	/// Output data stream containing object model point cloud with normals.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb_normals;


	// Handlers
	Base::EventHandler2 h_loadModels;

	/// List of the files containing models to be read.
	Base::Property<string> filenames;


	/// Load models from files.
	void loadModels();

	/*!
	 * Callback called when list of filenames changes.
	 */
	void onFilenamesChanged(const std::string & old_filenames, const std::string & new_filenames);
// Input data streams


// Output data streams

	// Handlers

	
	// Handlers

};

} //: namespace SIFTNOMReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTNOMReader", Processors::SIFTNOMReader::SIFTNOMReader)

#endif /* SIFTNOMREADER_HPP_ */
