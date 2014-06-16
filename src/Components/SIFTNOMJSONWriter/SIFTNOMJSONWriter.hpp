/*!
 * \file
 * \brief 
 * \author Marta Lepicka
 */

#ifndef SIFTNOMJSONWRITER_HPP_
#define SIFTNOMJSONWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/SIFTObjectModel.hpp>

namespace Processors {
namespace SIFTNOMJSONWriter {

/*!
 * \class SIFTNOMJSONWriter
 * \brief SIFTNOMJSONWriter processor class.
 *
 * SIFTNOMJSONWriter processor.
 */
class SIFTNOMJSONWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTNOMJSONWriter(const std::string & name = "SIFTNOMJSONWriter");

	/*!
	 * Destructor
	 */
	virtual ~SIFTNOMJSONWriter();

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

	/*!
	 * Callback called when name of the SOM is changed
	 */
	void onSOMNameChanged(const std::string & old_SOMname, const std::string & new_SOMname);

	/*!
	 * Callback called when dir is changed
	 */
	void onDirChanged(const std::string & old_dir, const std::string & new_dir);


	/// Input data stream containing SIFT Object Model
	Base::DataStreamIn<SIFTObjectModel*, Base::DataStreamBuffer::Newest> in_som;

// Input data streams

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzrgb_normals;


	/// Input data stream containing object model feature cloud (SIFTs).
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest> in_cloud_xyzsift;

	// Input stream containing mean number of features per view.
	Base::DataStreamIn<int, Base::DataStreamBuffer::Newest> in_mean_viewpoint_features_number;

// Output data streams

	// Handlers
	Base::EventHandler2 h_Write_normals;
	
	// Handlers
	void Write_normals();

	/// Name of the model - used for generation of names of JSON ("major" file) and PCDs ("minor" files containing clouds).
	Base::Property<std::string> SOMname;

	/// Directory to which model will be saved.
	Base::Property<std::string> dir;
};

} //: namespace SIFTNOMJSONWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTNOMJSONWriter", Processors::SIFTNOMJSONWriter::SIFTNOMJSONWriter)

#endif /* SIFTNOMJSONWRITER_HPP_ */
