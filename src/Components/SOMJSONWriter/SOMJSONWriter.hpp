/*!
 * \file
 * \brief 
 * \author tkornuta,,,
 */

#ifndef SOMJSONWRITER_HPP_
#define SOMJSONWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/SIFTObjectModel.hpp> 


namespace Processors {
namespace SOMJSONWriter {

/*!
 * \class SOMJSONWriter
 * \brief SOMJSONWriter processor class.
 *
 * SOMJSONWriter processor.
 */
class SOMJSONWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SOMJSONWriter(const std::string & name = "SOMJSONWriter");

	/*!
	 * Destructor
	 */
	virtual ~SOMJSONWriter();

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
	Base::DataStreamIn<SIFTObjectModel*> in_som;

	/// Input data stream containing object model point cloud.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	/// Input data stream containing object model feature cloud (SIFTs).
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

	// Input stream containing mean number of features per view. 
	Base::DataStreamIn<int> in_mean_viewpoint_features_number;


	// Handlers
	Base::EventHandler2 h_Write;
	
	// Handlers
	void Write();

	/// Name of the model - used for generation of names of JSON ("major" file) and PCDs ("minor" files containing clouds).
	Base::Property<std::string> SOMname;

	/// Directory to which model will be saved.
	Base::Property<std::string> dir;

};

} //: namespace SOMJSONWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMJSONWriter", Processors::SOMJSONWriter::SOMJSONWriter)

#endif /* SOMJSONWRITER_HPP_ */
