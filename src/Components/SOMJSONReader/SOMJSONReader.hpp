/*!
 * \file
 * \brief 
 * \author tkornuta,,,
 */

#ifndef SOMJSONREADER_HPP_
#define SOMJSONREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/SIFTObjectModelFactory.hpp> 
#include <Types/PointXYZSIFT.hpp>

namespace Processors {
namespace SOMJSONReader {

/*!
 * \class SOMJSONReader
 * \brief SOMJSONReader processor class.
 *
 * SOMJSONReader processor.
 */
class SOMJSONReader: public Base::Component, SIFTObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	SOMJSONReader(const std::string & name = "SOMJSONReader");

	/*!
	 * Destructor
	 */
	virtual ~SOMJSONReader();

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

	/// Output data stream containing models -- DEPRICATED.
	Base::DataStreamOut<std::vector<AbstractObject*> > out_models;


	/// Output data stream containing vector of model ids.
	Base::DataStreamOut < std::vector< std::string > > out_model_names;

	/// Output data stream containing vector of XYZRGB clouds.
	Base::DataStreamOut < std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > > out_model_clouds_xyzrgb;

	/// Output data stream containing vector of XYZSIFT clouds.
	Base::DataStreamOut < std::vector< pcl::PointCloud<PointXYZSIFT>::Ptr > > out_model_clouds_xyzsift;

	/// Output data stream containing vector of model vertices (used by polygons and boundingboxes).
	Base::DataStreamOut < std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > > out_model_vertices_xyz;

	/// Output data stream containing vector of models meshes - build on top of vertices.
	Base::DataStreamOut <std::vector< std::vector<pcl::Vertices> > > out_model_triangles;

	/// Output data stream containing vector of models bounding boxes - build on top of vertices.
	Base::DataStreamOut <std::vector<std::vector<pcl::Vertices> > > out_model_bounding_boxes;

	/// Property: loads models from files at init.
	Base::Property<bool> prop_load_on_init;

	/// Property: list of the files containing models to be read.
	Base::Property<std::string> prop_filenames;

	/// Adds xyz point to refered point cloud.
	void addPointToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_xyz_, double x_, double y_, double z_);

	/// Adds triangle to refered polygon list.
	void addTriangleToList(std::vector< ::pcl::Vertices> & triangles_, int index_0, int index_1, int index_2);

	/// Adds triangle to refered polygon list.
	void addLineToList(std::vector< ::pcl::Vertices> & lines_, int index_0, int index_1);

	/// Load models from files.
	void loadModels();

	/// Set flag - load models from files.
	void loadModelsButtonPressed();


	/// Set flag - clear stored models.
	void clearModelsButtonPressed();

	/// Removes stored models from memory.
	void clearModels();


	/// Publishes stored models.
	void publishModels();

protected:

	/// Flag - load models from files.
	bool load_models;

	/// Flag - clear models.
	bool clear_models;


	/// List of the returned SOMs -- DEPRICATED.
	std::vector<AbstractObject*> models;

	/// Names of stored models.
	std::vector<std::string> model_names;

	/// Vector of XYZRGB clouds of stored models.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> model_clouds_xyzrgb;

	/// Vector of XYZSIFT clouds of stored models.
	std::vector<pcl::PointCloud<PointXYZSIFT>::Ptr> model_clouds_xyzsift;

	/// Vector of model corners (each being a cloud containing 8 XYZ points).
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> model_vertices_xyz;

	/// Vector of models meshes - build on top of vertices.
	std::vector<std::vector<pcl::Vertices> > model_triangles;

	/// Vector of models bounding boces - build on top of vertices.
	std::vector<std::vector<pcl::Vertices> > model_bounding_boxes;

};

} //: namespace SOMJSONReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SOMJSONReader", Processors::SOMJSONReader::SOMJSONReader)

#endif /* SOMJSONREADER_HPP_ */
