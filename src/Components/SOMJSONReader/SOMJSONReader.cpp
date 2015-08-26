/*!
 * \file
 * \brief
 * \author tkornuta,,,
 */

#include <memory>
#include <string>

#include "SOMJSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <pcl/common/common.h>

// For mesh reconstruction.
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>


using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SOMJSONReader {

std::string dirnameOf(const std::string& fname)
{
     size_t pos = fname.find_last_of("\\/");
     return (std::string::npos == pos)
         ? ""
         : fname.substr(0, pos);
}

SOMJSONReader::SOMJSONReader(const std::string & name) :
	Base::Component(name),
	prop_load_on_init("load_on_init", true),
	prop_filenames("filenames", std::string(""))
{
	registerProperty(prop_filenames);
	registerProperty(prop_load_on_init);

}

SOMJSONReader::~SOMJSONReader() {
}


void SOMJSONReader::prepareInterface() {
	// Register data streams.
	registerStream("out_models", &out_models);
	registerStream("out_model_labels", &out_model_names);
	registerStream("out_model_clouds_xyzrgb", &out_model_clouds_xyzrgb);
	registerStream("out_model_clouds_xyzsift", &out_model_clouds_xyzsift);
	registerStream("out_model_vertices_xyz", &out_model_vertices_xyz);
	registerStream("out_model_triangles", &out_model_triangles);
	registerStream("out_model_bounding_boxes", &out_model_bounding_boxes);

	// Register manually triggered handlers.
	registerHandler("loadModelsButtonPressed", boost::bind(&SOMJSONReader::loadModelsButtonPressed, this));
	registerHandler("clearModelsButtonPressed", boost::bind(&SOMJSONReader::clearModelsButtonPressed, this));

	// Register handler - unconditioned model publication
	registerHandler("publishModels", boost::bind(&SOMJSONReader::publishModels, this));
	addDependency("publishModels", NULL);
}


bool SOMJSONReader::onInit() {
	CLOG(LTRACE) << "onInit()";

	// Set flags.
	clear_models = false;

	// Load models at start - if property set.
	load_models = prop_load_on_init;

	return true;
}


bool SOMJSONReader::onFinish() {
	return true;
}


bool SOMJSONReader::onStop() {
	return true;
}


bool SOMJSONReader::onStart() {
	return true;
}


void SOMJSONReader::publishModels() {
	CLOG(LTRACE) << "publishModels()";

	// Check user input.
	if (load_models)
		loadModels();

	if (clear_models)
		clearModels();

	CLOG(LDEBUG) << "Publishing " << models.size() << " SIFT Object Models";

	// Push models to output datastream -- DEPRICATED.
	out_models.write(models);
	
	// New: dataports containing simple types.
	out_model_names.write(model_names);
	out_model_clouds_xyzrgb.write(model_clouds_xyzrgb);
	out_model_clouds_xyzsift.write(model_clouds_xyzsift);
	out_model_vertices_xyz.write(model_vertices_xyz);
	out_model_triangles.write(model_triangles);
	out_model_bounding_boxes.write(model_bounding_boxes);
}


void SOMJSONReader::addPointToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_xyz_, double x_, double y_, double z_) {
	pcl::PointXYZ tmpPt;
	tmpPt.x = x_;
	tmpPt.y = y_;
	tmpPt.z = z_;
	cloud_xyz_->push_back(tmpPt);
}


void SOMJSONReader::addTriangleToList(std::vector< ::pcl::Vertices> & triangles_, int index_0, int index_1, int index_2) {
	pcl::Vertices tr;
	tr.vertices.push_back(index_0);
	tr.vertices.push_back(index_1);
	tr.vertices.push_back(index_2);
	tr.vertices.push_back(index_0);
	triangles_.push_back(tr);
}

void SOMJSONReader::addLineToList(std::vector< ::pcl::Vertices> & lines_, int index_0, int index_1) {
	pcl::Vertices tr;
	tr.vertices.push_back(index_0);
	tr.vertices.push_back(index_1);
	lines_.push_back(tr);
}

void SOMJSONReader::loadModels() {
	CLOG(LTRACE) << "loadModels()";

	load_models = false;

	// DEPRICATED.
	models.clear();

	// New: dataports containing simple types.
	model_names.clear();
	model_clouds_xyzrgb.clear();
	model_clouds_xyzsift.clear();
	model_vertices_xyz.clear();
	model_triangles.clear();
	model_bounding_boxes.clear();

	// Names of models/JSON files.	
	std::vector<std::string> namesList;
	std::string s= prop_filenames;
	boost::split(namesList, s, boost::is_any_of(";"));

	// Temporary variables - names.
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;

	
	// Iterate through JSON files.
	for (size_t i = 0; i < namesList.size(); i++){
		ptree ptree_file;
		try{
			// Open JSON file and load it to ptree.
			read_json(namesList[i], ptree_file);
			// Read JSON properties.
			model_name = ptree_file.get<std::string>("name");
			mean_viewpoint_features_number = ptree_file.get<int>("mean_viewpoint_features_number");

			// Get path to files.
			std::string dir = dirnameOf(namesList[i]);
			if (dir != "")
				dir = dir + "/";

			name_cloud_xyzrgb = dir + ptree_file.get<std::string>("cloud_xyzrgb");
			name_cloud_xyzsift = dir + ptree_file.get<std::string>("cloud_xyzsift");

		}//: try
		catch(std::exception const& e){
			CLOG(LERROR) << "SOMJSONReader: file "<< namesList[i] <<" not found or invalid\n";
			continue;	
		}//: catch
		CLOG(LINFO) << "Loading SOM from JSON file:" << namesList[i];

		CLOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
		CLOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
		

		// Read XYZRGB cloud.
		cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name_cloud_xyzrgb, *cloud_xyzrgb) == -1) 
		{
			CLOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzrgb <<" not found\n";
			continue;
		}//: if

		// Read XYZRGB cloud.
		cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_cloud_xyzsift, *cloud_xyzsift) == -1) 
		{
			CLOG(LERROR) << "SOMJSONReader: file "<< name_cloud_xyzsift <<" not found\n";
			continue;
		}//: if


		// Add data (names/clouds) to lists.
		model_names.push_back(model_name);
		model_clouds_xyzrgb.push_back(cloud_xyzrgb);
		model_clouds_xyzsift.push_back(cloud_xyzsift);


		// DEPRICATED!!
		// Generate clouds consisting of eight model corners.
		pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
		// Concatenate cloud - get only XYZ coordinates.
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		copyPointCloud(*cloud_xyzrgb, *tmp_cloud_xyz);

		pcl::PointXYZ minPt, maxPt, tmpPt;
		pcl::getMinMax3D(*tmp_cloud_xyz, minPt, maxPt);
		// Get width, height and length.
		float dx = maxPt.x - minPt.x;
		float dy = maxPt.y - minPt.y;
		float dz = maxPt.z - minPt.z;

		// Add corners to cloud.
		//   7 --- 6
		//  /     /|
		// 3 --- 2 |
		// | 4   | 5
		// |/    |/
		// 0 --- 1
		vertices_xyz->push_back(minPt); //0 left-bottom-front
		addPointToCloud(vertices_xyz, minPt.x + dx, minPt.y, minPt.z);  //1
		addPointToCloud(vertices_xyz, minPt.x + dx, minPt.y + dy, minPt.z);  //2
		addPointToCloud(vertices_xyz, minPt.x, minPt.y + dy, minPt.z);  //3
		addPointToCloud(vertices_xyz, minPt.x, minPt.y, minPt.z + dz);  //4
		addPointToCloud(vertices_xyz, minPt.x + dx, minPt.y, minPt.z + dz);  //5
		vertices_xyz->push_back(maxPt); //6
		addPointToCloud(vertices_xyz, minPt.x , minPt.y + dy, minPt.z + dz);  //7

		// Add corners.
		model_vertices_xyz.push_back(vertices_xyz);


		// Generate triangles - for meshes.
		// Connect vertices into triangles.
		std::vector<pcl::Vertices> triangles;
		// Back.
		addTriangleToList(triangles, 0, 1, 2);
		addTriangleToList(triangles, 0, 2, 3);
		// Front.
		addTriangleToList(triangles, 4, 5, 6);
		addTriangleToList(triangles, 4, 6, 7);

		// Left.
		addTriangleToList(triangles, 0, 4, 7);
		addTriangleToList(triangles, 0, 7, 3);

		// Right.
		addTriangleToList(triangles, 1, 5, 6);
		addTriangleToList(triangles, 1, 6, 2);

		// Bottom.
		addTriangleToList(triangles, 0, 1, 5);
		addTriangleToList(triangles, 0, 5, 4);

		// Top.
		addTriangleToList(triangles, 3, 2, 6);
		addTriangleToList(triangles, 3, 6, 7);

		model_triangles.push_back(triangles);

		// Generate lines - for bounding boxes.
		// Connect vertices into lines.
		std::vector<pcl::Vertices> lines;
		addLineToList(lines, 0, 1);
		addLineToList(lines, 1, 2);
		addLineToList(lines, 2, 3);
		addLineToList(lines, 3, 0);
		addLineToList(lines, 4, 5);
		addLineToList(lines, 5, 6);
		addLineToList(lines, 6, 7);
		addLineToList(lines, 7, 4);
		addLineToList(lines, 0, 4);
		addLineToList(lines, 1, 5);
		addLineToList(lines, 2, 6);
		addLineToList(lines, 3, 7);

		model_bounding_boxes.push_back(lines);

		// Create mesh object.
		//pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
		// mesh_ptr_->polygons[triangle_number].vertices[point_number];
		// Set mesh vertices.
		//pcl::toPCLPointCloud2(*corners_xyz, mesh->cloud);
		//mesh->polygons = polygons;
		// Set polygons.
		//object_meshes.push_back(mesh);

/*		viewer->removeShape(cname + std::string("01"));
		viewer->removeShape(cname + std::string("12"));
		viewer->removeShape(cname + std::string("23"));
		viewer->removeShape(cname + std::string("30"));
		viewer->removeShape(cname + std::string("04"));
		viewer->removeShape(cname + std::string("15"));
		viewer->removeShape(cname + std::string("26"));
		viewer->removeShape(cname + std::string("37"));
		viewer->removeShape(cname + std::string("45"));
		viewer->removeShape(cname + std::string("56"));
		viewer->removeShape(cname + std::string("67"));
		viewer->removeShape(cname + std::string("74"));*/



/*

		CLOG(LERROR) << "Please wait - generating mesh for object model " << model_name <<"...";
		// Create XYZ point cloud from XYZRGB.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);

		// Estimate normal vectors.
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud_xyz);
		normal_estimator.setInputCloud (cloud_xyz);
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setKSearch (30);
		normal_estimator.compute (*normals);

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_xyznormals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud_xyz, *normals, *cloud_xyznormals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_xyznormals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (0.05);

		// Set typical values for the parameters
		//gp3.setMu (2.5);
		//gp3.setMaximumNearestNeighbors (200);
		//gp3.setMaximumSurfaceAngle(M_PI/2); // 90 degrees
		//gp3.setMinimumAngle(M_PI/36); // 5 degrees
		//gp3.setMaximumAngle(M_PI); // 180 degrees
		//gp3.setNormalConsistency(false);

		gp3.setSearchRadius (5);
		gp3.setMu (2.5);//TODO TRY (1?)
		gp3.setMaximumNearestNeighbors (20);
		gp3.setMaximumSurfaceAngle(M_PI/1.2); //90// 45 degrees
		gp3.setMinimumAngle(M_PI/50);
		gp3.setMaximumAngle(2*M_PI/2.5); // 120 degrees
		gp3.setNormalConsistency(false);


		// Reconstruct object mesh.
		gp3.setInputCloud (cloud_xyznormals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (*triangles);

		object_meshes.push_back(triangles);
		CLOG(LERROR) << "Mesh generation finished!";
*/

		// Create SOModel and add it to list -- DEPRICATED.
		SIFTObjectModel* model;
		model = dynamic_cast<SIFTObjectModel*>(produce());
		models.push_back(model);


		CLOG(LINFO) << "Properly loaded " << model_name << "SIFT Object Model";

	}//: for

	CLOG(LINFO) << "Loaded " << models.size() << " SIFT Object Models";
}


void SOMJSONReader::clearModels(){
	CLOG(LTRACE) << "clearModels()";

	clear_models = false;

	// New: dataports containing simple types.
	model_names.clear();
	model_clouds_xyzrgb.clear();
	model_clouds_xyzsift.clear();
	model_vertices_xyz.clear();
	model_triangles.clear();
	model_bounding_boxes.clear();

	// DEPRICATED.
	models.clear();
}

void SOMJSONReader::loadModelsButtonPressed(){
	CLOG(LDEBUG) << "loadModelsButtonPressed";
	load_models = true;
}



void SOMJSONReader::clearModelsButtonPressed(){
	CLOG(LDEBUG) << "clearModelsButtonPressed";
	clear_models = true;
}


} //: namespace SOMJSONReader
} //: namespace Processors
