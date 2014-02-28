/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "PC2Octree.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

using namespace pcl::octree;

namespace Processors {
namespace PC2Octree {

PC2Octree::PC2Octree(const std::string & name) :
		Base::Component(name)  {

}

PC2Octree::~PC2Octree() {
}

void PC2Octree::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	// Register handlers
	h_cloud_xyzrgb_to_octree.setup(boost::bind(&PC2Octree::cloud_xyzrgb_to_octree, this));
	registerHandler("cloud_xyzrgb_to_octree", &h_cloud_xyzrgb_to_octree);
	addDependency("cloud_xyzrgb_to_octree", &in_cloud_xyzsift);


}

bool PC2Octree::onInit() {

	return true;
}

bool PC2Octree::onFinish() {
	return true;
}

bool PC2Octree::onStop() {
	return true;
}

bool PC2Octree::onStart() {
	return true;
}

void PC2Octree::cloud_xyzrgb_to_octree() {
	LOG(LTRACE) << "PC2Octree::cloud_xyzrgb_to_octree";
	// Read from dataport.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

	// Set voxel resolution.
	float voxelSize = 0.01f;
	pcl::octree::OctreePointCloud<PointXYZSIFT, pcl::octree::OctreeContainerPointIndices, int> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();

	unsigned int lastDepth = 0;
	unsigned int branchNodeCount = 0;
	unsigned int leafNodeCount = 0;
	unsigned int maxLeafContainerSize = 0;

	// Use breadth-first iterator.
	pcl::octree::OctreePointCloud<PointXYZSIFT, pcl::octree::OctreeContainerPointIndices, int>::BreadthFirstIterator bfIt;
	const pcl::octree::OctreePointCloud<PointXYZSIFT, pcl::octree::OctreeContainerPointIndices, int>::BreadthFirstIterator bfIt_end = octree.breadth_end();

	for (bfIt = octree.breadth_begin(); bfIt != bfIt_end; ++bfIt)
	{
		LOG(LINFO) << "depth = " << bfIt.getCurrentOctreeDepth ();
		// Get node from iterator.
		pcl::octree::OctreeNode* node = bfIt.getCurrentOctreeNode(); 
		if (node->getNodeType () == BRANCH_NODE) 
		{
			// Current node is a branch node.
			LOG(LINFO) << "BRANCH";
			// Cast to proper data structure.
			OctreeBranchNode<PointXYZSIFT>* branch_node =   static_cast<OctreeBranchNode<PointXYZSIFT>*> (node);
			// Iterate over all children.
			unsigned char child_idx;
			for (child_idx = 0; child_idx < 8 ; ++child_idx)
			{
				// Check whether given child exists.
				if (branch_node->hasChild(child_idx))
				{
					LOG(LINFO) << "Child "<<(int)child_idx << "present";
//					BranchNode* current_branch = octree->getBranchChildPtr(*current_branch, child_idx);
				}//: if
			}//: for children
			branchNodeCount++;
		}//: if branch 
	
		if (node->getNodeType () == LEAF_NODE) 
		{
			// Current node is a branch node.
			LOG(LINFO) << "LEAF";
			// Cast to proper data structure.
			OctreeLeafNode< OctreeContainerPointIndices >* leaf_node =   static_cast< OctreeLeafNode<OctreeContainerPointIndices>* > (node);
			// Get container size.
			int containter_size = leaf_node->getContainer().getSize();
			// Check whether size is proper.
			if(containter_size >8) {
				LOG(LERROR) << "Leaf containter too big! (" << containter_size << ")";
			}//: if
			// Get maximum size of container.	
			if(containter_size > maxLeafContainerSize)
				maxLeafContainerSize = containter_size;
	
			// Iterate through container elements, i.e. cloud points.
			std::vector<int> point_indices;
	 		leaf_node->getContainer().getPointIndices(point_indices);
			for(unsigned int i=0; i<leaf_node->getContainer().getSize(); i++)
			{
				LOG(LDEBUG) << "Iteration number " << i << " Point index=" << point_indices[i];
				PointXYZSIFT p = cloud->at(point_indices[i]);
				LOG(LINFO) << "p.x = " << p.x << " p.y = " << p.y << " p.z = " << p.z;
				LOG(LINFO) << "multiplicity: " << p.multiplicity;
			}//: for points		
			leafNodeCount++;
		}//: if leaf
	}//: for nodes
	LOG(LINFO) << "BranchNodeCount: " << branchNodeCount;
	LOG(LINFO) << "LeafNodeCount: " << leafNodeCount;
	LOG(LINFO) << "MaxLeafContainerSize: " << maxLeafContainerSize;

	// Delete octree data structure (pushes allocated nodes to memory pool!).
	// octree.deleteTree ();
}


} //: namespace PC2Octree
} //: namespace Processors
