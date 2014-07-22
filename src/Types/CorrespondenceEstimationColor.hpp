/*
 * CorrespondanceEstimationColor.hpp
 *
 *  Created on: 27 cze 2014
 *      Author: mlepicka
 */

#ifndef CORRESPONDENCE_ESTIMATION_COLOR_H_
#define CORRESPONDENCE_ESTIMATION_COLOR_H_

#include <string>
#include <iostream>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>

namespace pcl {
namespace registration {
template<typename PointSource, typename PointTarget, typename Scalar = float>
class CorrespondenceEstimationColor: // public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>,
public CorrespondenceEstimation<PointSource, PointTarget, Scalar> {
public:
	typedef boost::shared_ptr<
			CorrespondenceEstimation<PointSource, PointTarget, Scalar> > Ptr;
	typedef boost::shared_ptr<
			const CorrespondenceEstimation<PointSource, PointTarget, Scalar> > ConstPtr;

	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
	using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
	using PCLBase<PointSource>::deinitCompute;

	typedef pcl::search::KdTree<PointTarget> KdTree;
	typedef typename pcl::search::KdTree<PointTarget>::Ptr KdTreePtr;

	typedef pcl::PointCloud<PointSource> PointCloudSource;
	typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
	typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

	typedef pcl::PointCloud<PointTarget> PointCloudTarget;
	typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
	typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

	typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

	/** \brief Empty constructor. */
	CorrespondenceEstimationColor() {
		corr_name_ = "CorrespondenceEstimationColor";
	}

	/** \brief Empty destructor */
	virtual ~CorrespondenceEstimationColor() {
	}

	/** \brief Determine the correspondences between input and target cloud.
	 * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
	 * \param[in] max_distance maximum allowed distance between correspondences
	 */
	virtual void determineCorrespondences(pcl::Correspondences &correspondences,
			double max_distance = std::numeric_limits<double>::max()) {

		if (!initCompute())
			return;

		double max_dist_sqr = max_distance * max_distance;

		correspondences.resize(indices_->size());

		std::vector<int> index(1);
		std::vector<float> distance(1);
		std::vector<float> distanceRGB(1);

		//narazie na pale
		float minRGB = 2000;
		int minIndex = -2;
		float distance_ = 1000;

		pcl::Correspondence corr;
		unsigned int nr_valid_correspondences = 0;

		// Check if the template types are the same. If true, avoid a copy.
		// Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
		if (isSamePointType<PointSource, PointTarget>()) {
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx) {

				int r = input_->points[*idx].r;
				int g = input_->points[*idx].g;
				int b = input_->points[*idx].b;

				tree_->nearestKSearch (input_->points[*idx], 25, index, distance);
				float mindist = distance[24];
				float minrgbdist = 999999;
				int minind = 24;
				for (int i = 0; i < index.size(); i++) {

					int rr = target_->points[index[i]].r;
					int gg = target_->points[index[i]].g;
					int bb = target_->points[index[i]].b;

					float rd = r - rr;
					float gd = g - gg;
					float bd = b - bb;

					float distancergb = sqrt(rd*rd + gd*gd + bd*bd);

					if (distancergb < minrgbdist) {
						minrgbdist = distancergb;
						minind = i;
						mindist = distance[i];
					}
				}
				if (mindist > max_dist_sqr)
					continue;

				corr.index_query = *idx;
				corr.index_match = index[minind];
				corr.distance = mindist;
				correspondences[nr_valid_correspondences++] = corr;
			}
		} else {
			PointTarget pt;
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {
				// Copy the source data to a target PointTarget format so we can search in the tree
				pt = input_->points[*idx];

				tree_->nearestKSearch(pt, 1, index, distance);
				if (distance[0] > max_dist_sqr)
					continue;

				corr.index_query = *idx;
				corr.index_match = index[0];
				corr.distance = distance[0];
				correspondences[nr_valid_correspondences++] = corr;
			}
		}
		correspondences.resize(nr_valid_correspondences);
		deinitCompute();
	}

	/** \brief Determine the reciprocal correspondences between input and target cloud.
	 * A correspondence is considered reciprocal if both Src_i has Tgt_i as a
	 * correspondence, and Tgt_i has Src_i as one.
	 *
	 * \param[out] correspondences the found correspondences (index of query and target point, distance)
	 * \param[in] max_distance maximum allowed distance between correspondences
	 */
	virtual void determineReciprocalCorrespondences(
			pcl::Correspondences &correspondences, double max_distance =
					std::numeric_limits<double>::max()) {

		if (!initCompute())
			return;

		// setup tree for reciprocal search
		// Set the internal point representation of choice
		if (!initComputeReciprocal())
			return;
		double max_dist_sqr = max_distance * max_distance;

		correspondences.resize(indices_->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		std::vector<int> index_reciprocal(1);
		std::vector<float> distance_reciprocal(1);
		pcl::Correspondence corr;
		unsigned int nr_valid_correspondences = 0;
		int target_idx = 0;

		// Check if the template types are the same. If true, avoid a copy.
		// Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
		if (isSamePointType<PointSource, PointTarget>()) {
			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {
				tree_->nearestKSearch(input_->points[*idx], 1, index, distance);

				if (distance[0] > max_dist_sqr)
					continue;

				target_idx = index[0];

				tree_reciprocal_->nearestKSearch(target_->points[target_idx], 1,
						index_reciprocal, distance_reciprocal);
				if (distance_reciprocal[0] > max_dist_sqr
						|| *idx != index_reciprocal[0])
					continue;

				corr.index_query = *idx;
				corr.index_match = index[0];
				corr.distance = distance[0];
				correspondences[nr_valid_correspondences++] = corr;
			}
		} else {
			PointTarget pt_src;
			PointSource pt_tgt;

			// Iterate over the input set of source indices
			for (std::vector<int>::const_iterator idx = indices_->begin();
					idx != indices_->end(); ++idx) {
				// Copy the source data to a target PointTarget format so we can search in the tree
				pt_src = input_->points[*idx];

				tree_->nearestKSearch(pt_src, 1, index, distance);
				//

				if (distance[0] > max_dist_sqr)
					continue;

				target_idx = index[0];

				// Copy the target data to a target PointSource format so we can search in the tree_reciprocal
				pt_tgt = target_->points[target_idx];

				tree_reciprocal_->nearestKSearch(pt_tgt, 1, index_reciprocal,
						distance_reciprocal);
				if (distance_reciprocal[0] > max_dist_sqr
						|| *idx != index_reciprocal[0])
					continue;

				corr.index_query = *idx;
				corr.index_match = index[0];
				corr.distance = distance[0];
				correspondences[nr_valid_correspondences++] = corr;
			}
		}
		correspondences.resize(nr_valid_correspondences);
		deinitCompute();
	}

	/** \brief Clone and cast to CorrespondenceEstimationBase */
	virtual boost::shared_ptr<
			CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > clone() const {
		Ptr copy(
				new CorrespondenceEstimation<PointSource, PointTarget, Scalar>(
						*this));
		return (copy);
	}
};
}
}

#include <pcl/registration/impl/correspondence_estimation.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_ */

