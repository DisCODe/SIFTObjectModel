/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PassThrough.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


//PCL_INSTANTIATE(PCLBase, PointXYZSIFT);
//PCL_INSTANTIATE(PassThrough, PointXYZSIFT);

namespace Processors {
namespace PassThrough {

PassThrough::PassThrough(const std::string & name) :
    Base::Component(name) ,
    xa("x.a", 0),
    xb("x.b", 0),
    ya("y.a", 0),
    yb("y.b", 0),
    za("z.a", 0),
    zb("z.b", 0),
    negative_x("negative_x", false),
    negative_y("negative_y", false),
    negative_z("negative_z", false){
    registerProperty(xa);
    registerProperty(xb);
    registerProperty(ya);
    registerProperty(yb);
    registerProperty(za);
    registerProperty(zb);
    registerProperty(negative_x);
    registerProperty(negative_y);
    registerProperty(negative_z);


}

PassThrough::~PassThrough() {
}

void PassThrough::prepareInterface() {
    // Register data streams, events and event handlers HERE!
    registerStream("in_cloud_xyz", &in_cloud_xyz);
    registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
    registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
    registerStream("in_som", &in_som);
    registerStream("out_cloud_xyz", &out_cloud_xyz);
    registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
    registerStream("out_som", &out_som);
    // Register handlers
    h_filter_xyz.setup(boost::bind(&PassThrough::filter_xyz, this));
    registerHandler("filter_xyz", &h_filter_xyz);
    addDependency("filter_xyz", &in_cloud_xyz);
    h_filter_xyzrgb.setup(boost::bind(&PassThrough::filter_xyzrgb, this));
    registerHandler("filter_xyzrgb", &h_filter_xyzrgb);
    addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
    h_filter_xyzsift.setup(boost::bind(&PassThrough::filter_xyzsift, this));
    registerHandler("filter_xyzsift", &h_filter_xyzsift);
    addDependency("filter_xyzsift", &in_cloud_xyzsift);
    h_filter_som.setup(boost::bind(&PassThrough::filter_som, this));
    registerHandler("filter_som", &h_filter_som);
    addDependency("filter_som", &in_som);

}

bool PassThrough::onInit() {
    return true;
}

bool PassThrough::onFinish() {
    return true;
}

bool PassThrough::onStop() {
    return true;
}

bool PassThrough::onStart() {
    return true;
}

void PassThrough::filter_xyz() {
    LOG(LTRACE) <<"PassThrough::filter_xyz()";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xa, xb);
    pass.setFilterLimitsNegative (negative_x);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ya, yb);
    pass.setFilterLimitsNegative (negative_y);
    pass.filter (*cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (za, zb);
    pass.setFilterLimitsNegative (negative_z);
    pass.filter (*cloud);
    out_cloud_xyz.write(cloud);
}

void PassThrough::filter_xyzrgb() {
    LOG(LTRACE) <<"PassThrough::filter_xyzrgb()";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (xa, xb);
    pass.setFilterLimitsNegative (negative_x);
    pass.filter (*cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (ya, yb);
    pass.setFilterLimitsNegative (negative_y);
    pass.filter (*cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (za, zb);
    pass.setFilterLimitsNegative (negative_z);
    pass.filter (*cloud);
    out_cloud_xyzrgb.write(cloud);
}

void PassThrough::filter_xyzsift() {
    LOG(LTRACE) <<"PassThrough::filter_xyzsift()";
        pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

        applyFilter(cloud, *cloud, "x", xa, xb, negative_x);
        applyFilter(cloud, *cloud, "y", ya, yb, negative_y);
        applyFilter(cloud, *cloud, "z", za, zb, negative_z);
    //    pcl::PassThrough<PointXYZSIFT> pass;
    //    pass.setInputCloud (cloud);
    //    pass.setFilterFieldName ("x");
    //    pass.setFilterLimits (xa, xb);
    //    pass.setFilterLimitsNegative (negative_x);
    //    pass.filter (*cloud);
    //    pass.setFilterFieldName ("y");
    //    pass.setFilterLimits (ya, yb);
    //    pass.setFilterLimitsNegative (negative_y);
    //    pass.filter (*cloud);
    //    pass.setFilterFieldName ("z");
    //    pass.setFilterLimits (za, zb);
    //    pass.setFilterLimitsNegative (negative_z);
    //    pass.filter (*cloud);
        out_cloud_xyzsift.write(cloud);
}

void PassThrough::filter_som() {
    LOG(LTRACE) <<"PassThrough::filter_som()";
    //    SIFTObjectModel* som = in_som.read();
    //    pcl::PassThrough<pcl::PointXYZRGB> pass;
    //    pcl::PassThrough<PointXYZSIFT> pass_sift;

    //    pass.setInputCloud (som->cloud_xyzrgb);
    //    pass.setFilterFieldName ("x");
    //    pass.setFilterLimits (xa, xb);
    //    pass.setFilterLimitsNegative (negative_x);
    //    pass.filter (*(som->cloud_xyzrgb));
    //    pass.setFilterFieldName ("y");
    //    pass.setFilterLimits (ya, yb);
    //    pass.setFilterLimitsNegative (negative_y);
    //    pass.filter (*(som->cloud_xyzrgb));
    //    pass.setFilterFieldName ("z");
    //    pass.setFilterLimits (za, zb);
    //    pass.setFilterLimitsNegative (negative_z);
    //    pass.filter (*(som->cloud_xyzrgb));

    //    pass_sift.setInputCloud (som->cloud_xyzsift);
    //    pass_sift.setFilterFieldName ("x");
    //    pass_sift.setFilterLimits (xa, xb);
    //    pass_sift.setFilterLimitsNegative (negative_x);
    //    pass_sift.filter (*(som->cloud_xyzsift));
    //    pass_sift.setFilterFieldName ("y");
    //    pass_sift.setFilterLimits (ya, yb);
    //    pass_sift.setFilterLimitsNegative (negative_y);
    //    pass_sift.filter (*(som->cloud_xyzsift));
    //    pass_sift.setFilterFieldName ("z");
    //    pass_sift.setFilterLimits (za, zb);
    //    pass_sift.setFilterLimitsNegative (negative_z);
    //    pass_sift.filter (*(som->cloud_xyzsift));

    //    out_som.write(som);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PassThrough::applyFilter (pcl::PointCloud<PointXYZSIFT>::Ptr input, pcl::PointCloud<PointXYZSIFT> &output, std::string filter_field_name, float min, float max, bool negative)
{
    output.header = input->header;
    output.sensor_origin_ = input->sensor_origin_;
    output.sensor_orientation_ = input->sensor_orientation_;

    std::vector<int> indices;

    output.is_dense = true;
    applyFilterIndices (indices, input, filter_field_name, min, max, negative);
    cout<< "indices size "<< indices.size() <<endl;
    copyPointCloud (*input, indices, output);

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PassThrough::applyFilterIndices (std::vector<int> &indices, pcl::PointCloud<PointXYZSIFT>::Ptr input, std::string filter_field_name, float min, float max, bool negative)
{
    //using pcl::PCLBase<PointXYZSIFT>::indices_;

    pcl::IndicesPtr indices_(new vector<int>);
    pcl::IndicesPtr removed_indices_(new vector<int>);
    //indices_->resize(input->size());
    for(int i = 0; i < input->size(); i++){
        indices_->push_back(i);
    }

    // The arrays to be used
    indices.resize (indices_->size ());
    removed_indices_->resize (indices_->size ());
    int oii = 0, rii = 0; // oii = output indices iterator, rii = removed indices iterator
    // Has a field name been specified?

    if (filter_field_name.empty ())
    {
        // Only filter for non-finite entries then
        for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii) // iii = input indices iterator
        {
            // Non-finite entries are always passed to removed indices
            if (!pcl_isfinite (input->points[(*indices_)[iii]].x) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].y) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].z))
            {
                continue;
            }
            indices[oii++] = (*indices_)[iii];
        }
    }
    else
    {
        // Attempt to get the field name's index
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex (*input, filter_field_name, fields);
        if (distance_idx == -1)
        {
            //PCL_WARN ("[pcl::%s::applyFilter] Unable to find field name in point type.\n", getClassName ().c_str ());
            LOG(LWARNING) << "[pcl::?::applyFilter] Unable to find field name in point type."; //TODO
            indices.clear ();
            removed_indices_->clear ();
            return;
        }
        // Filter for non-finite entries and the specified field limits
        for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii) // iii = input indices iterator
        {
            // Non-finite entries are always passed to removed indices
            if (!pcl_isfinite (input->points[(*indices_)[iii]].x) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].y) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].z))
            {
                continue;
            }
            // Get the field's value
            const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input->points[(*indices_)[iii]]);
            float field_value = 0;
            memcpy (&field_value, pt_data + fields[distance_idx].offset, sizeof (float));
            // Remove NAN/INF/-INF values. We expect passthrough to output clean valid data.
            if (!pcl_isfinite (field_value))
            {
                continue;
            }
            // Outside of the field limits are passed to removed indices
            if (!negative && (field_value < min || field_value > max))
            {
                continue;
            }
            // Inside of the field limits are passed to removed indices if negative was set
            if (negative && field_value >= min && field_value <= max)
            {
                continue;
            }
            // Otherwise it was a normal point for output (inlier)
            indices[oii++] = (*indices_)[iii];
        }
    }
    // Resize the output arrays
    indices.resize (oii);
    removed_indices_->resize (rii);
    cout<< "indices_ size "<< indices_->size() <<endl;
    cout<< "removed_indices_ size "<< removed_indices_->size() <<endl;
}

} //: namespace PassThrough
} //: namespace Processors
