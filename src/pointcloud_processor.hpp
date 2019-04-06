#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPrt;

class PointcloudProcessor {
   public:
    PointcloudProcessor() {
        // Constructor
        pointcloud_temp = PointCloudPrt(new PointCloud);
        pointcloud_temp2 = PointCloudPrt(new PointCloud);

        pointcloud_floor = PointCloudPrt(new PointCloud);
        pointcloud_floor_blue = PointCloudPrt(new PointCloud);
        pointcloud_floor_yellow = PointCloudPrt(new PointCloud);

        pointcloud_not_floor = PointCloudPrt(new PointCloud);
        pointcloud_not_floor_green = PointCloudPrt(new PointCloud);
        pointcloud_not_floor_blue = PointCloudPrt(new PointCloud);
        pointcloud_not_floor_yellow = PointCloudPrt(new PointCloud);

        pass_through_filter = pcl::PassThrough<pcl::PointXYZRGB>();
        voxel_grid_filter = pcl::VoxelGrid<pcl::PointXYZRGB>();
        conditional_filter = pcl::ConditionalRemoval<pcl::PointXYZRGB>();
        statistical_outlier_removal_filter =
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>();

        // initialize_colorfilter_lookup_table();
    }

    ~PointcloudProcessor() {
        // Destructor
    }
    /*
        void initialize_colorfilter_lookup_table() {
            // colorfilter_lookup_table
            for (int r = 0; r < 256; r++) {
                for (int g = 0; g < 256; g++) {
                    for (int b = 0; b < 256; b++) {
                        // yellow
                        if (r >= 175 && r <= 255 && g >= 175 && r <= 255 &&
                            b >= 49 && b <= 241) {
                            colorfilter_lookup_table[r][g][b] = 1;
                            continue;
                        }

                        // green
                        if (r >= 36 && r <= 151 && g >= 10 && r <= 30 && b >= 48
       && b <= 196) { colorfilter_lookup_table[r][g][b] = 2; continue;
                        }

                        // blue
                        if (r >= 82 && r <= 242 && r >= -g + 250 && g >= r - 10
       && r <= 0.1 * r + 240 && b >= 0.9 * g - 20 && b <= 0.9 * g + 50) {
                            colorfilter_lookup_table[r][g][b] = 3;
                            continue;
                        }

                        // remove reserved values
                        if ((r == 1 && g == 1 && b == 1) ||
                            (r == 2 && g == 2 && b == 2) ||
                            (r == 3 && g == 3 && b == 3)) {
                            colorfilter_lookup_table[r][g][b] = 0;
                            continue;
                        }
                    }
                }
            }
        }*/

    int get_color_group(uint8_t r, uint8_t g, uint8_t b) {
        // This method determines if a pixel belongs to a green, a yellow or a
        // blue buck or goal green pucks
        if (r >= -0.15 * b + 62 && r <= 0.15 * b + 100 &&
            g >= 1.1 * b + 0.3 * r - 15 && g <= 0.48 * b + 147 &&
            b >= 0.4 * r + 25 && b <= -0.2 * r + 210) {
            return 2;
        }

        // blue pucks
        if (r >= 82 && r <= 240 && r >= -0.9 * b + 280 &&
            g >= 0.9 * b + 0.2 * r - 33 && g <= 0.9 * b + 45 && b >= r + 20 &&
            b <= 0.1 * r + 240) {
            return 3;
        }

        // yellow pucks
        if (r >= 230 && r <= 255 && r >= 0.35 * b + 180 && g >= 210 &&
            g >= 2 * r - 280 && r <= 2 * r - 230 && b >= 50 && b <= 235) {
            return 1;
        }

        // reserved value for filtering pucks
        if ((r == 255 && g == 255 && b == 0) ||
            (r == 0 && g == 255 && b == 0) || (r == 0 && g == 00 && b == 255)) {
            return 4;
        }
        return 0;
    }

    void pass_trough_filter_m(PointCloudPrt& cloud_in, PointCloudPrt& cloud_out,
                              std::string axis, double min, double max) {
        // Pass trough filter
        pass_through_filter.setInputCloud(cloud_in);
        pass_through_filter.setFilterFieldName(axis);
        pass_through_filter.setFilterLimits(min, max);
        pass_through_filter.filter(*cloud_out);
    }

    void voxel_grid_filter_m(PointCloudPrt& cloud_in, PointCloudPrt& cloud_out,
                             double leaf_size = 0.02) {
        // Voxel grid filter
        voxel_grid_filter.setInputCloud(cloud_in);
        voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid_filter.filter(*cloud_out);
    }

    // Filters colors by range
    void color_filter_range(PointCloudPrt& cloud_in, PointCloudPrt& cloud_out,
                            int min_r, int max_r, int min_g, int max_g,
                            int min_b, int max_b) {
        // Build the conditions
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_condition(
            new pcl::ConditionAnd<pcl::PointXYZRGB>());

        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "r", pcl::ComparisonOps::GT, min_r)));
        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "r", pcl::ComparisonOps::LT, max_r)));

        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "g", pcl::ComparisonOps::GT, min_g)));
        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "g", pcl::ComparisonOps::LT, max_g)));

        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "b", pcl::ComparisonOps::GT, min_b)));
        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "b", pcl::ComparisonOps::LT, max_b)));

        // Build the filter
        conditional_filter.setCondition(color_condition);
        conditional_filter.setInputCloud(cloud_in);
        conditional_filter.setKeepOrganized(true);

        // Apply filter
        conditional_filter.filter(*cloud_out);
    }

    // Filters colors by a specific color
    void color_filter(PointCloudPrt& cloud_in, PointCloudPrt& cloud_out, int r,
                      int g, int b) {
        // Build the condition
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_condition(
            new pcl::ConditionAnd<pcl::PointXYZRGB>());

        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "r", pcl::ComparisonOps::EQ, r)));

        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "g", pcl::ComparisonOps::EQ, g)));

        color_condition->addComparison(
            pcl::PackedRGBComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::PackedRGBComparison<pcl::PointXYZRGB>(
                    "b", pcl::ComparisonOps::EQ, b)));

        // Build the filter
        conditional_filter.setCondition(color_condition);
        conditional_filter.setInputCloud(cloud_in);
        conditional_filter.setKeepOrganized(true);

        // Apply filter
        conditional_filter.filter(*cloud_out);
    }

    void statistical_outlier_removal_filter_m(PointCloudPrt& cloud_in,
                                              PointCloudPrt& cloud_out) {
        statistical_outlier_removal_filter.setInputCloud(cloud_in);
        statistical_outlier_removal_filter.setMeanK(50);
        statistical_outlier_removal_filter.setStddevMulThresh(0.1);
        statistical_outlier_removal_filter.filter(*cloud_out);
    }

    void edit_colors_of_pointcloud(PointCloudPrt& cloud) {
        for (auto pt = cloud->begin(); pt != cloud->end(); ++pt) {
            uint32_t rgb = *reinterpret_cast<int*>(&pt->rgb);
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8) & 0x0000ff;
            uint8_t b = (rgb)&0x0000ff;

            uint8_t result = get_color_group(r, g, b);

            if (result == 1) {
                // yellow puck
                r = 255;
                g = 255;
                b = 0;
            } else if (result == 2) {
                // green puck
                r = 0;
                g = 255;
                b = 0;
            } else if (result == 3) {
                // blue puck
                r = 0;
                g = 0;
                b = 255;
            } else if (result == 4) {
                // reserved value
                r = 0;
                g = 0;
                b = 0;
            } else {
                // test
                r = 0;
                g = 0;
                b = 0;
            }

            // pack r/g/b into rgb
            rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pt->rgb = *reinterpret_cast<float*>(&rgb);
        };
    }

    void planar_segmentation(PointCloudPrt& cloud_in,
                             PointCloudPrt& cloud_inliers_out,
                             PointCloudPrt& cloud_outliers_out) {
        // Used to find planar surfaces from a point cloud

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02);

        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cerr
                << "Could not estimate a planar model for the given dataset."
                << std::endl;
            return;
        }

        //std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        //          << coefficients->values[1] << " " << coefficients->values[2]
        //          << " " << coefficients->values[3] << std::endl;

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);

        // Extract the inliers
        extract.setNegative(false);
        extract.filter(*cloud_inliers_out);

        // Extract the outliers
        extract.setNegative(true);
        extract.filter(*cloud_outliers_out);
    }

    void process_pointcloud(PointCloudPrt& cloud) {
        // Processes a new point cloud and extracts useful information
        voxel_grid_filter_m(cloud, pointcloud_temp);

        planar_segmentation(pointcloud_temp, pointcloud_floor,
                            pointcloud_not_floor);
        /*
                pass_trough_filter_m(pointcloud_temp, pointcloud_temp2, "z",
           -0.2, 0.05); pass_trough_filter_m(pointcloud_temp, pointcloud_temp2,
           "z", 0.05, 1);

                edit_colors_of_pointcloud(pointcloud_temp);
                edit_colors_of_pointcloud(pointcloud_temp2);

                // Blue goals
                color_filter(pointcloud_temp, pointcloud_floor_blue, 0, 0, 255);
                // Yellow goals
                color_filter(pointcloud_temp, pointcloud_floor_yellow, 255, 255,
           0);

                // Blue bucks
                color_filter(pointcloud_temp2, pointcloud_not_floor_blue, 0, 0,
           255);
                // Green bucks
                color_filter(pointcloud_temp2, pointcloud_not_floor_green, 0,
           255, 0);
                // Yellow bucks
                color_filter(pointcloud_temp2, pointcloud_not_floor_yellow, 255,
           255, 0);

                // std::cerr << "Cloud before filtering: " << std::endl;
                // std::cerr << *pointcloud_not_floor_green << std::endl;
                statistical_outlier_removal_filter_m(pointcloud_not_floor_green,
                                                     pointcloud_temp);
                // std::cerr << "Cloud after filtering: " << std::endl;
                // std::cerr << *pointcloud_temp << std::endl;*/
    }

    PointCloudPrt& get_floor_pointcloud() { return pointcloud_floor_blue; }
    PointCloudPrt& get_not_floor_pointcloud() { return pointcloud_not_floor; }

   private:
    pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> conditional_filter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>
        statistical_outlier_removal_filter;

    PointCloudPrt pointcloud_temp;   // temporary
    PointCloudPrt pointcloud_temp2;  // temporary

    PointCloudPrt pointcloud_floor;
    PointCloudPrt pointcloud_floor_blue;
    PointCloudPrt pointcloud_floor_yellow;

    PointCloudPrt pointcloud_not_floor;
    PointCloudPrt pointcloud_not_floor_green;
    PointCloudPrt pointcloud_not_floor_blue;
    PointCloudPrt pointcloud_not_floor_yellow;

    // uint8_t colorfilter_lookup_table[125][125][125];
};