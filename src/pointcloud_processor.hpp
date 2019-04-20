#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
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

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/io/pcd_io.h>
#include <chrono>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

class PointcloudProcessor {
   public:
    PointcloudProcessor() {
        // Constructor
        pointcloud_temp = PointCloudPtr(new PointCloud);
        pointcloud_temp2 = PointCloudPtr(new PointCloud);

        pointcloud_floor = PointCloudPtr(new PointCloud);
        pointcloud_floor_blue = PointCloudPtr(new PointCloud);
        pointcloud_floor_yellow = PointCloudPtr(new PointCloud);

        pointcloud_not_floor = PointCloudPtr(new PointCloud);
        pointcloud_not_floor_green = PointCloudPtr(new PointCloud);
        pointcloud_not_floor_blue = PointCloudPtr(new PointCloud);
        pointcloud_not_floor_yellow = PointCloudPtr(new PointCloud);

        //pointcloud_puck_model = PointCloudPtr(new PointCloud);
        //pointcloud_pole_model = PointCloudPtr(new PointCloud);
        //generate_puck_pointcloud(pointcloud_puck_model);
        //generate_pole_pointcloud(pointcloud_pole_model);

        pass_through_filter = pcl::PassThrough<pcl::PointXYZRGB>();
        voxel_grid_filter = pcl::VoxelGrid<pcl::PointXYZRGB>();
        conditional_filter = pcl::ConditionalRemoval<pcl::PointXYZRGB>();
        statistical_outlier_removal_filter =
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>();
    }

    ~PointcloudProcessor() {
        // Destructor
    }

    int get_color_group(uint8_t r, uint8_t g, uint8_t b) {
        // This method determines if a pixel belongs to a green, a yellow or a
        // blue buck or goal.

        // Green pucks
        if (r >= -0.15 * b + 62 && r <= 0.15 * b + 100 &&
            g >= 1.1 * b + 0.3 * r - 15 && g <= 0.48 * b + 147 &&
            b >= 0.4 * r + 25 && b <= -0.2 * r + 210) {
            return 2;
        }

        // Blue pucks
        if (r >= 82 && r <= 240 && r >= -0.9 * b + 280 &&
            g >= 0.9 * b + 0.2 * r - 33 && g <= 0.9 * b + 45 && b >= r + 20 &&
            b <= 0.1 * r + 240) {
            return 3;
        }

        // Yellow pucks
        if (r >= 230 && r <= 255 && r >= 0.35 * b + 180 && g >= 210 &&
            g >= 2 * r - 280 && r <= 2 * r - 230 && b >= 50 && b <= 235) {
            return 1;
        }

        // Reserved values for filtering colors
        if ((r == 255 && g == 255 && b == 0) ||
            (r == 0 && g == 255 && b == 0) || (r == 0 && g == 00 && b == 255)) {
            return 4;
        }
        return 0;  // color not relevant
    }

    void pass_trough_filter_m(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
                              std::string axis, double min, double max) {
        // Pass trough filter
        pass_through_filter.setInputCloud(cloud_in);
        pass_through_filter.setFilterFieldName(axis);
        pass_through_filter.setFilterLimits(min, max);
        pass_through_filter.filter(*cloud_out);
    }

    void voxel_grid_filter_m(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
                             double leaf_size = 0.02, int min_n_points = 0) {
        // Voxel grid filter
        voxel_grid_filter.setInputCloud(cloud_in);
        voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid_filter.setMinimumPointsNumberPerVoxel(min_n_points);
        voxel_grid_filter.filter(*cloud_out);
    }
    /*
    // Filters colors by range
    void color_filter_range(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
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
    }*/

    // Filters colors by a specific color
    void color_filter(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out, int r,
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
        // conditional_filter.setKeepOrganized(true);

        // Apply filter
        conditional_filter.filter(*cloud_out);
    }

    void statistical_outlier_removal_filter_m(PointCloudPtr& cloud_in,
                                              PointCloudPtr& cloud_out) {
        // Not very useful
        statistical_outlier_removal_filter.setInputCloud(cloud_in);
        statistical_outlier_removal_filter.setMeanK(50);
        statistical_outlier_removal_filter.setStddevMulThresh(0.1);
        statistical_outlier_removal_filter.filter(*cloud_out);
    }

    void edit_colors_of_pointcloud(PointCloudPtr& cloud) {
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
                // r = 0;
                // g = 0;
                // b = 0;
            }

            // pack r/g/b into rgb
            rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pt->rgb = *reinterpret_cast<float*>(&rgb);
        };
    }

    void planar_segmentation(PointCloudPtr& cloud_in,
                             PointCloudPtr& cloud_inliers_out,
                             PointCloudPtr& cloud_outliers_out) {
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

        // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
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

    void radius_outlier_removal(PointCloudPtr& cloud_in,
                                PointCloudPtr& cloud_out, double radius = 0.05,
                                int min_neighbors = 5) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        // build the filter
        outrem.setInputCloud(cloud_in);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(min_neighbors);
        // apply filter
        outrem.filter(*cloud_out);
    }

    /*
    void generate_puck_pointcloud(PointCloudPtr& cloud) {
        double radius = 0.05;  // m
        double radius_second_part = 0.0325;
        double radial_step = 0.002;
        double height = 0.18;  // m
        double height_second_part = 0.17;
        double height_step = 0.002;
        cloud->clear();

        // Lower part
        for (double h = 0; h <= height; h += height_step) {
            for (double a = 0; a <= 2 * 3.14 / 3.0; a += radial_step / radius) {
                double x = radius * cos(a);
                double y = radius * sin(a);

                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = h;
                uint32_t rgb =
                    ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
                point.rgb = rgb;
                cloud->points.push_back(point);
            }
        }

        // Upper part
        for (double h = height; h <= height + height_second_part;
             h += height_step) {
            for (double a = 0; a <= 2 * 3.14 / 3.0;
                 a += radial_step / radius_second_part) {
                double x = radius_second_part * cos(a);
                double y = radius_second_part * sin(a);

                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = h;
                uint32_t rgb =
                    ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
                point.rgb = rgb;
                cloud->points.push_back(point);
            }
        }
        cloud->is_dense = false;
        cloud->width = 1;
        cloud->height = cloud->points.size();
    }

    void generate_pole_pointcloud(PointCloudPtr& cloud) {
        double radius = 0.0425;  // m
        double radial_step = 0.002;
        double height = 0.5;  // m
        double height_step = 0.002;
        cloud->clear();

        // Lower part
        for (double h = 0; h <= height; h += height_step) {
            for (double a = 0; a <= 2 * 3.14 / 3.0; a += radial_step / radius) {
                double x = radius * cos(a);
                double y = radius * sin(a);

                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = h;
                uint32_t rgb =
                    ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
                point.rgb = rgb;
                cloud->points.push_back(point);
            }
        }
        cloud->is_dense = false;
        cloud->width = 1;
        cloud->height = cloud->points.size();
    }
    */

    void save_cloud_to_file(PointCloudPtr& cloud, std::string path_and_name) {
        if (cloud->points.size() > 0) {
            pcl::io::savePCDFileASCII(path_and_name, *cloud);
        }
    }

    void edit_z_to(PointCloudPtr& cloud, double z) {
        for (auto pt = cloud->begin(); pt != cloud->end(); ++pt) {
            pt->z = z;
        }
    }

    PointType is_buck_or_pole(PointCloudPtr& cloud) {
        // This method takes as input a pointcloud of a suspected pole or puck.
        // Returns a point containing information (x,y,z,r,g,b) of the object if
        // it fulfills the requirements.

        int color_threshold = 5;  // min number of points in main color
        PointType result_point = PointType(0, 0, 0.2);

        PointType min_point;
        PointType max_point;
        pcl::getMinMax3D(*cloud, min_point, max_point);

        // Calculate metrics of the pointcloud
        double average_x = 0.5 * (max_point.x + min_point.x);
        double average_y = 0.5 * (max_point.y + min_point.y);
        double diagonal_xy = sqrt(pow(max_point.y - min_point.y, 2) +
                                  pow(max_point.x - min_point.x, 2));

        // Check if size of the pointcloud is within the limits
        if (!(max_point.z - min_point.z >= 0.1 &&
              max_point.z - min_point.z <= 0.52 && diagonal_xy <= 0.16 &&
              diagonal_xy >= 0.06)) {
            // std::cout << "too big, diagonal: " << diagonal_xy<< std::endl;
            uint32_t rgb = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
            result_point.rgb = *reinterpret_cast<float*>(&rgb);
            result_point.x = 0;
            result_point.y = 0;

            // size does not match -> return a black point in origo
            return result_point;
        }

        // Calculate values for colors
        uint32_t rgb = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        float yellow = *reinterpret_cast<float*>(&rgb);
        rgb = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        float blue = *reinterpret_cast<float*>(&rgb);
        rgb = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        float green = *reinterpret_cast<float*>(&rgb);
        rgb = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        float cyan = *reinterpret_cast<float*>(&rgb);

        // Calculate color frequencies
        int blue_points = 0;
        int green_points = 0;
        int yellow_points = 0;
        for (auto pt = cloud->begin(); pt < cloud->end(); pt++) {
            if (pt->rgb == yellow) {
                yellow_points++;
            } else if (pt->rgb == green) {
                green_points++;
            } else if (pt->rgb == blue) {
                blue_points++;
            }
        }

        // std::cout << blue_points << " g" << green_points << " y"
        //          << yellow_points << std::endl;

        // Determine to which catecory the object belongs to
        float choosen_color = 0;
        if (blue_points > green_points && blue_points > yellow_points &&
            blue_points > color_threshold) {
            // Blue
            choosen_color = blue;

        } else if (green_points > blue_points && green_points > yellow_points &&
                   green_points > color_threshold) {
            // Green
            choosen_color = green;

        } else if (yellow_points > green_points &&
                   yellow_points > blue_points &&
                   yellow_points > color_threshold) {
            // Yellow
            choosen_color = yellow;
        } else {
            // Unknown
            choosen_color = cyan;

            // Put unknown objects to origo
            // average_x = 0;
            // average_y = 0;
        }

        // Return point, color represents point type: green -> buck, blue or
        // yellow -> pole.
        result_point.rgb = choosen_color;
        result_point.x = average_x;
        result_point.y = average_y;

        return result_point;
    }

    PointCloudPtr get_bucks_and_poles(PointCloudPtr& cloud) {
        // This algorithm uses Euclidean Cluster Extraction to segment the cloud
        // into regions. After segmentation objects are filtered by size (min
        // and max xyz coordinates). Finally, the objects are detectect by the
        // highlighted colors.

        PointCloudPtr cloud_f(new PointCloud);
        PointCloudPtr result(new PointCloud);

        /*
        // Create the segmentation object for the planar model and set all the
        // parameters
        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        PointCloudPtr cloud_plane(new PointCloud());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02);

        int i = 0, nr_points = (int)cloud->points.size();
        while (true) {
            break;  //!!!!!!!!!!!!!!!!
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                std::cout << "Could not estimate a planar model for the given "
                             "dataset."
                          << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<PointType> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);

            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            std::cout << "PointCloud representing the planar component: "
                      << cloud_plane->points.size() << " data points."
                      << std::endl;

            if (cloud_plane->points.size() < 2000) {
                break;  // stop if found planes are too small
            }

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud = *cloud_f;
        }*/

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointType>::Ptr tree(
            new pcl::search::KdTree<PointType>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(0.03);  // 2cm
        ec.setMinClusterSize(16);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 1;
        for (std::vector<pcl::PointIndices>::const_iterator it =
                 cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            pcl::PointCloud<PointType>::Ptr cloud_cluster(
                new pcl::PointCloud<PointType>);

            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // std::cout << "PointCloud representing the Cluster: "
            //          << cloud_cluster->points.size() << " data points."
            //          << std::endl;

            result->points.push_back(is_buck_or_pole(cloud_cluster));
            /*
            int r = j & 0b1;
            int g = j & 0b10;
            int b = j & 0b100;
            int32_t rgb =
                (static_cast<uint32_t>(r) << 16 |
                 static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            for (auto& p : cloud_cluster->points) p.rgb = rgb;

            *result += *cloud_cluster;
            */

            j++;
        }

        // Set header information
        result->is_dense = false;
        result->width = 1;
        result->height = result->points.size();

        return result;
    }

    void process_pointcloud(PointCloudPtr& cloud) {
        // Processes a new point cloud and extracts useful information

        auto start_time = std::chrono::high_resolution_clock::now();

        // Reduce nuber of points in the pointcloud
        voxel_grid_filter_m(cloud, pointcloud_temp2, 0.015, 5);

        // Find interesting colors in the cloud (maps specific color regions to
        // a specific color value for filtering the points later)
        edit_colors_of_pointcloud(pointcloud_temp2);

        // Segment cloud into floor and not floor
        planar_segmentation(pointcloud_temp2, pointcloud_floor,
                            pointcloud_not_floor);

        // save_cloud_to_file(pointcloud_not_floor,
        //"/home/cnc/Desktop/Hockey/PCL-cluster-segmentation-tests/not-floor2.pcd"
        //);

        // Remove outliers
        radius_outlier_removal(pointcloud_not_floor, pointcloud_temp2, 0.02, 3);

        // Find pucks and poles from not floor pointcoud
        recognized_objects = get_bucks_and_poles(pointcloud_temp2);

        // Calculate used time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto delta_time = end_time - start_time;
        std::cout << "Took " << delta_time / std::chrono::milliseconds(1)
                  << "ms to run.\n";

        return;  //----------------------------------------------------
        // Testing stuff:

        // Filter region by color:
        // Blue goals
        color_filter(pointcloud_floor, pointcloud_floor_blue, 0, 0, 255);
        // Yellow goals
        color_filter(pointcloud_floor, pointcloud_floor_yellow, 255, 255, 0);

        // Remove outliers from filtered pointclouds
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pointcloud_not_floor_blue,
                                     *pointcloud_temp, indices);
        if (pointcloud_not_floor_blue->points.size() > 0) {
            radius_outlier_removal(pointcloud_temp, pointcloud_not_floor_blue);
        }


        edit_z_to(pointcloud_floor, 0);
        edit_z_to(pointcloud_not_floor_green, 0.2);
        edit_z_to(pointcloud_not_floor_yellow, 0.2);

        pointcloud_temp->points.clear();
        *pointcloud_temp += *pointcloud_not_floor_blue;
        *pointcloud_temp += *pointcloud_not_floor_green;
        *pointcloud_temp += *pointcloud_not_floor_yellow;

        // Find goals from floor pointclouds
        // TODO

        // Return found objects to pointcloud_node
        // TODO
    }

    PointCloudPtr& get_floor_pointcloud() { return pointcloud_floor; }
    PointCloudPtr& get_not_floor_pointcloud() { return pointcloud_not_floor; }
    PointCloudPtr& get_recognized_objects() { return recognized_objects;}

   private:
    pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> conditional_filter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>
        statistical_outlier_removal_filter;

    PointCloudPtr pointcloud_temp;   // temporary
    PointCloudPtr pointcloud_temp2;  // temporary

    PointCloudPtr pointcloud_floor;
    PointCloudPtr pointcloud_floor_blue;
    PointCloudPtr pointcloud_floor_yellow;

    PointCloudPtr pointcloud_not_floor;
    PointCloudPtr pointcloud_not_floor_green;
    PointCloudPtr pointcloud_not_floor_blue;
    PointCloudPtr pointcloud_not_floor_yellow;

    PointCloudPtr recognized_objects;

    //PointCloudPtr pointcloud_puck_model;
    //PointCloudPtr pointcloud_pole_model;
};