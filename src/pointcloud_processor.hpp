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
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <chrono>
#include "opencv2/opencv.hpp"

#include "pointcloud_helpers.hpp"

const std::string IMAGE_WINDOW = "Floor image";
const std::string IMAGE_WINDOW2 = "Opencv edge image";

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

        pass_through_filter = pcl::PassThrough<pcl::PointXYZRGB>();
        voxel_grid_filter = pcl::VoxelGrid<pcl::PointXYZRGB>();
        conditional_filter = pcl::ConditionalRemoval<pcl::PointXYZRGB>();
        statistical_outlier_removal_filter =
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>();

        // cv::namedWindow(IMAGE_WINDOW);
        // cv::namedWindow(IMAGE_WINDOW2);
        // cv::moveWindow(IMAGE_WINDOW2, 200, 0);
    }

    ~PointcloudProcessor() {
        // Destructor
    }

    int get_color_group(uint8_t r, uint8_t g, uint8_t b) {
        // This method determines if a pixel belongs to a green, a yellow or a
        // blue buck or goal. The 3D regions in RGB space are determined
        // manually from sample images using a python script for visualising.

        // Green poles
        if (r >= -0.15 * b + 62 && r <= 0.15 * b + 100 &&
            g >= 1.1 * b + 0.3 * r - 15 && g <= 0.48 * b + 147 &&
            b >= 0.4 * r + 25 && b <= -0.2 * r + 210) {
            return 2;
        }

        // Blue pucks and goals
        if (r >= 82 && r <= 240 && r >= -2 * b + 450 &&
            g >= 0.9 * b + 0.2 * r - 33 && g <= 0.9 * b + 45 && b >= r + 48 &&
            b <= 0.1 * r + 240) {
            return 3;
        }

        // Yellow pucks and goals
        if (r >= 230 && r <= 255 && r >= 0.35 * b + 180 && g >= 210 &&
            g >= 2 * r - 280 && r <= 2 * r - 230 && b >= 50 && b <= 235) {
            return 1;
        }

        // Reserved values for filtering colors (highlighting)
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

    // Filters pointcloud by a specific color
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
        // This function highlight intereseting colors of the pointcloud. For
        // example, all yellow colors that belong to yellow pucks and goals are
        // mapper to a RGB value of (255,255,0).

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

            pt->rgb = to_pcl_rgb(r, g, b);
        };
    }

    bool planar_segmentation(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
                             pcl::ModelCoefficients::Ptr& coefficients,
                             bool get_inliers) {
        // Used to find planar surfaces from a point cloud. Useful for removing
        // floor from pointcloud. Returns true if succesfull.

        if (cloud_in->points.size() == 0) {
            return false;
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<PointType> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02);

        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cout
                << "Could not estimate a planar model for the given dataset."
                << std::endl;
            return false;
        }

        // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
        //          << coefficients->values[1] << " " << coefficients->values[2]
        //          << " " << coefficients->values[3] << std::endl;

        // Create the filtering object
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);

        // Extract the inliers
        extract.setNegative(!get_inliers);
        extract.filter(*cloud_out);

        // Extract the outliers
        // extract.setNegative(true);
        // extract.filter(*cloud_outliers_out);
        return true;
    }

    void plane_filter(PointCloudPtr& cloud_in, PointCloudPtr& cloud_out,
                      pcl::ModelCoefficients::Ptr& coefficients,
                      double z_offset) {
        // This function filters away all points below the plane given by the
        // coefficients. Roughly same speed as planar segmentation.

        // Coefficients
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3] + z_offset;

        // std::cout << a << " " << b << " " << c << " " << d << " " <<
        // std::endl;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (int i = 0; i < (*cloud_in).size(); i++) {
            double x = cloud_in->points[i].x;
            double y = cloud_in->points[i].y;
            double z = cloud_in->points[i].z;

            double res = a * x + b * y + c * z + d;
            if (res >= 0) {
                // Keep these points
                inliers->indices.push_back(i);
            }
        }

        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_out);
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

    void edit_z_to(PointCloudPtr& cloud, double z) {
        // Set z value of all points to a specific value.
        for (auto pt = cloud->begin(); pt != cloud->end(); ++pt) {
            pt->z = z;
        }
    }

    static PointType is_buck_or_pole(PointCloudPtr& cloud) {
        // This method takes as input a pointcloud of a suspected pole or puck.
        // Returns a point containing information (x,y,z,r,g,b) of the object if
        // it fulfills the requirements. If the cloud has wrong size a black
        // point in the origo is returnet. If only the color requirements are
        // not fulfilled a magenta point is returned at the position of the
        // object.

        // Detection parameters
        const int color_threshold =
            5;  // min number of points in main color // 5 works
        const double min_z_height = 0.1;
        const double max_z_height = 0.52;
        const double min_diagonal_size = 0.06;
        const double max_diagonal_size = 0.16;

        PointType result_point = PointType(0, 0, 0.2);

        // Calculate metrics of the pointcloud
        PointType min_point;
        PointType max_point;
        PointType centroid;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        pcl::computeCentroid(*cloud, centroid);

        double average_x = centroid.x;
        double average_y = centroid.y;
        double diagonal_xy = sqrt(pow(max_point.y - min_point.y, 2) +
                                  pow(max_point.x - min_point.x, 2));

        // Check if size of the pointcloud is within the limits
        if (!(max_point.z - min_point.z >= min_z_height &&
              max_point.z - min_point.z <= max_z_height &&
              diagonal_xy <= max_diagonal_size &&
              diagonal_xy >= min_diagonal_size)) {
            result_point.rgb = to_pcl_rgb(0, 0, 0);
            result_point.x = 0;
            result_point.y = 0;

            // size does not match -> return a black point in origo
            return result_point;
        }

        // Calculate values for colors
        float yellow = to_pcl_rgb(255, 255, 0);
        float blue = to_pcl_rgb(0, 0, 255);
        float green = to_pcl_rgb(0, 255, 0);
        float magenta = to_pcl_rgb(255, 0, 255);

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
            // Unknown type
            choosen_color = magenta;
        }

        // Return point, color represents point type: green -> buck, blue or
        // yellow -> pole, magenta -> unknown.
        result_point.rgb = choosen_color;
        result_point.x = average_x;
        result_point.y = average_y;

        return result_point;
    }

    static PointType is_goal_corner(PointCloudPtr& cloud) {
        // This method takes as input a pointcloud of a suspected goal corner.
        // Returns a point containing information (x,y,z,r,g,b) of the object if
        // it fulfills the requirements.

        PointType result_point = PointType(0, 0, 0.2);

        // Calculate metrics of the pointcloud
        PointType min_point;
        PointType max_point;
        PointType centroid;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        pcl::computeCentroid(*cloud, centroid);

        double average_x = centroid.x;
        double average_y = centroid.y;
        double diagonal_xy = sqrt(pow(max_point.y - min_point.y, 2) +
                                  pow(max_point.x - min_point.x, 2));

        // Check if size of the pointcloud is within the limits
        if (!(diagonal_xy <= 0.3)) {
            result_point.rgb = to_pcl_rgb(0, 0, 0);
            result_point.x = 0;
            result_point.y = 0;

            // size does not match -> return a black point in origo
            return result_point;
        }

        // Returns same color as the orginal cloud
        result_point.rgb = cloud->points[0].rgb;
        result_point.x = average_x;
        result_point.y = average_y;

        return result_point;
    }

    /*PointCloudPtr get_bucks_and_poles(PointCloudPtr& cloud) {
        // This algorithm uses Euclidean Cluster Extraction to segment the cloud
        // into regions. After segmentation objects are filtered by size (min
        // and max xyz coordinates). Finally, the objects are detectect by the
        // highlighted colors.

        PointCloudPtr result(new PointCloud);

        if (cloud->points.size() == 0) {
            return result;
        }

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

            PointType point = is_buck_or_pole(cloud_cluster);
            if (*reinterpret_cast<int*>(&point.rgb) != 0) {
                // Add only succesfull detections (== not black points)
                result->points.push_back(point);
            }
        }

        // Set header information
        result->is_dense = false;
        result->width = 1;
        result->height = result->points.size();

        return result;
    }*/

    void extract_edge_points(PointCloudPtr& cloud,
                             PointCloudPtr& cloud_blue_out,
                             PointCloudPtr& cloud_yellow_out) {
        // This function extracts edge points of the goals using pcl. Quite
        // slow.

        const float COLOR_DIFF_MAX =
            0.10;  // 1/(100%) maximun distance from 50 % //0.15
        const int NEIGHBORHS_MIN = 2;  // 2
        float radius = 0.04;           // 0.035

        if (cloud->points.size() == 0) {
            return;
        }

        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(cloud);
        PointType searchPoint;

        // Neighbors within radius search - edge extraction
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        pcl::PointIndices::Ptr inliers_blue(new pcl::PointIndices());
        pcl::PointIndices::Ptr inliers_yellow(new pcl::PointIndices());
        pcl::ExtractIndices<PointType> extract;
        for (int i = 0; i < (*cloud).size(); i += 1) {
            searchPoint.x = cloud->points[i].x;
            searchPoint.y = cloud->points[i].y;
            searchPoint.z = 0;

            int number_of_blue_color = 0;
            int number_of_yellow_color = 0;
            int nuber_of_not_color = 0;

            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance) > 0) {
                for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                    uint32_t rgb = *reinterpret_cast<int*>(
                        &cloud->points[pointIdxRadiusSearch[i]].rgb);
                    uint8_t rp = (rgb >> 16) & 0x0000ff;
                    uint8_t gp = (rgb >> 8) & 0x0000ff;
                    uint8_t bp = (rgb)&0x0000ff;

                    if (0 == rp && 0 == gp && 255 == bp) {
                        number_of_blue_color++;
                    } else if (255 == rp && 255 == gp && 0 == bp) {
                        number_of_yellow_color++;
                    } else {
                        nuber_of_not_color++;
                    }
                }
            }

            int sum_of_points = (number_of_blue_color + number_of_yellow_color +
                                 nuber_of_not_color);

            float ratio_blue = (float)number_of_blue_color / sum_of_points;
            float ratio_yellow = (float)number_of_yellow_color / sum_of_points;

            if (ratio_blue > 0.5 - COLOR_DIFF_MAX &&
                ratio_blue < 0.5 + COLOR_DIFF_MAX &&
                sum_of_points > NEIGHBORHS_MIN) {
                // These points are on an edge and belog to a blue goal
                inliers_blue->indices.push_back(i);
            } else if (ratio_yellow > 0.5 - COLOR_DIFF_MAX &&
                       ratio_yellow < 0.5 + COLOR_DIFF_MAX &&
                       sum_of_points > NEIGHBORHS_MIN) {
                // These points are on an edge and belong to a yellow goal
                inliers_yellow->indices.push_back(i);
            }
        }

        extract.setInputCloud(cloud);

        extract.setIndices(inliers_blue);
        extract.filter(*cloud_blue_out);

        extract.setIndices(inliers_yellow);
        extract.filter(*cloud_yellow_out);
    }

    void extract_edge_points_opencv(PointCloudPtr& cloud, double& x_offset,
                                    double& y_offset, double pixel_size,
                                    int marginal,
                                    cv::Mat& floor_image_extracted_blue,
                                    cv::Mat& floor_image_extracted_yellow) {
        // This function extracts goal edges to corresponding images. A bit
        // faster than the pcl version.

        // auto start_time = std::chrono::high_resolution_clock::now();

        if (cloud->points.size() == 0) {
            return;
        }

        // Parameters for filtering
        int search_radius = 2;  // pix //3 works also but slower
        int kernel_size = 2 * search_radius + 1;  // pix
        int sum_of_points = kernel_size * kernel_size;
        const float COLOR_DIFF_MAX =
            0.13;  // 1/(100%) maximun distance from 50 % //0.15
        const int MAX_BLACK_PIX = 16;  // = 0.6 * sum_of_points // 30 works also

        // Parameters for pointcloud -> 2D image
        PointType max_point;
        PointType min_point;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        x_offset = min_point.x;  // origo of image is here
        y_offset = min_point.y;
        double h = fabs(max_point.y - min_point.y);
        double w = fabs(max_point.x - min_point.x);

        if (h <= 0 || w <= 0) {
            return;
        }

        // Image resolution
        int h_pix = (int)round(h / pixel_size) + 2 * marginal;  // pix
        int w_pix = (int)round(w / pixel_size) + 2 * marginal;  // pix

        // Create image from pointcloud
        cv::Mat floor_image = cv::Mat::zeros(h_pix, w_pix, CV_8UC3);
        floor_image_extracted_blue = cv::Mat::zeros(h_pix, w_pix, CV_8UC1);
        floor_image_extracted_yellow = cv::Mat::zeros(h_pix, w_pix, CV_8UC1);

        for (int i = 0; i < (*cloud).size(); i++) {
            int x = (int)round((cloud->points[i].x - x_offset) / pixel_size) +
                    marginal;
            int y = (int)round((cloud->points[i].y - y_offset) / pixel_size) +
                    marginal;

            if (x < w_pix && y < h_pix && x >= 0 && y >= 0) {
                uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
                uint8_t r = (rgb >> 16) & 0x0000ff;
                uint8_t g = (rgb >> 8) & 0x0000ff;
                uint8_t b = (rgb)&0x0000ff;
                floor_image.at<cv::Vec3b>(y, x, 0) = cv::Vec3b(b, g, r);
            }
        }
        // imwrite("/home/cnc/Desktop/ResultImage.png", floor_image);

        // Extract edges
        for (int x = 0; x < floor_image.cols - kernel_size; x++) {
            for (int y = 0; y < floor_image.rows - kernel_size; y++) {
                int number_of_blue_color = 0;
                int number_of_yellow_color = 0;
                int number_of_black = 0;

                // Calculate frequencies of colors
                for (int i = x; i < x + kernel_size; i++) {
                    for (int j = y; j < y + kernel_size; j++) {
                        int rp = floor_image.at<cv::Vec3b>(j, i)[2];
                        int gp = floor_image.at<cv::Vec3b>(j, i)[1];
                        int bp = floor_image.at<cv::Vec3b>(j, i)[0];

                        if (0 == rp && 0 == gp && 255 == bp) {
                            number_of_blue_color++;
                        } else if (255 == rp && 255 == gp && 0 == bp) {
                            number_of_yellow_color++;
                        } else if (0 == rp && 0 == gp && 0 == bp) {
                            number_of_black++;
                        }
                    }
                }

                float sum_of_valid_points = sum_of_points - number_of_black;
                float ratio_blue =
                    (float)number_of_blue_color / sum_of_valid_points;
                float ratio_yellow =
                    (float)number_of_yellow_color / sum_of_valid_points;

                if (ratio_blue > 0.5 - COLOR_DIFF_MAX &&
                    ratio_blue < 0.5 + COLOR_DIFF_MAX &&
                    number_of_black < MAX_BLACK_PIX) {
                    // These points are on an edge and belog to a blue goal
                    floor_image_extracted_blue.at<uint8_t>(
                        y + search_radius, x + search_radius, 0) = 255;

                } else if (ratio_yellow > 0.5 - COLOR_DIFF_MAX &&
                           ratio_yellow < 0.5 + COLOR_DIFF_MAX &&
                           number_of_black < MAX_BLACK_PIX) {
                    // These points are on an edge and belong to a yellow goal
                    floor_image_extracted_yellow.at<uint8_t>(
                        y + search_radius, x + search_radius, 0) = 255;
                }
            }
        }
        // Calculate used time
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto delta_time = end_time - start_time;
        // std::cout << "Took opencv " << delta_time /
        // std::chrono::milliseconds(1)
        //          << "ms to run.\n";
        // imwrite("/home/cnc/Desktop/edgesblue.png",
        // floor_image_extracted_blue);
        // imwrite("/home/cnc/Desktop/edgesyellow.png",
        //        floor_image_extracted_yellow);

        /*
        // Visualisation for debugging
        cv::Mat combined;
        cv::Mat combined2;
        cv::Mat rgb_blue;
        cvtColor(floor_image_extracted_blue, rgb_blue, cv::COLOR_GRAY2RGB);
        cv::Mat rgb_yellow;
        cvtColor(floor_image_extracted_yellow, rgb_yellow, cv::COLOR_GRAY2RGB);

        cv::hconcat(floor_image, rgb_blue, combined);
        cv::hconcat(combined, rgb_yellow, combined2);
        cv::imshow(IMAGE_WINDOW2, combined2);
        cv::waitKey(1);
        */
    }

    PointCloudPtr get_goal_corners_opencv(cv::Mat& edge_image, double x_offset,
                                          double y_offset, double pixel_size,
                                          int marginal, int r, int g, int b) {
        // Calculate corners of edge_image, returns cloud containing the
        // corners marked with a point (its color is based on the input rgb
        // values).

        pcl::PointCloud<PointType>::Ptr cloud_corners(
            new pcl::PointCloud<PointType>);

        if (edge_image.rows < 5) {
            // Not enough data
            return cloud_corners;
        }

        /*
        // For debugging
        cv::Mat floor_result =
            cv::Mat::zeros(edge_image.rows, edge_image.cols, CV_8UC1);
        */

        // Parameters of the hough transform
        int line_detection_thresh =
            25;  // min number of intersecting points //25 works
        int resolution_of_r = 1;                         // pix
        double resolution_of_angle = CV_PI / 180 * 0.5;  // rad
        double max_deviation_from_90_deg =
            1 * CV_PI / 180;  // rad, for 90 deg corner detection // 1 deg works

        // Apply Hough Transform with opencv
        std::vector<cv::Vec2f> lines;  // will hold the results of the detection
        cv::HoughLines(edge_image, lines, resolution_of_r, resolution_of_angle,
                       line_detection_thresh);

        /*
        // Draw lines on the image for visualisation and debugging
        for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(floor_result, pt1, pt2, cv::Scalar(255, 0, 0), 1, CV_AA);
        }  */

        // Calculate corner points
        for (size_t i = 0; i < lines.size(); i++) {
            for (size_t j = i; j < lines.size(); j++) {
                float p1_rho = lines[i][0];
                float p1_theta = lines[i][1];

                float p2_rho = lines[j][0];
                float p2_theta = lines[j][1];

                float angle = fabs(p1_theta - p2_theta);

                if (angle < CV_PI / 2 + max_deviation_from_90_deg &&
                    angle > CV_PI / 2 - max_deviation_from_90_deg) {
                    // Two lines are roughly at 90 deg angle
                    // Calculate the coordinates of the intersecting point
                    double x;
                    double y;
                    /*
                    double d2 = (cos(p1_theta) * cos(p2_theta) * p2_rho
                                + sin(p1_theta) * sin(p2_theta) * p2_rho -
                    p1_rho/ (cos(p1_theta) * sin(p2_theta) - sin(p1_theta) *
                    cos(p2_theta)));

                    x = p2_rho * cos(p2_theta) + d2 * cos(p2_theta + CV_PI/2);
                    y = p2_rho * sin(p2_theta) + d2 * sin(p2_theta + CV_PI/2);
                    */
                    double a1 = cos(p1_theta);
                    double b1 = sin(p1_theta);
                    double a2 = cos(p2_theta);
                    double b2 = sin(p2_theta);
                    double det = a1 * b2 - b1 * a2;
                    if (det != 0 && p1_rho != 0 && p2_rho != 0 && a1 != 0 &&
                        a2 != 0 && b1 != 0 && b2 != 0) {
                        x = (b1 * p2_rho - p1_rho * b2) / (-det);
                        y = -a1 / b1 * x + p1_rho / b1;

                        /*/ Debugging:
                        cv::Point pt1;
                        pt1.x = cvRound(x);
                        pt1.y = cvRound(y);
                        circle(floor_result, pt1, 10, cv::Scalar(255, 0, 0), 1);
                        */

                        // Intersecting point to odom frame transformation
                        PointType point;
                        point.x = x_offset + (x - marginal) * pixel_size;
                        point.y = y_offset + (y - marginal) * pixel_size;
                        point.z = 0;
                        point.rgb = to_pcl_rgb(r, g, b);

                        cloud_corners->points.push_back(point);
                    }
                }
            }
        }
        /*
        // Visualisation
        cv::Mat combined;
        cv::hconcat(edge_image, floor_result, combined);
        cv::imshow(IMAGE_WINDOW2, combined);
        cv::waitKey(1);
        */
        return cloud_corners;
    }

    PointCloudPtr combine_close_points(PointCloudPtr cloud,
                                       PointType (*classify)(PointCloudPtr&),
                                       double cluster_tolerance = 0.1,
                                       int min_cluster_size = 1,
                                       int max_cluster_size = 100) {
        // This algorithm uses Euclidean Cluster Extraction to segment the cloud
        // into regions. After segmentation the regions are saved as one cloud
        // to merge point clusters into points.

        PointCloudPtr result(new PointCloud);

        if (cloud->points.size() == 0) {
            return result;
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointType>::Ptr tree(
            new pcl::search::KdTree<PointType>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

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

            // result->points.push_back(is_goal_corner(cloud_cluster));

            PointType point = classify(cloud_cluster);
            if (*reinterpret_cast<int*>(&point.rgb) != 0) {
                // Add only succesfull detections (== not black points)
                result->points.push_back(point);
            }
        }

        // Set header information
        result->is_dense = false;
        result->width = 1;
        result->height = result->points.size();

        return result;
    }

    PointCloudPtr get_goal_corners(PointCloudPtr& cloud, int r, int g, int b) {
        // Calculate corners of cloud, returns cloud containing the corners
        // marked with a point (its color is based on the input rgb values).
        // PCL approach

        pcl::PointCloud<PointType>::Ptr cloud_corners(
            new pcl::PointCloud<PointType>);

        if (cloud->points.size() < 5) {
            return cloud_corners;
        }
        // Parameters
        PointType max_point;
        PointType min_point;
        pcl::getMinMax3D(*cloud, min_point, max_point);
        double x_offset = min_point.x;  // origo of image is here
        double y_offset = min_point.y;
        double h = fabs(max_point.y - min_point.y);
        double w = fabs(max_point.x - min_point.x);
        double pixel_size = 0.01;  // m // 0.01 works

        if (h <= 0 || w <= 0) {
            return cloud_corners;
        }

        int marginal = 10;                                      // pix
        int h_pix = (int)round(h / pixel_size) + 2 * marginal;  // pix
        int w_pix = (int)round(w / pixel_size) + 2 * marginal;  // pix

        // Create image from pointcloud
        cv::Mat floor_image = cv::Mat::zeros(h_pix, w_pix, CV_8UC1);
        cv::Mat floor_result = cv::Mat::zeros(h_pix, w_pix, CV_8UC1);
        for (int i = 0; i < (*cloud).size(); i++) {
            int x = (int)round((cloud->points[i].x - x_offset) / pixel_size) +
                    marginal;
            int y = (int)round((cloud->points[i].y - y_offset) / pixel_size) +
                    marginal;

            if (x < w_pix && y < h_pix && x >= 0 && y >= 0) {
                floor_image.at<uint8_t>(y, x, 0) = 255;
            }
        }

        // Apply Hough Transform wiht opencv
        int line_detection_thresh =
            30;  // min number of intersecting points //14 works
        int resolution_of_r = 1;                         // pix
        double resolution_of_angle = CV_PI / 180 * 0.5;  // deg

        std::vector<cv::Vec2f> lines;  // will hold the results of the detection
        cv::HoughLines(floor_image, lines, resolution_of_r, resolution_of_angle,
                       line_detection_thresh);

        // Draw lines on the image
        for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0], theta = lines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(floor_result, pt1, pt2, cv::Scalar(255, 0, 0), 1, CV_AA);
        }

        // Calculate corner points
        double max_deviation_from_90_deg = 1 * CV_PI / 180;  // rad
        for (size_t i = 0; i < lines.size(); i++) {
            for (size_t j = i; j < lines.size(); j++) {
                float p1_rho = lines[i][0];
                float p1_theta = lines[i][1];

                float p2_rho = lines[j][0];
                float p2_theta = lines[j][1];

                float angle = fabs(p1_theta - p2_theta);

                if (angle < CV_PI / 2 + max_deviation_from_90_deg &&
                    angle > CV_PI / 2 - max_deviation_from_90_deg) {
                    // Two lines are roughly at 90 deg angle
                    // Calculate the coordinates of the crossing
                    double x;
                    double y;
                    /*
                    double d2 = (cos(p1_theta) * cos(p2_theta) * p2_rho
                                + sin(p1_theta) * sin(p2_theta) * p2_rho -
                    p1_rho/ (cos(p1_theta) * sin(p2_theta) - sin(p1_theta) *
                    cos(p2_theta)));

                    x = p2_rho * cos(p2_theta) + d2 * cos(p2_theta + CV_PI/2);
                    y = p2_rho * sin(p2_theta) + d2 * sin(p2_theta + CV_PI/2);
                    */
                    double a1 = cos(p1_theta);
                    double b1 = sin(p1_theta);
                    double a2 = cos(p2_theta);
                    double b2 = sin(p2_theta);
                    double det = a1 * b2 - b1 * a2;
                    if (det != 0 && p1_rho != 0 && p2_rho != 0 && a1 != 0 &&
                        a2 != 0 && b1 != 0 && b2 != 0) {
                        x = (b1 * p2_rho - p1_rho * b2) / (-det);
                        y = -a1 / b1 * x + p1_rho / b1;

                        // Debugging:
                        cv::Point pt1;
                        pt1.x = cvRound(x);
                        pt1.y = cvRound(y);
                        circle(floor_result, pt1, 10, cv::Scalar(255, 0, 0), 1);

                        PointType point;
                        point.x = x_offset + (x - marginal) * pixel_size;
                        point.y = y_offset + (y - marginal) * pixel_size;
                        point.z = 0;
                        point.rgb = to_pcl_rgb(r, g, b);

                        cloud_corners->points.push_back(point);
                    }
                }
            }
        }
        /* // For debugging hough transform, visualisation
        cv::Mat combined;
        cv::hconcat(floor_image, floor_result, combined);
        cv::imshow(IMAGE_WINDOW, combined);
        // cv::imshow(IMAGE_WINDOW2, floor_result);
        cv::waitKey(1);
        */
        return cloud_corners;
    }

    void remove_edge_detections(PointCloudPtr& recognized_objects,
                                tf::Transform transform_odom_to_baselink) {
        // This function removes puck and pole detecions that are close to the
        // edge of the orginal input pointcloud. The idea is to reduce false
        // detections that occur if colors of the puck/pole are projected on the
        // wall.

        // Parameters
        const double KINECT_FIELD_OF_VIEW_HORIZONTAL_ON_FLOOR =
            3.1415 / 180 * 51 / 2;              // 2*rad
        const double FILTER_ZONE_WIDTH = 0.12;  // m

        // Transform pointcloud to base_link tf frame
        pcl_ros::transformPointCloud(*recognized_objects, *pointcloud_temp2,
                                     transform_odom_to_baselink);

        pcl::PointIndices::Ptr to_be_removed(new pcl::PointIndices());
        pcl::ExtractIndices<PointType> extract;

        // Check all points if they are in the edge zone and if they need to be
        // removed
        for (int i = 0; i < (*pointcloud_temp2).size(); i++) {
            double x = pointcloud_temp2->points[i].x;
            double y = pointcloud_temp2->points[i].y;
            double r = sqrt(x * x + y * y);
            double beta = atan2(y, x);

            double distance_to_left_zone =
                r * sin(KINECT_FIELD_OF_VIEW_HORIZONTAL_ON_FLOOR - beta);
            double distance_to_right_zone =
                r * sin(KINECT_FIELD_OF_VIEW_HORIZONTAL_ON_FLOOR + beta);

            if (distance_to_left_zone < FILTER_ZONE_WIDTH ||
                distance_to_right_zone < FILTER_ZONE_WIDTH) {
                // Delete these points from recognized_objects
                to_be_removed->indices.push_back(i);
            }
        }
        extract.setInputCloud(recognized_objects);
        extract.setIndices(to_be_removed);
        extract.setNegative(true);
        extract.filter(*recognized_objects);
    }

    void process_pointcloud(PointCloudPtr& cloud,
                            tf::Transform transform_odom_to_baselink) {
        // Processes a new point cloud and extracts useful information

        auto start_time = std::chrono::high_resolution_clock::now();

        // *********************************************
        // Prepare pointcloud for object recognition
        // *********************************************

        // Different voxel filtering and planar segmentations are used for goals
        // and pucks/poles, because otherwise too much information for detection
        // is lost from one or another.

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // For goals
        voxel_grid_filter_m(cloud, pointcloud_temp, 0.01, 0);
        bool success = planar_segmentation(pointcloud_temp, pointcloud_floor,
                                           coefficients, true);

        // For pucks and poles
        // if (success) {
        voxel_grid_filter_m(cloud, pointcloud_temp, 0.015, 5);
        planar_segmentation(pointcloud_temp, pointcloud_not_floor, coefficients,
                            false);
        // plane_filter(pointcloud_temp, pointcloud_not_floor, coefficients,
        //             -0.02);
        //}

        edit_colors_of_pointcloud(pointcloud_floor);
        edit_colors_of_pointcloud(pointcloud_not_floor);

        // Calculate used time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto delta_time = end_time - start_time;
        std::cout << "Took for preparations "
                  << delta_time / std::chrono::milliseconds(1)
                  << "ms to run.\n";

        // *********************************************
        // Puck and Pole recognition
        // *********************************************

        // Remove outliers
        // radius_outlier_removal(pointcloud_not_floor, pointcloud_temp2, 0.02,
        // 3);

        // save_cloud_to_file(pointcloud_not_floor,
        //                   "/home/cnc/Desktop/Hockey/pucks_1.pcd");

        // Find pucks and poles from not floor pointcoud
        recognized_objects = combine_close_points(
            pointcloud_not_floor, is_buck_or_pole, 0.03, 16, 2500);

        remove_edge_detections(recognized_objects, transform_odom_to_baselink);

        // *********************************************
        // Goal recognition
        // *********************************************

        // OpenCV goals ~250 ms:*******************************************
        double x_offset = 0;
        double y_offset = 0;
        double pixel_size = 0.01;  // m
        int marginal = 0;          // pix
        cv::Mat floor_blue_edges;
        cv::Mat floor_yellow_edges;
        extract_edge_points_opencv(pointcloud_floor, x_offset, y_offset,
                                   pixel_size, marginal, floor_blue_edges,
                                   floor_yellow_edges);

        *recognized_objects += *(combine_close_points(
            get_goal_corners_opencv(floor_blue_edges, x_offset, y_offset,
                                    pixel_size, marginal, 0, 255, 255),
            is_goal_corner, 0.1, 1, 100));

        *recognized_objects += *(combine_close_points(
            get_goal_corners_opencv(floor_yellow_edges, x_offset, y_offset,
                                    pixel_size, marginal, 255, 140, 0),
            is_goal_corner, 0.1, 1, 100));

        // PCL goals ~300 ms:***********************************************
        /*
        edit_z_to(pointcloud_floor, 0);  // z = 0

        // Extract edge points based on color
        extract_edge_points(pointcloud_floor, pointcloud_temp,
                            pointcloud_temp2);

        // Get corners
        *recognized_objects += *(combine_close_points(
            get_goal_corners(pointcloud_temp, 0, 255, 255)));  // blue -> cyan
        *recognized_objects +=
            *(combine_close_points(get_goal_corners(pointcloud_temp2, 255, 140,
                                                    0)));  // yellow -> orange
        */

        // Calculate used time
        end_time = std::chrono::high_resolution_clock::now();
        delta_time = end_time - start_time;
        std::cout << "Took total " << delta_time / std::chrono::milliseconds(1)
                  << "ms to run.\n";

        return;  //----------------------------------------------------
        // Testing stuff:

        // save_cloud_to_file(pointcloud_not_floor,
        //"/home/cnc/Desktop/Hockey/PCL-cluster-segmentation-tests/not-floor2.pcd"
        //);

        // Filter region by color:
        // Blue goals
        // color_filter(pointcloud_floor, pointcloud_temp, 0, 0, 255);
        // Yellow goals
        // color_filter(pointcloud_floor, pointcloud_temp2, 255, 255, 0);

        // Remove outliers
        // radius_outlier_removal(pointcloud_temp, pointcloud_floor_blue, 0.2,
        // 25); radius_outlier_removal(pointcloud_temp2,
        // pointcloud_floor_yellow, 0.2,25);

        // Remove outliers from filtered pointclouds
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*pointcloud_not_floor_blue,
        //                             *pointcloud_temp, indices);
        // if (pointcloud_not_floor_blue->points.size() > 0) {
        //    radius_outlier_removal(pointcloud_temp,
        //    pointcloud_not_floor_blue);
        //}
    }

    PointCloudPtr& get_floor_pointcloud() { return pointcloud_floor; }
    PointCloudPtr& get_not_floor_pointcloud() { return pointcloud_not_floor; }
    PointCloudPtr& get_recognized_objects() { return recognized_objects; }

   private:
    pcl::PassThrough<PointType> pass_through_filter;
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    pcl::ConditionalRemoval<PointType> conditional_filter;
    pcl::StatisticalOutlierRemoval<PointType>
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
};