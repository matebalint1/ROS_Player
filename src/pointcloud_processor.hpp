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

// Recognition
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPrt;

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

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

        pointcloud_puck_model = PointCloudPrt(new PointCloud);
        pointcloud_pole_model = PointCloudPrt(new PointCloud);
        generate_puck_pointcloud(pointcloud_puck_model);
        generate_pole_pointcloud(pointcloud_pole_model);

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
    /*
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
    }*/

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
        // Not very useful
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
                // r = 0;
                // g = 0;
                // b = 0;
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

    void radius_outlier_removal(PointCloudPrt& cloud_in,
                                PointCloudPrt& cloud_out, double radius = 0.05,
                                int min_neighbors = 5) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        // build the filter
        outrem.setInputCloud(cloud_in);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(min_neighbors);
        // apply filter
        outrem.filter(*cloud_out);
    }

    void generate_puck_pointcloud(PointCloudPrt& cloud) {
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

    void generate_pole_pointcloud(PointCloudPrt& cloud) {
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

    void object_recognition(PointCloudPrt& scene, PointCloudPrt& model) {
        // Algorithm params
        bool show_keypoints_(false);
        bool show_correspondences_(false);
        bool use_cloud_resolution_(false);
        bool use_hough_(false);
        float model_ss_(0.01f);
        float scene_ss_(0.03f);
        float rf_rad_(0.015f);
        float descr_rad_(0.02f);
        float cg_size_(10 * 0.01f);
        float cg_thresh_(10 * 5.0f);

        pcl::PointCloud<PointType>::Ptr model_keypoints(
            new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr scene_keypoints(
            new pcl::PointCloud<PointType>());
        pcl::PointCloud<NormalType>::Ptr model_normals(
            new pcl::PointCloud<NormalType>());
        pcl::PointCloud<NormalType>::Ptr scene_normals(
            new pcl::PointCloud<NormalType>());
        pcl::PointCloud<DescriptorType>::Ptr model_descriptors(
            new pcl::PointCloud<DescriptorType>());
        pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(
            new pcl::PointCloud<DescriptorType>());

        //
        //  Compute Normals
        //
        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        norm_est.setKSearch(10);
        norm_est.setInputCloud(model);
        norm_est.compute(*model_normals);

        norm_est.setInputCloud(scene);
        norm_est.compute(*scene_normals);

        //
        //  Downsample Clouds to Extract keypoints
        //
        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud(model);
        uniform_sampling.setRadiusSearch(model_ss_);
        // uniform_sampling.filter(*model_keypoints);
        pcl::PointCloud<int> keypointIndices1;
        uniform_sampling.compute(keypointIndices1);
        pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);

        std::cout << "Model total points: " << model->size()
                  << "; Selected Keypoints: " << model_keypoints->size()
                  << std::endl;

        uniform_sampling.setInputCloud(scene);
        uniform_sampling.setRadiusSearch(scene_ss_);
        // uniform_sampling.filter(*scene_keypoints);
        pcl::PointCloud<int> keypointIndices2;
        uniform_sampling.compute(keypointIndices2);
        pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);

        std::cout << "Scene total points: " << scene->size()
                  << "; Selected Keypoints: " << scene_keypoints->size()
                  << std::endl;

        //
        //  Compute Descriptor for keypoints
        //
        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        descr_est.setRadiusSearch(descr_rad_);

        descr_est.setInputCloud(model_keypoints);
        descr_est.setInputNormals(model_normals);
        descr_est.setSearchSurface(model);
        descr_est.compute(*model_descriptors);

        descr_est.setInputCloud(scene_keypoints);
        descr_est.setInputNormals(scene_normals);
        descr_est.setSearchSurface(scene);
        descr_est.compute(*scene_descriptors);

        //
        //  Find Model-Scene Correspondences with KdTree
        //
        pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

        pcl::KdTreeFLANN<DescriptorType> match_search;
        match_search.setInputCloud(model_descriptors);

        //  For each scene keypoint descriptor, find nearest neighbor into the
        //  model keypoints descriptor cloud and add it to the correspondences
        //  vector.
        for (size_t i = 0; i < scene_descriptors->size(); ++i) {
            std::vector<int> neigh_indices(1);
            std::vector<float> neigh_sqr_dists(1);
            if (!std::isfinite(
                    scene_descriptors->at(i).descriptor[0]))  // skipping NaNs
            {
                continue;
            }
            int found_neighs = match_search.nearestKSearch(
                scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
            if (found_neighs == 1 &&
                neigh_sqr_dists[0] <
                    0.25f)  //  add match only if the squared descriptor
                            //  distance is less than 0.25 (SHOT descriptor
                            //  distances are between 0 and 1 by design)
            {
                pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
                                         neigh_sqr_dists[0]);
                model_scene_corrs->push_back(corr);
            }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size()
                  << std::endl;

        //
        //  Actual Clustering
        //
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
            rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;

        //  Using Hough3D
        if (use_hough_) {
            //
            //  Compute (Keypoints) Reference Frames only for Hough
            //
            pcl::PointCloud<RFType>::Ptr model_rf(
                new pcl::PointCloud<RFType>());
            pcl::PointCloud<RFType>::Ptr scene_rf(
                new pcl::PointCloud<RFType>());

            pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType,
                                                    RFType>
                rf_est;
            rf_est.setFindHoles(true);
            rf_est.setRadiusSearch(rf_rad_);

            rf_est.setInputCloud(model_keypoints);
            rf_est.setInputNormals(model_normals);
            rf_est.setSearchSurface(model);
            rf_est.compute(*model_rf);

            rf_est.setInputCloud(scene_keypoints);
            rf_est.setInputNormals(scene_normals);
            rf_est.setSearchSurface(scene);
            rf_est.compute(*scene_rf);

            //  Clustering
            pcl::Hough3DGrouping<PointType, PointType, RFType, RFType>
                clusterer;
            clusterer.setHoughBinSize(cg_size_);
            clusterer.setHoughThreshold(cg_thresh_);
            clusterer.setUseInterpolation(true);
            clusterer.setUseDistanceWeight(false);

            clusterer.setInputCloud(model_keypoints);
            clusterer.setInputRf(model_rf);
            clusterer.setSceneCloud(scene_keypoints);
            clusterer.setSceneRf(scene_rf);
            clusterer.setModelSceneCorrespondences(model_scene_corrs);

            // clusterer.cluster (clustered_corrs);
            clusterer.recognize(rototranslations, clustered_corrs);
        } else {
            // Using GeometricConsistency
            pcl::GeometricConsistencyGrouping<PointType, PointType>
                gc_clusterer;
            gc_clusterer.setGCSize(cg_size_);
            gc_clusterer.setGCThreshold(cg_thresh_);

            gc_clusterer.setInputCloud(model_keypoints);
            gc_clusterer.setSceneCloud(scene_keypoints);
            gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

            // gc_clusterer.cluster (clustered_corrs);
            gc_clusterer.recognize(rototranslations, clustered_corrs);
        }

        std::cout << "Model instances found: " << rototranslations.size()
                  << std::endl;
        for (size_t i = 0; i < rototranslations.size(); ++i) {
            std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
            std::cout << "        Correspondences belonging to this instance: "
                      << clustered_corrs[i].size() << std::endl;

            // Print the rotation matrix and translation vector
            Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
            Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

            printf("\n");
            printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0),
                   rotation(0, 1), rotation(0, 2));
            printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0),
                   rotation(1, 1), rotation(1, 2));
            printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0),
                   rotation(2, 1), rotation(2, 2));
            printf("\n");
            printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0),
                   translation(1), translation(2));
        }
    }

    void save_cloud_to_file(PointCloudPrt& cloud, std::string path_and_name) {
        if (cloud->points.size() > 0) {
            pcl::io::savePCDFileASCII(path_and_name, *cloud);
        }
    }

    void process_pointcloud(PointCloudPrt& cloud) {
        // Processes a new point cloud and extracts useful information

        // Remove NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *pointcloud_temp, indices);

        // Reduce nuber of points in the pointcloud
        voxel_grid_filter_m(pointcloud_temp, pointcloud_temp2);

        // Segment cloud into floor and not floor
        planar_segmentation(pointcloud_temp2, pointcloud_floor,
                            pointcloud_not_floor);

        // Find interesting colors in the cloud (maps specific color regions to
        // a specific color value for filtering the points later)
        edit_colors_of_pointcloud(pointcloud_floor);
        edit_colors_of_pointcloud(pointcloud_not_floor);

        // Filter region by color:
        // Blue goals
        color_filter(pointcloud_floor, pointcloud_floor_blue, 0, 0, 255);
        // Yellow goals
        color_filter(pointcloud_floor, pointcloud_floor_yellow, 255, 255, 0);

        // Blue bucks
        color_filter(pointcloud_not_floor, pointcloud_not_floor_blue, 0, 0,
                     255);
        // Green bucks
        color_filter(pointcloud_not_floor, pointcloud_not_floor_green, 0, 255,
                     0);
        // Yellow bucks
        color_filter(pointcloud_not_floor, pointcloud_not_floor_yellow, 255,
                     255, 0);

        // Remove outliers from filtered pointclouds
        pcl::removeNaNFromPointCloud(*pointcloud_not_floor_blue,
                                     *pointcloud_temp, indices);
        if (pointcloud_not_floor_blue->points.size() > 0) {
            radius_outlier_removal(pointcloud_temp, pointcloud_not_floor_blue);
        }
        // TODO ...

        // Find pucks and poles from not floor pointclouds
        // TODO

        // Find goals from floor pointclouds
        // TODO

        // Return found objects to pointcloud_node
        // TODO

        // For reference and debugging:
        // std::cerr << "Cloud before filtering: " << std::endl;
        // std::cerr << *pointcloud_not_floor_green << std::endl;
        // statistical_outlier_removal_filter_m(pointcloud_not_floor_green,
        //                                     pointcloud_temp);
        // std::cerr << "Cloud after filtering: " << std::endl;
        // std::cerr << *pointcloud_temp << std::endl;
    }

    PointCloudPrt& get_floor_pointcloud() { return pointcloud_floor; }
    PointCloudPrt& get_not_floor_pointcloud() {
        return pointcloud_not_floor;
    }

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

    PointCloudPrt pointcloud_puck_model;
    PointCloudPrt pointcloud_pole_model;
};