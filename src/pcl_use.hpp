#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

template <typename PointT>
inline void savePcd(std::string name, const typename pcl::PointCloud<PointT>& cloud) {
    if (cloud.size() > 0) {
        pcl::io::savePCDFileBinary<PointT>(name, cloud);
    } else {
        printf("%s is empty \n", name.c_str());
    }
}

template <typename PointT>
inline void outlierFilter(typename pcl::PointCloud<PointT>::Ptr& cloud, float radius = 0.04, int num = 2) {
    pcl::RadiusOutlierRemoval<PointT> rad;
    rad.setInputCloud(cloud);
    rad.setRadiusSearch(radius);
    rad.setMinNeighborsInRadius(num);
    rad.filter(*cloud);
}

template <typename PointT>
inline void viewCompareCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud1, 
    const typename pcl::PointCloud<PointT>::Ptr& cloud2) {
    pcl::visualization::PCLVisualizer::Ptr viewerPtr(new pcl::visualization::PCLVisualizer("viewer"));
    viewerPtr->removePointCloud("cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud1, 255, 0, 0);
    viewerPtr->addPointCloud(cloud1, red, "cloud1");
    viewerPtr->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");

    viewerPtr->removePointCloud("cloud2");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud2, 0, 255, 0);
    viewerPtr->addPointCloud(cloud2, green, "cloud2");
    viewerPtr->spin();
}

template <typename PointT>
inline void voxelFilter(typename pcl::PointCloud<PointT>::Ptr& cloud, float voxel_size = 0.4) {
    pcl::VoxelGrid<PointT> sor;//滤波处理对象
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);//设置滤波器处理时采用的体素大小的参数
    sor.filter(*cloud);
}

template <typename PointT>
inline void voxelFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud, 
    typename pcl::PointCloud<PointT>::Ptr& cloud_filter,
    float voxel_size = 0.4) {
    pcl::VoxelGrid<PointT> sor;//滤波处理对象
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);//设置滤波器处理时采用的体素大小的参数
    sor.filter(*cloud_filter);
}

template <typename PointT>
inline void randomFilter(typename pcl::PointCloud<PointT>::Ptr& cloud, int num = 500) {
    pcl::RandomSample<PointT> random_filter;
    random_filter.setSample(num);
    random_filter.setInputCloud(cloud);
    random_filter.filter(*cloud);
}



