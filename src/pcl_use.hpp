#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

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

