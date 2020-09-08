#include <string>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/common/transforms.h>  
#include <pcl_use.hpp>

using namespace std;
using namespace pcl;

string extractString(string s) {
    size_t pos = s.find(".");
    string str1 = s.substr(0, pos);
    return str1;
}

void processPcd(string pcdfile1, string pcdfile2) {
    double start, finish;
    start = clock();
    PointCloud<PointXYZI>::Ptr cloud1(new PointCloud<PointXYZI>), 
        cloud2(new PointCloud<PointXYZI>);
    io::loadPCDFile(pcdfile1, *cloud1);
    io::loadPCDFile(pcdfile2, *cloud2);
    if (cloud1->size() < 1 || cloud2->size() < 1) {
        printf("have empty cloud, please check \n");
        return;
    }
    outlierFilter<PointXYZI>(cloud1, 0.1, 2);
    outlierFilter<PointXYZI>(cloud2, 0.1, 2);
    PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
    copyPointCloud(*cloud1, *src);
    PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);
    copyPointCloud(*cloud2, *tgt);
    voxelFilter<PointNormal>(src, 0.1);
    voxelFilter<PointNormal>(tgt, 0.1);
    randomFilter<PointNormal>(src, 400);
    randomFilter<PointNormal>(tgt, 400);
    start = clock();
    NormalEstimation<PointNormal, PointNormal> norm_est;
    norm_est.setSearchMethod(search::KdTree<PointNormal>::Ptr(new search::KdTree<PointNormal>));
    norm_est.setKSearch(40);
    norm_est.setInputCloud(tgt);
    norm_est.compute(*tgt);

    IterativeClosestPoint<PointNormal, PointNormal> icp;
    typedef registration::TransformationEstimationPointToPlane<PointNormal, PointNormal> PointToPlane;
    boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
    icp.setTransformationEstimation(point_to_plane); // key

    icp.setInputSource(src);
    icp.setInputTarget(tgt);

    icp.setRANSACIterations(30);
    icp.setMaximumIterations(60);
    icp.setTransformationEpsilon(1e-5);
    icp.setMaxCorrespondenceDistance(0.4); //点对的最大距离超过0.1则忽略!!!!!!
    PointCloud<PointNormal> output;
    icp.align(output); // align 的另一个重载可以设置一个初始矩阵guess
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    float fitnessScore = icp.getFitnessScore();
    cout << "T: " << endl;
    cout << T << endl;
    // cout << "fitnessScore " << fitnessScore << endl;
    finish = clock();
    printf("icp time is %f sec\n", (finish - start) / CLOCKS_PER_SEC);
    start = clock();
    transformPointCloud(*cloud1, *cloud1, T);
    PointCloud<PointXYZRGB>::Ptr color_cloud1(new PointCloud<PointXYZRGB>),
    color_cloud2(new PointCloud<PointXYZRGB>);
    for (const auto& p : cloud1->points) {
        PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.r = 255;
        color_cloud1->push_back(point);
    }
    for (const auto& p : cloud2->points) {
        PointXYZRGB point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        point.g = 255;
        color_cloud2->push_back(point);
    }
    string s1 = extractString(pcdfile1);
    savePcd<PointXYZRGB>(s1 + "_new.pcd", *color_cloud1);
    string s2 = extractString(pcdfile2);
    savePcd<PointXYZRGB>(s2 + "_new.pcd", *color_cloud2);
    viewCompareCloud<PointXYZI>(cloud1, cloud2);
}

int main(int argc, char** argv) {
    if (argv[1] == nullptr || argv[2] == nullptr) {
        printf("you need input two pcds, please check !\n");
        return 0;
    }    
    processPcd(argv[1], argv[2]);
    return 1;
}