一、问题描述

使用icp或ndt方法完成两帧点云配准, 对匹配后的点云进行去噪处理；
输出：匹配去噪后的点云pcd文件、转换坐标矩阵和耗时，坐标变换后的两帧点云分别用不同颜色；
二、依赖库

主要依赖于pcl库和Eigen3库。
三、编译指令

mkdir -p ~/catkin_ws/src
cp -r cloud_registration ~/catkin_ws/src
cd catkin_ws
catkin_make -j4

四、执行

也可以做一个launch文件，但既然全程没有使用ROS,这里也就没有使用launch文件。
cd ~/catkin_ws/devel/lib/cloud_registration
./test_pcd ~/catkin/src/cloud_registration/pcd/cloud_002.pcd ~/catkin/src/cloud_registration/pcd/cloud_003.pcd

会在终端输出T和耗时。
在~/catkin/src/cloud_registration/pcd文件夹下生成两个新的pcd文件。
在程序执行的最后，自动弹出一个窗口显示配准结果，如下图所示，为了比较显示，把红色点云每个点的size放大了5倍。

终端输出如下图所示：

