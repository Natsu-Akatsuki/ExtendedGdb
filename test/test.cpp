#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace std::chrono_literals;

int main() {

  cv::Mat matImg = cv::imread("../gogh.jpg", cv::IMREAD_GRAYSCALE);

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // 使用时，需要在对应行加入如下代码（让编译器对该函数进行编译），否则出现如下报错
  // gdb.error: Cannot evaluate function -- may be inlined
  target_cloud.get()->points.data();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../scan.pcd", *target_cloud) == -1) {
    PCL_ERROR("Couldn't read pointcloud file \n");
    return (-1);
  }

  return (0);
}