#include <math.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Bool.h"

namespace arc {
namespace obstacle_detection {

class DistanceHistogram {
 public:
  DistanceHistogram();
  ~DistanceHistogram();
  std::vector<std::vector<double> > d_histo_15_;
  std::vector<std::vector<double> > d_histo_13_;
  std::vector<std::vector<double> > d_histo_11_;
  std::vector<std::vector<double> > d_histo_9_;
  std::vector<std::vector<double> > d_histo_7_;
  std::vector<std::vector<double> > d_histo_5_;
  std::vector<std::vector<double> > d_histo_3_;
  std::vector<std::vector<double> >* d_histo_ptr_[7];
};

class Obstacle_Detection {
 public:
  Obstacle_Detection(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
  ~Obstacle_Detection();
  void conversion_PC2toPCL(const sensor_msgs::PointCloud2& cloud_message, pcl::PointCloud<pcl::PointXYZ>& temp_cloud);
  void conversion_PCLtoPC2(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud, sensor_msgs::PointCloud2& converted);
  void cloudCallback(const sensor_msgs::PointCloud2& cloud_message);
  void steeringAngleCallback(const std_msgs::Float64::ConstPtr& msg);
  void scan(const sensor_msgs::PointCloud2& cloud_message);
  void histogram_allocation(double d, int j, DistanceHistogram& temp);
  void Filter(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud, double* inter_d_ptr);
  void GridMap(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud, nav_msgs::OccupancyGrid& grid); 
  void shut_down(const std_msgs::Bool::ConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher obstacle_pub_;
  ros::Publisher gridmap_pub_;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber shutdown_sub_;
  ros::Subscriber steering_angle_sub_;
  double boundary_slope_;
  double y_limit_m_;
  double tolerance_m_;
  double tolerance_factor_;
  double norm_delta_;
  double angle_;
  double delta_factor_;
  double height_laser_;
  int number_points_;
  std::vector<std::vector<double> > points_15_;
  std::vector<std::vector<double> > points_13_;
  std::vector<std::vector<double> > points_11_;
  std::vector<std::vector<double> > points_9_;
  std::vector<std::vector<double> > points_7_;
  std::vector<std::vector<double> > points_5_;
  std::vector<std::vector<double> > points_3_;
  std::vector<std::vector<double> > points_0_;
  std::vector<std::vector<double> >* points_ptr_[8];
};

}  //End obstacle_detection
}  //End arc

