#include "../include/arc_obstacle_detection/obstacle_detection.hpp"

namespace arc {
namespace obstacle_detection {

Obstacle_Detection::Obstacle_Detection(const ros::NodeHandle &nh,
                                       const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh) {

  obstacle_sub_ = nh_.subscribe("/velodyne_points", 1,
                                &Obstacle_Detection::cloudCallback, this);

  shutdown_sub_ = nh_.subscribe("/state/stop", 1,
   &Obstacle_Detection::shut_down, this);

  steering_angle_sub_ = nh_.subscribe("/state_steering_angle", 1,
   &Obstacle_Detection::steeringAngleCallback, this);

  obstacle_pub_ = nh_.advertise < sensor_msgs::PointCloud2 > ("/obstacles", 1);
  gridmap_pub_ = nh_.advertise < nav_msgs::OccupancyGrid > ("/gridmap", 1);
  //nh.getParam("/safety/STATIC_TOLERANCE_LASER", tolerance_m_);
  //nh.getParam("/safety/FACTOR_TOLERANCE_LASER", tolerance_factor_);
  //nh.getParam("/safety/SEARCH_WIDTH", y_limit_m_);
  //nh.getParam("/safety/NORM_DELTA", norm_delta_);
  //nh.getParam("/safety/OD_ANGLE", angle_);
  //nh.getParam("/safety/NUMBER_POINTS", number_points_);
  boundary_slope_ = 0;
  tolerance_m_ = 0.5;
  tolerance_factor_ = 0.3;
  y_limit_m_= 2;
  norm_delta_ = 0.3; // 0.9
  angle_ = 20;  //20
  delta_factor_ = 0.0;
  number_points_ = 400;
  height_laser_ = 0.9;
  points_ptr_[0] = &points_15_;
  points_ptr_[1] = &points_13_;
  points_ptr_[2] = &points_11_;
  points_ptr_[3] = &points_9_;
  points_ptr_[4] = &points_7_;
  points_ptr_[5] = &points_5_;
  points_ptr_[6] = &points_3_;
  points_ptr_[7] = &points_0_;
}

Obstacle_Detection::~Obstacle_Detection() {
}

DistanceHistogram::DistanceHistogram() {
  d_histo_ptr_[0] = &d_histo_15_;
  d_histo_ptr_[1] = &d_histo_13_;
  d_histo_ptr_[2] = &d_histo_11_;
  d_histo_ptr_[3] = &d_histo_9_;
  d_histo_ptr_[4] = &d_histo_7_;
  d_histo_ptr_[5] = &d_histo_5_;
  d_histo_ptr_[6] = &d_histo_3_;

}

DistanceHistogram::~DistanceHistogram() {
  d_histo_15_.clear();
  d_histo_13_.clear();
  d_histo_11_.clear();
  d_histo_9_.clear();
  d_histo_7_.clear();
  d_histo_5_.clear();
  d_histo_3_.clear();
}

void Obstacle_Detection::cloudCallback( const sensor_msgs::PointCloud2& cloud_message) {
  scan(cloud_message);
}

void Obstacle_Detection::steeringAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
  float steering_angle = (msg->data)*180/M_PI;
  if(fabs(steering_angle)>2) boundary_slope_=tan((msg->data))*2;
  else boundary_slope_=0;
}

void Obstacle_Detection::conversion_PC2toPCL(
    const sensor_msgs::PointCloud2& cloud_message,
    pcl::PointCloud<pcl::PointXYZ>& temp_cloud) {

  pcl::PCLPointCloud2 pcl_pc2_1;
  pcl_conversions::toPCL(cloud_message, pcl_pc2_1);
  pcl::fromPCLPointCloud2(pcl_pc2_1, temp_cloud);

}

void Obstacle_Detection::conversion_PCLtoPC2(
    pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
    sensor_msgs::PointCloud2& converted) {

  pcl::PCLPointCloud2 pcl_pc2_2;
  pcl::toPCLPointCloud2(filtered_cloud, pcl_pc2_2);
  pcl_conversions::fromPCL(pcl_pc2_2, converted);

}

void Obstacle_Detection::histogram_allocation(double d, int j,
                                              DistanceHistogram& temp) {
  std::vector < std::vector<double> > &d_histo_temp = *(temp.d_histo_ptr_[j]);
//Store
  if (d_histo_temp.size() == 0) {
    std::vector<double> v_dist;
    v_dist.push_back(d);
    v_dist.push_back(1);
    d_histo_temp.push_back(v_dist);
    return;
  } else {
    for (int i = 0; i < d_histo_temp.size(); i++) {
      double distance = d_histo_temp[i][0];
      if (distance - tolerance_m_ * (1 + tolerance_factor_ * j) <= d
          && d <= distance + tolerance_m_ * (1 + tolerance_factor_ * j)) {
        //Update the average distance of Intervall and the sum
        d_histo_temp[i][0] = (d_histo_temp[i][1] * d_histo_temp[i][0] + d)
            / (d_histo_temp[i][1] + 1);
        d_histo_temp[i][1] = d_histo_temp[i][1] + 1;
        return;
      }
    }

    //if no interval could be found, create a new interval
    std::vector<double> v_dist_2;
    v_dist_2.push_back(d);
    v_dist_2.push_back(1);
    d_histo_temp.push_back(v_dist_2);

}}

void Obstacle_Detection::Filter(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud, double* inter_d_ptr) {
  for (int j=0; j<7; j++) {
    std::vector <std::vector<double> > &temp_vector = *(points_ptr_[j]);
    for(int i=0; i<points_ptr_[j]->size(); i++) {
      double x = temp_vector[i][0];
      double y = temp_vector[i][1];
      //if((x>2.2) && ((-y_limit_m_+boundary_slope_*x < y) && (y < y_limit_m_+boundary_slope_*x))) {              //Check if index was updated 
      if((y < x) && (y > -x)) {
        double z = temp_vector[i][2];
        double check_d = sqrt(x*x + y*y + z*z);
          if (*(inter_d_ptr + j) - tolerance_m_ *(1 + tolerance_factor_*j) < check_d && 
                check_d < *(inter_d_ptr + j) + tolerance_m_ * (1 + tolerance_factor_*j)) {
            temp_vector[i][3] = 0;   //Street
    }}}
    for(int i=1;i<number_points_;i++) {
      if(temp_vector[i][0]==0) continue;
      int counter_end = 0;
      for(; (i-1-counter_end)>0; counter_end++) {
        if(temp_vector[i-1-counter_end][0]) break;
      }
      if(temp_vector[i-1-counter_end][0]==0) continue;
      if(temp_vector[i][0]>2.2 && ((fabs(temp_vector[i][2]+height_laser_)/temp_vector[i][0])<tan(angle_*M_PI/180)||(j==0))) {
        if(temp_vector[i][3]==1 && temp_vector[i-1-counter_end][3]==0) {
          double delta_x = temp_vector[i][0] - temp_vector[i-1-counter_end][0];
          double delta_z = temp_vector[i][2] - temp_vector[i-1-counter_end][2];
          double norm_delta = sqrt(delta_x*delta_x + delta_z*delta_z);
          if(norm_delta < norm_delta_+j*delta_factor_) {
            temp_vector[i][3]=0;
          }    
        }
      }  
    }// End for

    for(int i=number_points_-2;i>=0;i--) {
      if(temp_vector[i][0]==0) continue;
      int counter_end = 0;
      for(; (i+1+counter_end)<number_points_; counter_end++) {
        if(temp_vector[i+1+counter_end][0]!=0) break;
      }
      if(temp_vector[i+1+counter_end][0]==0) continue;
      if(temp_vector[i][0]>2.2 && (fabs(temp_vector[i][2]/temp_vector[i][0])<tan(angle_*M_PI/180)||(j==0))) {
        if(temp_vector[i][3]==1 && temp_vector[i+1+counter_end][3]==0) {
          double delta_x = temp_vector[i][0] - temp_vector[i+1+counter_end][0];
          double delta_z = temp_vector[i][2] - temp_vector[i+1+counter_end][2];
          double norm_delta = sqrt(delta_x*delta_x + delta_z*delta_z);
          if(norm_delta < norm_delta_+j*delta_factor_) {
            temp_vector[i][3]=0;
          }    
        }
      }
      if(temp_vector[i][3]==1 && (i!=399 && i!=0) && temp_vector[i][0]!=0) {
        pcl::PointXYZ temp(temp_vector[i][0] ,temp_vector[i][1] , temp_vector[i][2]);
        filtered_cloud.push_back(temp);
      }    
    }// End for

    temp_vector.clear();
  }
  std::vector <std::vector<double> > &temp_vector = *(points_ptr_[7]);
  for(int i=0; i<temp_vector.size();i++) {
    if(temp_vector[i][0]!=0) {
     pcl::PointXYZ temp(temp_vector[i][0] ,temp_vector[i][1] , temp_vector[i][2]);
     filtered_cloud.push_back(temp);
   }
  }
  temp_vector.clear();
}

void Obstacle_Detection::GridMap(pcl::PointCloud<pcl::PointXYZ>& filtered_cloud,
                                 nav_msgs::OccupancyGrid& grid) {
  double left_tolerance = 6;
  double right_tolerance = -6;
  double front_tolerance = 20;
  double back_tolerance = -20;
  double width = 120;
  double height = 400;
  double resolution = 0.10;

  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  grid.info.origin = pose;

  for (int i = 0; i < (width * height); i++) {
    grid.data.push_back(0);
  }

  for (int i = 0; i < filtered_cloud.size(); i++) {
    double x = filtered_cloud[i].x;
    double y = -filtered_cloud[i].y;
    double z = filtered_cloud[i].z;

    if ((x > (back_tolerance + 0.5968)) && (x < (front_tolerance + 0.5968)) && (y > (right_tolerance+resolution))
        && (y < left_tolerance)) {
      int a = round(
          round((x + 0.54968)  / resolution) * width + round(y/ resolution) + width / 2
              + (height / 2) * width-1);
      grid.data[a] = 100;
  }}
  gridmap_pub_.publish(grid);

}

void Obstacle_Detection::shut_down(const std_msgs::Bool::ConstPtr& msg) {
  if(&msg!=0) {
  ros::shutdown();
  ros::waitForShutdown();
}}

void Obstacle_Detection::scan(const sensor_msgs::PointCloud2& cloud_message) {
  for(int j=0; j<8; j++) {
    std::vector<double> temp_point;
    temp_point.push_back(0);    //If x=0, vector index was not updated, ignore
    temp_point.push_back(0);
    temp_point.push_back(0);
    temp_point.push_back(1);
    for(int i=0; i<=number_points_; i++) {
      points_ptr_[j]->push_back(temp_point);
  }}
  pcl::PointCloud < pcl::PointXYZ > temp_cloud;
  conversion_PC2toPCL(cloud_message, temp_cloud);
  DistanceHistogram Front;
  for (int i = 0; i < temp_cloud.size(); i++) {
    bool angle_assigned = false;
    double x = temp_cloud.points[i].x;
    if(x>2.2) {
    double y = temp_cloud.points[i].y;
    double z = temp_cloud.points[i].z;
    double d = sqrt(x * x + y * y + z * z);
    double alpha_deg = asin(z / d) / M_PI * 180;
    for (int j = 0; (!angle_assigned) && (j<8); j++) {
      if ((-15.1 + 2 * j < alpha_deg) && (alpha_deg < -14.9 + 2 * j)) {
        angle_assigned = true;        
        if ((-y_limit_m_+boundary_slope_*x < y) && (y < y_limit_m_+boundary_slope_*x) && (j!=7)) {
          //allocate an distance intevall for the point
          if(fabs((z+height_laser_)/x)<tan(angle_*M_PI/180) || (j==0)) {
            histogram_allocation(d, j, Front);
        }}
        if((y < x) && (y > -x)){
          std::vector <std::vector<double> > &temp_vector = *(points_ptr_[j]);
          double beta_deg = atan2(y,x)*180/M_PI+45;
          int index = beta_deg/90*number_points_;
          if(index<number_points_) {
            temp_vector[index][0] = x;
            temp_vector[index][1] = y;
            temp_vector[index][2] = z;
            temp_vector[index][3] = 1; //1-Obstacles, 0-Street
            std::vector<double> temp;
  }}}}}} //End for   

  int sum[7] = { 0, 0, 0, 0, 0, 0, 0 };  //number of points in the interval
  double inter_d[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  //distance most points had

  for (int i = 0; i < 7; i++) {
    std::vector < std::vector<double> > &d_histo_temp = *(Front.d_histo_ptr_[i]);
    //Search distance intervall, in which most points are found
    for (int j = 0; j < d_histo_temp.size(); j++) {
      if (d_histo_temp[j][1] > sum[i]) {
        sum[i] = d_histo_temp[j][1];
        inter_d[i] = d_histo_temp[j][0];
  }}}

  pcl::PointCloud < pcl::PointXYZ > filtered_cloud;
  Filter(filtered_cloud, inter_d);
//only for visualisation
  sensor_msgs::PointCloud2 converted;
  conversion_PCLtoPC2(filtered_cloud, converted);
  converted.header.stamp = ros::Time::now();
  converted.header.frame_id = cloud_message.header.frame_id;
  obstacle_pub_.publish(converted);
  nav_msgs::OccupancyGrid grid;
  GridMap(filtered_cloud, grid);
}}}
