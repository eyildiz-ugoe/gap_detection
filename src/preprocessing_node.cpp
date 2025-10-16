#include <ros/ros.h>
#include <ros/package.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
// Dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <ugoe_gap_detection_ros/preprocessingConfig.h>
// Denoising
#include <pcl/filters/statistical_outlier_removal.h>

#include<iostream>
using namespace std;

// PUBLISHER AND SUBSCRIBER
ros::Publisher filtered_pub;
ros::Publisher denoised_pub;
ros::Subscriber sub;

// DYNAMIC RECONFIGURE
// passthrough:
float zLowerLimit;
float zUpperLimit;
float yLowerLimit;
float yUpperLimit;
float xLowerLimit;
float xUpperLimit;

// denoising:
int meanK;
float stddevMulThresh;

// pcd creation for evaluation
bool create_PCD;


#if __cplusplus < 201103L
string to_string(size_t n) {
  string str;
  if (not n) {
    return "0";
  }
  while (n) {
    str.push_back('0'+n%10);
    n /= 10;
  }
  reverse(str.begin(), str.end());
  return str;
}
#endif


void create_denoised_PCD(pcl::PointCloud<pcl::PointXYZ> cloud) {
  /*
    Creates a PCD file in ASCII format ...

    Parameters
    ----------

    Returns
    -------

  */
string src_path = ros::package::getPath("ugoe_gap_detection_ros").c_str();
string file_path = src_path + "/evaluation/denoised_pcd.pcd";
pcl::io::savePCDFileASCII(file_path, cloud);
string msg = "Saved " + to_string(cloud.points.size()) + " data points to " + file_path;

}

void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp_cloud_ptr);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stddevMulThresh);
  sor.filter(*temp_cloud_ptr);
}

void reconf_callback(ugoe_gap_detection_ros::preprocessingConfig &config, uint32_t level) {
  zLowerLimit = config.zLowerLimit;
  zUpperLimit = config.zUpperLimit;
  yLowerLimit = config.yLowerLimit;
  yUpperLimit = config.yUpperLimit;
  xLowerLimit = config.xLowerLimit;
  xUpperLimit = config.xUpperLimit;

  meanK = config.MeanK;
  stddevMulThresh = config.StdDevMulThresh;

  create_PCD = config.create_PCD;
  // directly uncheck checkbox to use it similar to a button
  config.create_PCD = false;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  /* ----- ROSMSG CONVERSION ----- */
  sensor_msgs::PointCloud2 output;
  output = *input;
  // convert sensor-type cloud to processable-type cloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> &temp_cloud = *temp_cloud_ptr;
  pcl::fromPCLPointCloud2(pcl_pc2,temp_cloud);

  /* ----- PASSTHROUGH FILTER ----- */
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(temp_cloud_ptr);
  // filter in x-direction
  pass.setFilterFieldName("x");
  pass.setFilterLimits(xLowerLimit, xUpperLimit);
  pass.filter(temp_cloud);
  // filter in y-direction
  pass.setFilterFieldName("y");
  pass.setFilterLimits(yLowerLimit, yUpperLimit);
  pass.filter(temp_cloud);
  // filter in z-direction
  pass.setFilterFieldName("z");
  pass.setFilterLimits(zLowerLimit, zUpperLimit);
  pass.filter(temp_cloud);

  /* ----- PUBLISHING THE FILTERED CLOUD ----- */
  pcl::toPCLPointCloud2(temp_cloud, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, output);
  filtered_pub.publish(output);

  /* ----- DENOISING THE FILTERED CLOUD ----- */
  if(temp_cloud.empty() == true) 
  {
    ROS_INFO("Pointcloud is empty!");
    return;
  }

  statistical_outlier_removal(temp_cloud_ptr);

  /* ----- CREATE A PCD FOR EVALUATION ----- */
  if(create_PCD) {
    create_denoised_PCD(temp_cloud);
    create_PCD = false;
  }

  /* ----- PUBLISHING THE DENOISED CLOUD ----- */
  pcl::toPCLPointCloud2(temp_cloud, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, output);
  denoised_pub.publish(output);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "preprocessing_node");
  ros::NodeHandle nh;

  // Dynamic reconfigure configuration
  dynamic_reconfigure::Server<ugoe_gap_detection_ros::preprocessingConfig> server;
  dynamic_reconfigure::Server<ugoe_gap_detection_ros::preprocessingConfig>::CallbackType callbackBinding;
  callbackBinding = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(callbackBinding);

  sub = nh.subscribe("/nerian_stereo/point_cloud", 1, cloud_cb);
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  denoised_pub = nh.advertise<sensor_msgs::PointCloud2>("denoised_cloud", 1);

  ros::spin();
}
