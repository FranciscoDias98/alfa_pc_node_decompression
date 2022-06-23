#include <stdint.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <inttypes.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <chrono>
#include <string>
#include <std_msgs/String.h>
#include <bitset>
#include <std_msgs/UInt8MultiArray.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <iterator>
#include <chrono>
#include <time.h>
#include "alfa_node.h"
#include "CompressedPointCloud.h"



using namespace std::chrono;

class Alfa_Pc_Decompress: public AlfaNode
{
public:
    Alfa_Pc_Decompress();
    void do_Decompression();
    //void cloud_cb (const compressed_pointcloud_transport::CompressedPointCloud::ConstPtr& input);
    void exe_time(unsigned long size,unsigned long time);
    alfa_msg::AlfaMetrics output_metrics;
    void process_pointcloud(compressed_pointcloud_transport::CompressedPointCloud input);
    alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);
    void metrics(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,double duration);
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud;
    sensor_msgs::PointCloud2 decompressed_cloud;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder;
    stringstream compressed_data;
    bool show_statistics;
    pcl::io::compression_Profiles_e compression_profile;
    ros::Subscriber sub_;
    ros::Publisher pub_cloud;
    ros::NodeHandle nh;
    void init();
    void sub_topic();
    void pub_();
    void spin();
    Alfa_Pc_Decompress* node;
    boost::shared_ptr<ros::Subscriber> sub__;

};
