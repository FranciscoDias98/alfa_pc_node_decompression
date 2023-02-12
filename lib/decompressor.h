#include <stdint.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <inttypes.h>
//#include <pcl/compression/octree_pointcloud_compression.h>
#include "octree_pointcloud_compression_2.h"
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
    void exe_time();
    alfa_msg::AlfaMetrics output_metrics;
    void process_pointcloud(compressed_pointcloud_transport::CompressedPointCloud input);
    alfa_msg::AlfaConfigure::Response   process_config(alfa_msg::AlfaConfigure::Request &req);
    void metrics(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,double duration);

    // for multi
    void my_read_frame_header(std::istream &compressed_data);
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud,octant_0,octant_1,octant_2,octant_3,octant_4,octant_5,octant_6,octant_7;
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

    //new for multi-threading compression
    bool multi_thread;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_0;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_1;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_2;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_3;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_4;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_5;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_6;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* point_cloud_decoder_7;
    uint64_t point_count_0,point_count_1,point_count_2,point_count_3,point_count_4,point_count_5,point_count_6,point_count_7;
    unsigned int size_0,size_1,size_2,size_3,size_4,size_5,size_6,size_7;

};
