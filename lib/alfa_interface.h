#include <ros/ros.h>


// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"
#include "alfa_msg/AlfaAlivePing.h"

#include "alfa_pd.h"  //include user defined module

#define NODE_NAME "alfa_pd"

#define NODE_TYPE "Weather denoising"


#define TIMER_SLEEP 50000


using namespace std;
class AlfaInterface
{
public:
    AlfaInterface();
    void publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud);
    void publish_metrics(alfa_msg::AlfaMetrics &metrics);


private:
    void cloud_cb (const  sensor_msgs::PointCloud2ConstPtr& cloud);
    bool parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res);
    ros::Subscriber sub_cloud;
    ros::ServiceServer sub_parameters;
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud;
    void init();
    void subscrive_topics();
    void ticker_thread();
    boost::thread *m_spin_thread;
    ros::Publisher filter_metrics;
    ros::Publisher alive_publisher;
    ros::Publisher cloud_publisher;

    boost::thread* alive_ticker;

    void spin();

    Alfa_Pd* node; //define a object of the defined module



};





