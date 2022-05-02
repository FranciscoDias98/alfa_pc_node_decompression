#ifndef ALFA_PD_H
#define ALFA_PD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"


#define DDR_SIZE 0x060000;
#define CONFIG_SIZE 0x0006;
#define DDR_BASE_PTR 0x0F000000;
#define CONFIGS_BASE_PTR 0xA0000000;

using namespace std;
typedef long long int u64;

class Alfa_Pd
{
public:
    Alfa_Pd();
    void do_voxelfilter();
    void do_sorfilter();
    void do_drorfilter();
    void do_rorfilter();
    void do_fcsorfilter();
    void do_LIORfilter();
    void do_DIORfilter();
    void do_hardwarefilter();

    void apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud);
    void update_filterSettings(const alfa_msg::AlfaConfigure::Request configs);
private:
    unsigned int filter_number;
    float parameter1;
    float parameter2;
    float parameter3;
    float parameter4;
    float parameter5;
    bool use_multi;
    bool hardware_ready;
    int number_threads;
    long unsigned int frame_id;
    vector<boost::thread *> thread_list;
    u64 *ddr_pointer;
    uint32_t *configs_pointer;

    void run_worker(int thread_number);
    void run_lior_worker(int thread_number);
    void run_dlior_worker(int thread_number);
    void filter_point(pcl::PointXYZI point,bool isDIOR=0);
    void filter_pointROR(pcl::PointXYZI point);
    void decode_pointcloud();


    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, outputCloud;

    alfa_msg::AlfaMetrics outputMetrics;


};

#endif // ALFA_PD_H
