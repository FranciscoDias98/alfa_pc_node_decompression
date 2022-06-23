#include "decompressor.h"
#include <iostream>


unsigned int x = 0;
unsigned long tempos = 0;
unsigned long size =0;

float size_decompressed = 0;

Alfa_Pc_Decompress::Alfa_Pc_Decompress()
{
    std::cout << "entrei no construtor" << std::endl;
    out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    show_statistics = true;
    compression_profile = pcl::io::MANUAL_CONFIGURATION;
    point_cloud_decoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    //sub_topic();
    //pub_();
    //spin();

    output_metrics.message_tag = "Decompression performance";

}

void Alfa_Pc_Decompress::do_Decompression()
{
    //cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    //decompress
    std::cout << "entrei na descompressao" << std::endl;
    auto start = high_resolution_clock::now();
    point_cloud_decoder->decodePointCloud(compressed_data,output_cloud);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    ROS_INFO("Decompressing in %ld ms",duration.count());
    ROS_INFO("Tree depth: %d\n",point_cloud_decoder->getTreeDepth());
}


void Alfa_Pc_Decompress::process_pointcloud(compressed_pointcloud_transport::CompressedPointCloud input)
{
    output_metrics.metrics.clear();

    std::cout << "Entrei na Descompressao" << std::endl;
    //cloud

    //point_cloud_decoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    std::cout << input.header << std::endl;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud;
    // compressed point cloud data stream
    std::stringstream compressed_data(input.data);

    auto start = high_resolution_clock::now();

    //do_Decompression();
    out_cloud->clear();
    point_cloud_decoder->decodePointCloud(compressed_data,out_cloud);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    std::cout << "Fiz Descompressao  " << out_cloud->size() << std::endl;

    compressed_data.seekg(0,ios::end);
    size = compressed_data.tellg();
    std::cout << "Size comrpessed  " << size << std::endl;

    size_decompressed = (static_cast<float> (out_cloud->size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;

    alfa_msg::MetricMessage new_message;


    new_message.metric = size_decompressed/1000;
    new_message.units = "kB";
    new_message.metric_name = "Size Decompressed";
    output_metrics.metrics.push_back(new_message);


    new_message.metric = out_cloud->size();
    new_message.units = "";
    new_message.metric_name = "Nº of Points in Point Cloud";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = duration.count();
    new_message.units = "ms";
    new_message.metric_name = "Total processing time";
    output_metrics.metrics.push_back(new_message);


    publish_pointcloud(out_cloud);
    publish_metrics(output_metrics);
}

alfa_msg::AlfaConfigure::Response Alfa_Pc_Decompress::process_config(alfa_msg::AlfaConfigure::Request &req)
{

}



void Alfa_Pc_Decompress::exe_time(unsigned long size, unsigned long time)
{
    time = time/100 ;
    size = size/100;
    std::ofstream myFile("./output/exe_time");
    myFile<< "Exe. Time: "<< time << std::endl << "Compressed Size: " <<size;
    myFile.close();
    std::cout << "-----------Acabei------------------------------------------------------------- \n" ;
    x=0;
    size = 0;
}
