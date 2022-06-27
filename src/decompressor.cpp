#include "decompressor.h"
#include <iostream>


unsigned int x = 0;
unsigned long tempos = 0;
unsigned long size_compressed =0;

unsigned long tempos_test = 0;
float size_compressed_test =0;
float size_original_test =0;

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



    //do_Decompression();
    out_cloud->clear();
    auto start = high_resolution_clock::now();
    point_cloud_decoder->decodePointCloud(compressed_data,out_cloud);
    std::cout << "Fiz Descompressao  " << out_cloud->size() << std::endl;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    size_original_test = size_original_test + (static_cast<float> (out_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;
    x++;
    compressed_data.seekg(0,ios::end);
    size_compressed_test = size_compressed_test+compressed_data.tellg();
    tempos_test = tempos_test + duration.count();

    if(x==100){
        x=0;
        exe_time();
    }
    //convert to sensor_msgs::PointCloud2
    //sensor_msgs::PointCloud2 decompressed_cloud;
    //decompressed_cloud.header = input.header;
    //pcl::toROSMsg(*output_cloud, decompressed_cloud);
    //publish decompressed cloud

    metrics(out_cloud,duration.count());


    publish_pointcloud(out_cloud);
    publish_metrics(output_metrics);
}

alfa_msg::AlfaConfigure::Response Alfa_Pc_Decompress::process_config(alfa_msg::AlfaConfigure::Request &req)
{

}

void Alfa_Pc_Decompress::metrics(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud,double duration)
{
    alfa_msg::MetricMessage new_message;
    int size_reconstructed;

    size_reconstructed = (static_cast<float> (output_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;

    new_message.metric = size_reconstructed/1000;
    new_message.units = "kB";
    new_message.metric_name = "Reconstructed Cloud Size";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = output_cloud->size();
    new_message.units = "";
    new_message.metric_name = "Nº of Points in Point Cloud";
    output_metrics.metrics.push_back(new_message);

    new_message.metric = duration;
    new_message.units = "ms";
    new_message.metric_name = "Total processing time";
    output_metrics.metrics.push_back(new_message);


}





void Alfa_Pc_Decompress::init()
{

    if (!ros::master::check())
    {
        cout <<"Failed to inicialize ros"<<endl;
        return;
    }
}

void Alfa_Pc_Decompress::spin()
{
    std::cout<<"Started spinning with success"<<endl;
    ros::spin();
}

void Alfa_Pc_Decompress::sub_topic()
{
    //sub_ = nh.subscribe("point_cloud/compressed", 1, &Alfa_Pc_Decompress::cloud_cb,this);
}

void Alfa_Pc_Decompress::pub_()
{
    //pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud/decompressed", 1);
}

void Alfa_Pc_Decompress::exe_time()
{
    tempos_test = tempos_test/100 ;
    size_compressed_test = size_compressed_test/100;
    size_original_test = (size_original_test)/100;
    //std::ofstream myFile("./output/exe_time");
    //myFile<< "Exe. Time: "<< tempos_test << std::endl << "Point Cloud Size: "<< size_original_test << std::endl << "Compressed Size: "<<size_compressed_test<< std::endl << "Ratio: " << size_original_test/size_compressed_test << std::endl ;
    //myFile.close();
    ROS_INFO("-----------Acabei------------------------------------------------------------- \n");
    ROS_INFO("Time: %d\n", tempos_test);
    ROS_INFO("Point Cloud Size: %f\n", size_original_test);
    ROS_INFO("Ratio: %f\n", size_original_test/size_compressed_test);

    x=0;
    size_compressed_test = 0;
    size_original_test = 0;
}
