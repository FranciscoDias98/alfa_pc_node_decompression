#include "alfa_node.h"
#include <thread>
#include <unistd.h>
#include <chrono>


AlfaNode::AlfaNode()
{

    subscribe_topics();
    alive_ticker = new boost::thread(&AlfaNode::ticker_thread,this);
    pcloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

}

void AlfaNode::publish_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud)
{
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*output_cloud,pcl2_frame);
    pcl2_frame.header.frame_id = "decompressed";
    cloud_publisher.publish(pcl2_frame);
    cout << "Published decompressed point cloud"<<endl;

}

void AlfaNode::publish_metrics(alfa_msg::AlfaMetrics &metrics)
{
    filter_metrics.publish(metrics);

}

void AlfaNode::process_pointcloud(compressed_pointcloud_transport::CompressedPointCloud input)
{
    cout << "Please implement the process_pointcloud function"<<endl;
}

alfa_msg::AlfaConfigure::Response AlfaNode::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    cout << "Please implement the process_config function"<<endl;

}

AlfaNode::~AlfaNode()
{

}

void AlfaNode::cloud_cb(const compressed_pointcloud_transport::CompressedPointCloud::ConstPtr &input)
{
    std::cout << "entrei na callback da descompressao" << std::endl;
    std::cout << input->header << std::endl;
    std::stringstream compressed_data(input->data);


    compressed_pointcloud_transport::CompressedPointCloud input_compressed;
    input_compressed.data = input->data;
    input_compressed.header = input->header;
    //input_compressed.header.seq +=586;
    process_pointcloud(input_compressed);

}

bool AlfaNode::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    cout<<"Recieved FilterSettings with size" <<req.configurations.size()<<"... Updating"<<endl;
    for (int i=0; i< req.configurations.size();i++) {
        cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
    }

    res = process_config(req);
    return true;
}

void AlfaNode::init()
{
        char arg0[]= "filter_node";
        char *argv[]={arg0,NULL};
        int argc=(int)(sizeof(argv) / sizeof(char*)) - 1;;
        ros::init (argc, argv, NODE_NAME);
          if (!ros::master::check()) {
              cout <<"Failed to inicialize ros"<<endl;
            return;
          }

}

void AlfaNode::subscribe_topics()
{
    sub_cloud = nh.subscribe("point_cloud/compressed",1,&AlfaNode::cloud_cb,this);
    sub_parameters = nh.advertiseService(string(NODE_NAME).append("_settings"),&AlfaNode::parameters_cb,this);
    ros::NodeHandle n;
    filter_metrics = n.advertise<alfa_msg::AlfaMetrics>(string(NODE_NAME).append("_metrics"), 1);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>(string(NODE_NAME).append("_alive"),1);
    cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("point_cloud/decompressed",1);
    m_spin_thread = new boost::thread(&AlfaNode::spin, this);


}

void AlfaNode::ticker_thread()
{
    while(ros::ok())
    {
        alfa_msg::AlfaAlivePing newPing;
        newPing.node_name= NODE_NAME;
        newPing.node_type = NODE_TYPE;
        newPing.config_service_name = string(NODE_NAME)+"_settings";
        newPing.config_tag = "DIOR software example";
        alfa_msg::ConfigMessage parameter1,parameter2,parameter3,parameter4,parameter5,parameter6;

        parameter1.config = 7;
        parameter1.config_name = "Filter Selector";

        parameter2.config = 0.1;
        parameter2.config_name = "Minimal Search Radius:";

        parameter3.config = 0.2;
        parameter3.config_name = "Multiplication Parameter:";

        parameter4.config = 5;
        parameter4.config_name = "Neighbor Threshold";

        parameter5.config = 0.005;
        parameter5.config_name = "Intensity Treshold Parameter:";

        parameter6.config = 4;
        parameter6.config_name = "Multithreading: Number of threads";

        newPing.default_configurations.push_back(parameter1);
        newPing.default_configurations.push_back(parameter2);
        newPing.default_configurations.push_back(parameter3);
        newPing.default_configurations.push_back(parameter4);
        newPing.default_configurations.push_back(parameter5);
        newPing.default_configurations.push_back(parameter6);

        newPing.current_status = node_status;
        alive_publisher.publish(newPing);
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMER_SLEEP));
    }
}

void AlfaNode::spin()
{
    cout<<"started spinning with success"<<endl;
    ros::spin();
}
