#include "alfa_interface.h"
#include <thread>
#include <unistd.h>
#include <chrono>


AlfaInterface::AlfaInterface()
{
    node = new Alfa_Pd();
    subscrive_topics();
    alive_ticker = new boost::thread(&AlfaInterface::ticker_thread,this);
    pcloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

}

void AlfaInterface::publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud)
{
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*output_cloud,pcl2_frame);
    cloud_publisher.publish(pcl2_frame);
}

void AlfaInterface::publish_metrics(alfa_msg::AlfaMetrics &metrics)
{
    filter_metrics.publish(metrics);

}

void AlfaInterface::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    //cout<<"Recieved pointcloud"<<endl;
    if ((cloud->width * cloud->height) == 0)
    {
        cout <<"Recieved empty point cloud"<<endl;
        return;
    }
    cout<<"Recieved cloud"<<endl;
    pcl::fromROSMsg(*cloud,*pcloud);


    publish_pointcloud(node->apply_filter(pcloud));
    cout<< "Sent point cloud"<<endl;
    publish_metrics(node->outputMetrics);
    cout << "Sent metrics"<<endl;

}

bool AlfaInterface::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    cout<<"Recieved FilterSettings with size" <<req.configurations.size()<<"... Updating"<<endl;
    for (int i=0; i< req.configurations.size();i++) {
        cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
    }
    node->update_filterSettings(req);
    res.return_status=1;
    return true;

}

void AlfaInterface::init()
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

void AlfaInterface::subscrive_topics()
{
    sub_cloud = nh.subscribe("alfa_pointcloud",0,&AlfaInterface::cloud_cb,this);
    sub_parameters = nh.advertiseService(string(NODE_NAME).append("_settings"),&AlfaInterface::parameters_cb,this);
    ros::NodeHandle n;
    filter_metrics = n.advertise<alfa_msg::AlfaMetrics>(string(NODE_NAME).append("_metrics"), 1);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>(string(NODE_NAME).append("_alive"),1);
    cloud_publisher = n.advertise<sensor_msgs::PointCloud2>(string(NODE_NAME).append("_cloud"),1);
    m_spin_thread = new boost::thread(&AlfaInterface::spin, this);


}

void AlfaInterface::ticker_thread()
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

        parameter5.config = 1;
        parameter5.config_name = "Intensity Treshold Parameter:";

        parameter6.config = 4;
        parameter6.config_name = "Multithreading: Number of threads";

        newPing.default_configurations.push_back(parameter1);
        newPing.default_configurations.push_back(parameter2);
        newPing.default_configurations.push_back(parameter3);
        newPing.default_configurations.push_back(parameter4);
        newPing.default_configurations.push_back(parameter5);
        newPing.default_configurations.push_back(parameter6);


        alive_publisher.publish(newPing);
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMER_SLEEP));
    }
}

void AlfaInterface::spin()
{
    cout<<"started spinning with success"<<endl;
    ros::spin();
}
