 #include "ros/ros.h"
#include "alfa_interface.h"

int main(int argc, char **argv)
{

     ros::init (argc, argv, "alfa_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }

    AlfaInterface new_node;
    while(ros::ok())
    {

    }
}
