#include "ros/ros.h"
#include "decompressor.h"

int main(int argc, char **argv)
{

     ros::init (argc, argv, "alfa_pc_decompression_node");
       if (!ros::master::check()) {
           cout <<"Failed to inicialize ros"<<endl;
       }

    Alfa_Pc_Decompress new_node;
    while(ros::ok())
    {

    }
}
