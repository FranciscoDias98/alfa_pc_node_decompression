#include "alfa_pd.h"
#include <chrono>
#include<time.h>

Alfa_Pd::Alfa_Pd()
{
    frame_id =0;
    filter_number = 1;
    parameter1 = 0;
    parameter2 = 0.1;
    parameter3 = 0.1;
    parameter4 = 0.1;
    parameter5 = 0.1;
    hardware_ready = 0;
    int fd;
    int ddr_size = DDR_SIZE;
    int ddr_base_ptr = DDR_BASE_PTR;
    int config_size = CONFIG_SIZE;
    int config_base_ptr = CONFIGS_BASE_PTR;
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
         ddr_pointer = (u64 *)mmap(NULL, ddr_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, ddr_base_ptr);
         configs_pointer = (uint32_t *)mmap(NULL, config_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, config_base_ptr);
         hardware_ready=1;
    }

    outputMetrics.message_tag = "Filter performance";

}

void Alfa_Pd::do_voxelfilter()
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud (inputCloud);
    vg.setLeafSize (parameter1,parameter2,parameter3);
    vg.filter (*outputCloud);
}


void Alfa_Pd::do_sorfilter()
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
       sor.setInputCloud (inputCloud);
       sor.setMeanK (parameter1);
       sor.setStddevMulThresh (parameter2);
       sor.filter (*outputCloud);
}

void Alfa_Pd::do_drorfilter()
{
    vector<pcl::PointXYZI> outliners;
    vector<pcl::PointXYZI> inliners;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

   inliners.clear();
   kdtree.setInputCloud(inputCloud);
   if(thread_list.size()>1)
   {
       for (int i =0;i < thread_list.size();i++)
       {
           thread_list[i]->join();
        }

       thread_list.clear();
   }
   if(use_multi == true)
   {
       thread_list.clear();
       cout << "Running multithreading with "<< number_threads<<endl;
       for (int i =0;i <= number_threads;i++)
       {
           thread_list.push_back(new boost::thread(&Alfa_Pd::run_worker, this,i));
       }
       for (int i =0;i <= number_threads;i++)
       {
           thread_list[i]->join();
       }

   }
   else {
       for (auto &point : *inputCloud)
       {
           filter_point(point);
       }
   }
   outputCloud->resize(inliners.size());
   int counter =0;
   for(auto& point: *outputCloud)
   {
       point = inliners[counter];
       counter++;
   }
}

void Alfa_Pd::do_rorfilter()
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud(inputCloud);
    ror.setRadiusSearch(parameter1);
    ror.setMinNeighborsInRadius (parameter2);
    ror.filter (*outputCloud);
}

void Alfa_Pd::do_fcsorfilter()
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    double NumberClusters = parameter1;
    double  ClusterLength = abs(maxPt.x -minPt.x)/NumberClusters;
    double ClusterWidth = abs(maxPt.y - minPt.y)/NumberClusters;
    double ClusterHeight = abs(maxPt.z - minPt.z)/NumberClusters;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud (inputCloud);
    voxel.setLeafSize (ClusterLength,ClusterWidth, ClusterHeight);
    pcl::PointCloud<pcl::PointXYZI>::Ptr betweenCloud( new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter (*betweenCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (betweenCloud);
    sor.setMeanK (parameter2);
    sor.setStddevMulThresh (parameter3);
    sor.filter (*outputCloud);
}

void Alfa_Pd::do_LIORfilter()
{
    vector<pcl::PointXYZI> outliners;
    vector<pcl::PointXYZI> inliners;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    number_threads = parameter4;

    if(thread_list.size()>1)
    {
        for (int i =0;i < thread_list.size();i++)
        {
            thread_list[i]->join();
         }

        thread_list.clear();
    }

    if (number_threads >1)
    {
        thread_list.clear();

        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Alfa_Pd::run_lior_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
         }
    }
    else {
        for (auto &point : *inputCloud)
        {
            double intensity_trehshold=parameter2;
            if(point._PointXYZI::intensity > intensity_trehshold)
            {
                inliners.push_back(point);
            }
            else
            {
                filter_pointROR(point);
            }
        }

    }

    outputCloud->resize(inliners.size());
    int counter = 0;
    for (auto &point : *outputCloud)
    {
        point = inliners[counter];
        counter++;
    }
}




void Alfa_Pd::do_DIORfilter()
{
    vector<pcl::PointXYZI> outliners;
    vector<pcl::PointXYZI> inliners;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
   kdtree.setInputCloud(inputCloud);
   number_threads = parameter5;
       if(thread_list.size()>1)
       {
           for (int i =0;i < thread_list.size();i++)
           {
               thread_list[i]->join();
            }

           thread_list.clear();
       }

       if (number_threads >1)
       {
           thread_list.clear();
           for (int i =0;i <= number_threads;i++)
           {
               thread_list.push_back(new boost::thread(&Alfa_Pd::run_dlior_worker, this,i));
           }
           for (int i =0;i <= number_threads;i++)
           {
               thread_list[i]->join();
            }
       }
       else {
           for (auto &point : *inputCloud)
           {
               double intensity_trehshold=parameter4;
               if(point._PointXYZI::intensity > intensity_trehshold)
               {
                   inliners.push_back(point);
               }
               else
               {
                   filter_point(point,1);
               }
           }

       }

       outputCloud->resize(inliners.size());
       int counter = 0;
       for (auto &point : *outputCloud)
       {
           point = inliners[counter];
           counter++;
       }

}

using namespace std::chrono;

void Alfa_Pd::do_hardwarefilter()
{
    auto start = high_resolution_clock::now();
    frame_id++;
    int intensity_mult;
    if(parameter1 !=1)intensity_mult = parameter5;

    configs_pointer[0]=0;
    uint32_t config = 2+ ((((uint)parameter1)<<2)+((uint)parameter3<<6) +((uint)parameter2<<10)+((uint)parameter4<<14)+((uint)parameter5<<23));
       //cout << "Storing points!"<<endl;
       //for (auto &point : *inputCloud)
   int32_t a_64points[2];
   for (int var = 0; var < inputCloud->size(); var++)
   {
       a_64points[0] = ((int16_t)((*inputCloud)[var].x*100))+(((int16_t)((*inputCloud)[var].y*100+1))<<16);
       //cout<<"Point.x: " <<hex<<(int16_t)((*inputCloud)[var].x*100)<<"point.y"<<(int16_t)((*inputCloud)[var].y*100)<<endl;
       a_64points[1]=((int16_t)((*inputCloud)[var].z*100))+(((int16_t)((*inputCloud)[var].intensity*intensity_mult+1))<<16);
       //cout<<"Point: "<<var<<" :"<<hex<<a_64points[1]<<" "<< hex << a_64points[0]<<endl;
       memcpy((void*)(ddr_pointer+var),a_64points,sizeof(int32_t)*2);
   }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    alfa_msg::MetricMessage newMessage;
    newMessage.metric = duration.count();
    newMessage.units = "ms";
    newMessage.metric_name = "Storing points";

    outputMetrics.metrics.push_back(newMessage);
    start = high_resolution_clock::now();
    configs_pointer[4]= frame_id;
    configs_pointer[2]= inputCloud->size();
    configs_pointer[0] = config;
    int hardware_finish =1;
    int value = 0;
    int hardware_frame_id=0;
    usleep(10);
    while (hardware_finish) {
           value = configs_pointer[1];
            if(value >=1)
            {
                bool frame_differ =1;
                while (frame_differ) {
                    hardware_frame_id = configs_pointer[5];
                    if(hardware_frame_id == frame_id)
                    {
                        frame_differ =0;
                    }
                }
                hardware_finish=0;
            }
            else
                 usleep(1);
    }

    configs_pointer[0] = 0;
    configs_pointer[1] = 0;
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    newMessage.metric = duration.count();
    newMessage.units = "ms";
    newMessage.metric_name = "Processing points";
    outputMetrics.metrics.push_back(newMessage);


    start = high_resolution_clock::now();
    decode_pointcloud();
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    newMessage.metric = duration.count();
    newMessage.units = "ms";
    newMessage.metric_name = "Reading and decoding point cloud";
    outputMetrics.metrics.push_back(newMessage);


}

void Alfa_Pd::apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    auto start = high_resolution_clock::now();

    switch (filter_number) {
       case 1:
           do_voxelfilter();
           break;
       case 2:
           do_rorfilter();
           break;
       case 3:
           do_sorfilter();
           break;
       case 4:
           do_drorfilter();
           break;
       case 5:
           do_fcsorfilter();
           break;
       case 6:
           do_LIORfilter();
           break;
       case 7:
           do_DIORfilter();
           break;
       case 8:
           do_hardwarefilter();
           break;
    }
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<milliseconds>(stop - start);
   alfa_msg::MetricMessage newMessage;
   newMessage.metric = duration.count();
   newMessage.units = "ms";
   newMessage.metric_name = "Total processing time";

}



void Alfa_Pd::decode_pointcloud()
{
    outputCloud->clear();

      for (int i = 0; i < inputCloud->size(); ++i) {
          if(ddr_pointer[i]!=0)
          {
              outputCloud->push_back((*inputCloud)[i]);
          }
      }
}


