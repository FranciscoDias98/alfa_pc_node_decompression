#include "decompressor.h"
#include <iostream>
#include <pcl/io/pcd_io.h>

unsigned int x = 0;
unsigned long tempos = 0;
unsigned long size_compressed =0;

unsigned long tempos_test = 0;
float size_compressed_test =0;
float size_original_test =0;

unsigned long points_test =0;

Alfa_Pc_Decompress::Alfa_Pc_Decompress()
{
    std::cout << "entrei no construtor" << std::endl;
    out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_0.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_3.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_4.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_5.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_6.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    octant_7.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    show_statistics = true;
    compression_profile = pcl::io::MANUAL_CONFIGURATION;
    point_cloud_decoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
    point_cloud_decoder_0 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_1 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_2 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_3 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_4 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_5 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_6 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);
    point_cloud_decoder_7 = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compression_profile,show_statistics);

    //sub_topic();
    //pub_();
    //spin();

    multi_thread = false;

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

    if(multi_thread){

        std::cout << "Entrei na Descompressao Multi" << std::endl;

        out_cloud->clear();
        octant_0->clear();
        octant_1->clear();
        octant_2->clear();
        octant_3->clear();
        octant_4->clear();
        octant_5->clear();
        octant_6->clear();
        octant_7->clear();

        std::stringstream compressed_data(input.data);

        point_cloud_decoder_0->syncToHeader(compressed_data);
        point_cloud_decoder_0->switchBuffers();
        point_cloud_decoder_0->setOutputCloud(octant_0);


//        //point_cloud_decoder_1->syncToHeader(compressed_data);
          point_cloud_decoder_1->switchBuffers();
          point_cloud_decoder_1->setOutputCloud(octant_1);


//        //point_cloud_decoder_2->syncToHeader(compressed_data);
        point_cloud_decoder_2->switchBuffers();
        point_cloud_decoder_2->setOutputCloud(octant_2);

//        //point_cloud_decoder_3->syncToHeader(compressed_data);
        point_cloud_decoder_3->switchBuffers();
        point_cloud_decoder_3->setOutputCloud(octant_3);

//        //point_cloud_decoder_4->syncToHeader(compressed_data);
        point_cloud_decoder_4->switchBuffers();
        point_cloud_decoder_4->setOutputCloud(octant_4);

//        //point_cloud_decoder_5->syncToHeader(compressed_data);
        point_cloud_decoder_5->switchBuffers();
        point_cloud_decoder_5->setOutputCloud(octant_5);

//        //point_cloud_decoder_6->syncToHeader(compressed_data);
        point_cloud_decoder_6->switchBuffers();
        point_cloud_decoder_6->setOutputCloud(octant_6);

//        //point_cloud_decoder_7->syncToHeader(compressed_data);
        point_cloud_decoder_7->switchBuffers();
        point_cloud_decoder_7->setOutputCloud(octant_7);

        std::cout << "Fiz Initialization dos Decoders " << std::endl;

        my_read_frame_header(compressed_data);

        std::cout << "Acabei Read Frame Header " << std::endl;

        point_cloud_decoder_0->entropyDecoding(compressed_data);

//        for(int i=0; i<point_cloud_decoder_0->binary_tree_data_vector_.size();i++)
//            printf("%i --> %x \n",i,point_cloud_decoder_0->binary_tree_data_vector_[i]);






        std::cout << "Acabei Entropy Decoding " << std::endl;

        point_cloud_decoder_0->point_coder_.initializeDecoding();
        point_cloud_decoder_1->point_coder_.initializeDecoding();
        point_cloud_decoder_2->point_coder_.initializeDecoding();
        point_cloud_decoder_3->point_coder_.initializeDecoding();
        point_cloud_decoder_4->point_coder_.initializeDecoding();
        point_cloud_decoder_5->point_coder_.initializeDecoding();
        point_cloud_decoder_6->point_coder_.initializeDecoding();
        point_cloud_decoder_7->point_coder_.initializeDecoding();

        std::cout << "Fiz Initialization Decoding " << std::endl;

        point_cloud_decoder_0->output_->points.clear();
        point_cloud_decoder_1->output_->points.clear();
        point_cloud_decoder_2->output_->points.clear();
        point_cloud_decoder_3->output_->points.clear();
        point_cloud_decoder_4->output_->points.clear();
        point_cloud_decoder_5->output_->points.clear();
        point_cloud_decoder_6->output_->points.clear();
        point_cloud_decoder_7->output_->points.clear();

        std::cout << "Acabei Point Clear " << std::endl;

        point_cloud_decoder_0->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_0->point_count_));
        point_cloud_decoder_1->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_1->point_count_));
        point_cloud_decoder_2->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_2->point_count_));
        point_cloud_decoder_3->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_3->point_count_));
        point_cloud_decoder_4->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_4->point_count_));
        point_cloud_decoder_5->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_5->point_count_));
        point_cloud_decoder_6->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_6->point_count_));
        point_cloud_decoder_7->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_7->point_count_));

        std::cout << "Acabei Point Reserve " << std::endl;

        std::vector<char>vec_0 = {point_cloud_decoder_0->binary_tree_data_vector_.begin(), point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0};
        std::vector<char>vec_1 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 , point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1};
        std::vector<char>vec_2 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2};
        std::vector<char>vec_3 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3};
        std::vector<char>vec_4 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4};
        std::vector<char>vec_5 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5};
        std::vector<char>vec_6 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5 + size_6};
        std::vector<char>vec_7 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5 + size_6, point_cloud_decoder_0->binary_tree_data_vector_.end()};

//        for(int i=0; i<vec_0.size();i++)
//            printf("%i --> %x \n",i,vec_0[i]);

//        for(int i=0; i<vec_1.size();i++)
//            printf("%i --> %x \n",i,vec_1[i]);

        std::cout << "Acabei Vector Division " << std::endl;

        if(point_cloud_decoder_0->i_frame_){
           point_cloud_decoder_0->deserializeTree(vec_0,false);
           point_cloud_decoder_1->deserializeTree(vec_1,false);
           point_cloud_decoder_2->deserializeTree(vec_2,false);
           point_cloud_decoder_3->deserializeTree(vec_3,false);
           point_cloud_decoder_4->deserializeTree(vec_4,false);
           point_cloud_decoder_5->deserializeTree(vec_5,false);
           point_cloud_decoder_6->deserializeTree(vec_6,false);
           point_cloud_decoder_7->deserializeTree(vec_7,false);
        }else{
           point_cloud_decoder_0->deserializeTree(vec_0,true);
           point_cloud_decoder_1->deserializeTree(vec_1,true);
           point_cloud_decoder_2->deserializeTree(vec_2,true);
           point_cloud_decoder_3->deserializeTree(vec_3,true);
           point_cloud_decoder_4->deserializeTree(vec_4,true);
           point_cloud_decoder_5->deserializeTree(vec_5,true);
           point_cloud_decoder_6->deserializeTree(vec_6,true);
           point_cloud_decoder_7->deserializeTree(vec_7,true);
        }

         std::cout << "Acabei Desirialzation " << std::endl;
         //point_cloud_decoder_1->deserializeTree(vec_1,false);



        point_cloud_decoder_0->output_->height = 1;
        point_cloud_decoder_0->output_->width = static_cast<uint32_t> (octant_0->points.size ());
        point_cloud_decoder_0->output_->is_dense = false;

        point_cloud_decoder_1->output_->height = 1;
        point_cloud_decoder_1->output_->width = static_cast<uint32_t> (octant_1->points.size ());
        point_cloud_decoder_1->output_->is_dense = false;

        point_cloud_decoder_2->output_->height = 1;
        point_cloud_decoder_2->output_->width = static_cast<uint32_t> (octant_2->points.size ());
        point_cloud_decoder_2->output_->is_dense = false;

        point_cloud_decoder_3->output_->height = 1;
        point_cloud_decoder_3->output_->width = static_cast<uint32_t> (octant_3->points.size ());
        point_cloud_decoder_3->output_->is_dense = false;

        point_cloud_decoder_4->output_->height = 1;
        point_cloud_decoder_4->output_->width = static_cast<uint32_t> (octant_4->points.size ());
        point_cloud_decoder_4->output_->is_dense = false;

        point_cloud_decoder_5->output_->height = 1;
        point_cloud_decoder_5->output_->width = static_cast<uint32_t> (octant_5->points.size ());
        point_cloud_decoder_5->output_->is_dense = false;

        point_cloud_decoder_6->output_->height = 1;
        point_cloud_decoder_6->output_->width = static_cast<uint32_t> (octant_6->points.size ());
        point_cloud_decoder_6->output_->is_dense = false;

        point_cloud_decoder_7->output_->height = 1;
        point_cloud_decoder_7->output_->width = static_cast<uint32_t> (octant_7->points.size ());
        point_cloud_decoder_7->output_->is_dense = false;

        std::cout << "Octant 0 Size: " << octant_0->points.size() << std::endl;
        std::cout << "Octant 1 Size: " << octant_1->points.size() << std::endl;
        std::cout << "Octant 2 Size: " << octant_2->points.size() << std::endl;
        std::cout << "Octant 3 Size: " << octant_3->points.size() << std::endl;
        std::cout << "Octant 4 Size: " << octant_4->points.size() << std::endl;
        std::cout << "Octant 5 Size: " << octant_5->points.size() << std::endl;
        std::cout << "Octant 6 Size: " << octant_6->points.size() << std::endl;
        std::cout << "Octant 7 Size: " << octant_7->points.size() << std::endl;





        for (auto &point: *octant_0) {
            point._PointXYZRGB::r = 255; //red
            point._PointXYZRGB::g = 0;
            point._PointXYZRGB::b = 0;
            out_cloud->push_back(point);
        }
        for (auto &point: *octant_1) {
            point._PointXYZRGB::r = 0;
            point._PointXYZRGB::b = 0;
            point._PointXYZRGB::g = 255; //lime
            out_cloud->push_back(point);
        }
        for (auto &point: *octant_2) {
            point._PointXYZRGB::b = 255; // blue
            point._PointXYZRGB::r = 0;
            point._PointXYZRGB::g = 0;
            out_cloud->push_back(point);
        }
        for (auto &point: *octant_3) {
            point._PointXYZRGB::r = 255; // yellow
            point._PointXYZRGB::g = 255;
            point._PointXYZRGB::b = 0;
            out_cloud->push_back(point);
        }
        for (auto &point: *octant_4) {
            point._PointXYZRGB::g = 255; // cyan
            point._PointXYZRGB::b = 255;
            point._PointXYZRGB::r = 0;
            out_cloud->push_back(point);
        }
        for (auto &point: *octant_5) {
            point._PointXYZRGB::r = 255; // magenta
            point._PointXYZRGB::b = 255;
            point._PointXYZRGB::g = 0;

            out_cloud->push_back(point);
        }
        for (auto &point: *octant_6) {
            point._PointXYZRGB::r = 255; // white
            point._PointXYZRGB::g = 255;
            point._PointXYZRGB::b = 255;
            out_cloud->push_back(point);
        }
        for (auto &point: *octant_7) {
            point._PointXYZRGB::r = 255; // orange
            point._PointXYZRGB::g = 165;
            point._PointXYZRGB::b = 0;
            out_cloud->push_back(point);
        }


//        for (auto &point: *out_cloud) {
//            point._PointXYZRGB::r = 255; // orange
//            point._PointXYZRGB::g = 165;
//        }


        printf("Point Cloud Reconstructed Size: %d \n",out_cloud->size());

        pcl::io::savePCDFileASCII("compressed_color_test.pcd", *out_cloud);
        publish_pointcloud(out_cloud);



    }else {

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
        point_cloud_decoder->syncToHeader(compressed_data);
        point_cloud_decoder->switchBuffers();
        point_cloud_decoder->setOutputCloud(out_cloud);
        point_cloud_decoder->cloud_with_color_ = false;
        point_cloud_decoder->deleteTree();
        point_cloud_decoder->readFrameHeader(compressed_data);
        printf("Octree Resolution: %f\n",point_cloud_decoder->getResolution());
        //printf("Point Resolution: %f\n",point_cloud_decoder->point_resolution_);
        point_cloud_decoder->entropyDecoding(compressed_data);
        // initialize color and point encoding
        printf("Binary Tree Size: %d\n",point_cloud_decoder->binary_tree_data_vector_.size());
        printf("Point Count %d \n",point_cloud_decoder->point_count_);
        printf("Leaf Count %d \n",point_cloud_decoder->getLeafCount());
        printf("Object Count %d \n",point_cloud_decoder->object_count_);


        point_cloud_decoder->color_coder_.initializeDecoding ();
        point_cloud_decoder->point_coder_.initializeDecoding ();

        // initialize output cloud
        point_cloud_decoder->output_->points.clear ();
        point_cloud_decoder->output_->points.reserve (static_cast<std::size_t> (point_cloud_decoder->point_count_));
        point_cloud_decoder->defineBoundingBox(-256,-256,-256,256,256,256);

        //make swicth to define tree depth
        double res = point_cloud_decoder->getResolution();
        point_cloud_decoder->setResolution(res);

        if(res == 0.03){
            point_cloud_decoder->setTreeDepth(15);
        } else if(res == 0.06){
            point_cloud_decoder->setTreeDepth(14);
        } else if(res == 0.125){
            point_cloud_decoder->setTreeDepth(13);
        } else if(res == 0.25){
            point_cloud_decoder->setTreeDepth(11);
        }else if(res == 0.5){
            point_cloud_decoder->setTreeDepth(10);
        }else if(res == 1){
            point_cloud_decoder->setTreeDepth(9);
        }





        //point_cloud_decoder->setTreeDepth(13);
        point_cloud_decoder->deserializeTree(point_cloud_decoder->binary_tree_data_vector_,false);


        point_cloud_decoder->output_->height = 1;
        point_cloud_decoder->output_->width = static_cast<uint32_t>(out_cloud->points.size ());
        point_cloud_decoder->output_->is_dense = false;


        //point_cloud_decoder->setTreeDepth(13);
        //point_cloud_decoder->decodePointCloud_2(compressed_data,out_cloud);


        std::cout << "Fiz Descompressao  " << out_cloud->size() << std::endl;
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);

        size_original_test = size_original_test + (static_cast<float> (out_cloud->points.size()) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f)*1000;
        points_test = points_test + out_cloud->points.size();
        compressed_data.seekg(0,ios::end);
        size_compressed_test = size_compressed_test+compressed_data.tellg();
        tempos_test = tempos_test + duration.count();

        x++;

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
        printf("Point Cloud Reconstructed Size: %d \n",out_cloud->size());

        publish_pointcloud(out_cloud);
        publish_metrics(output_metrics);
    }
}


void Alfa_Pc_Decompress::my_read_frame_header(std::istream &compressed_data)
{

    compressed_data.read (reinterpret_cast<char*> (&point_cloud_decoder_0->frame_ID_), sizeof (point_cloud_decoder_0->frame_ID_));
    compressed_data.read (reinterpret_cast<char*>(&point_cloud_decoder_0->i_frame_), sizeof (point_cloud_decoder_0->i_frame_));

    if(point_cloud_decoder_0->i_frame_)
    {
        double octree_resolution;
        unsigned char color_bit_depth;
        double point_resolution;

        double min_x_0, min_y_0, min_z_0, max_x_0, max_y_0, max_z_0;
        double min_x_1, min_y_1, min_z_1, max_x_1, max_y_1, max_z_1;
        double min_x_2, min_y_2, min_z_2, max_x_2, max_y_2, max_z_2;
        double min_x_3, min_y_3, min_z_3, max_x_3, max_y_3, max_z_3;
        double min_x_4, min_y_4, min_z_4, max_x_4, max_y_4, max_z_4;
        double min_x_5, min_y_5, min_z_5, max_x_5, max_y_5, max_z_5;
        double min_x_6, min_y_6, min_z_6, max_x_6, max_y_6, max_z_6;
        double min_x_7, min_y_7, min_z_7, max_x_7, max_y_7, max_z_7;






        // read coder configuration
        compressed_data.read (reinterpret_cast<char*> (&point_cloud_decoder_0->do_voxel_grid_enDecoding_), sizeof (point_cloud_decoder_0->do_voxel_grid_enDecoding_));
        compressed_data.read (reinterpret_cast<char*> (&point_cloud_decoder_0->data_with_color_), sizeof (point_cloud_decoder_0->data_with_color_));
        compressed_data.read (reinterpret_cast<char*> (&point_count_0), sizeof (point_count_0));
        compressed_data.read (reinterpret_cast<char*> (&octree_resolution), sizeof (octree_resolution));
        compressed_data.read (reinterpret_cast<char*> (&color_bit_depth), sizeof (color_bit_depth));
        compressed_data.read (reinterpret_cast<char*> (&point_resolution), sizeof (point_resolution));

        // read octree bounding box_0
        compressed_data.read (reinterpret_cast<char*> (&min_x_0), sizeof (min_x_0));
        compressed_data.read (reinterpret_cast<char*> (&min_y_0), sizeof (min_y_0));
        compressed_data.read (reinterpret_cast<char*> (&min_z_0), sizeof (min_z_0));
        compressed_data.read (reinterpret_cast<char*> (&max_x_0), sizeof (max_x_0));
        compressed_data.read (reinterpret_cast<char*> (&max_y_0), sizeof (max_y_0));
        compressed_data.read (reinterpret_cast<char*> (&max_z_0), sizeof (max_z_0));

        // read octree bounding box_1
        compressed_data.read (reinterpret_cast<char*> (&point_count_1), sizeof (point_count_1));
        compressed_data.read (reinterpret_cast<char*> (&min_x_1), sizeof (min_x_1));
        compressed_data.read (reinterpret_cast<char*> (&min_y_1), sizeof (min_y_1));
        compressed_data.read (reinterpret_cast<char*> (&min_z_1), sizeof (min_z_1));
        compressed_data.read (reinterpret_cast<char*> (&max_x_1), sizeof (max_x_1));
        compressed_data.read (reinterpret_cast<char*> (&max_y_1), sizeof (max_y_1));
        compressed_data.read (reinterpret_cast<char*> (&max_z_1), sizeof (max_z_1));

        // read octree bounding box_2
        compressed_data.read (reinterpret_cast<char*> (&point_count_2), sizeof (point_count_2));
        compressed_data.read (reinterpret_cast<char*> (&min_x_2), sizeof (min_x_2));
        compressed_data.read (reinterpret_cast<char*> (&min_y_2), sizeof (min_y_2));
        compressed_data.read (reinterpret_cast<char*> (&min_z_2), sizeof (min_z_2));
        compressed_data.read (reinterpret_cast<char*> (&max_x_2), sizeof (max_x_2));
        compressed_data.read (reinterpret_cast<char*> (&max_y_2), sizeof (max_y_2));
        compressed_data.read (reinterpret_cast<char*> (&max_z_2), sizeof (max_z_2));

        // read octree bounding box_3
        compressed_data.read (reinterpret_cast<char*> (&point_count_3), sizeof (point_count_3));
        compressed_data.read (reinterpret_cast<char*> (&min_x_3), sizeof (min_x_3));
        compressed_data.read (reinterpret_cast<char*> (&min_y_3), sizeof (min_y_3));
        compressed_data.read (reinterpret_cast<char*> (&min_z_3), sizeof (min_z_3));
        compressed_data.read (reinterpret_cast<char*> (&max_x_3), sizeof (max_x_3));
        compressed_data.read (reinterpret_cast<char*> (&max_y_3), sizeof (max_y_3));
        compressed_data.read (reinterpret_cast<char*> (&max_z_3), sizeof (max_z_3));

        // read octree bounding box_4
        compressed_data.read (reinterpret_cast<char*> (&point_count_4), sizeof (point_count_4));
        compressed_data.read (reinterpret_cast<char*> (&min_x_4), sizeof (min_x_4));
        compressed_data.read (reinterpret_cast<char*> (&min_y_4), sizeof (min_y_4));
        compressed_data.read (reinterpret_cast<char*> (&min_z_4), sizeof (min_z_4));
        compressed_data.read (reinterpret_cast<char*> (&max_x_4), sizeof (max_x_4));
        compressed_data.read (reinterpret_cast<char*> (&max_y_4), sizeof (max_y_4));
        compressed_data.read (reinterpret_cast<char*> (&max_z_4), sizeof (max_z_4));

        // read octree bounding box_5
        compressed_data.read (reinterpret_cast<char*> (&point_count_5), sizeof (point_count_5));
        compressed_data.read (reinterpret_cast<char*> (&min_x_5), sizeof (min_x_5));
        compressed_data.read (reinterpret_cast<char*> (&min_y_5), sizeof (min_y_5));
        compressed_data.read (reinterpret_cast<char*> (&min_z_5), sizeof (min_z_5));
        compressed_data.read (reinterpret_cast<char*> (&max_x_5), sizeof (max_x_5));
        compressed_data.read (reinterpret_cast<char*> (&max_y_5), sizeof (max_y_5));
        compressed_data.read (reinterpret_cast<char*> (&max_z_5), sizeof (max_z_5));

        // read octree bounding box_6
        compressed_data.read (reinterpret_cast<char*> (&point_count_6), sizeof (point_count_6));
        compressed_data.read (reinterpret_cast<char*> (&min_x_6), sizeof (min_x_6));
        compressed_data.read (reinterpret_cast<char*> (&min_y_6), sizeof (min_y_6));
        compressed_data.read (reinterpret_cast<char*> (&min_z_6), sizeof (min_z_6));
        compressed_data.read (reinterpret_cast<char*> (&max_x_6), sizeof (max_x_6));
        compressed_data.read (reinterpret_cast<char*> (&max_y_6), sizeof (max_y_6));
        compressed_data.read (reinterpret_cast<char*> (&max_z_6), sizeof (max_z_6));

        // read octree bounding box_7
        compressed_data.read (reinterpret_cast<char*> (&point_count_7), sizeof (point_count_7));
        compressed_data.read (reinterpret_cast<char*> (&min_x_7), sizeof (min_x_7));
        compressed_data.read (reinterpret_cast<char*> (&min_y_7), sizeof (min_y_7));
        compressed_data.read (reinterpret_cast<char*> (&min_z_7), sizeof (min_z_7));
        compressed_data.read (reinterpret_cast<char*> (&max_x_7), sizeof (max_x_7));
        compressed_data.read (reinterpret_cast<char*> (&max_y_7), sizeof (max_y_7));
        compressed_data.read (reinterpret_cast<char*> (&max_z_7), sizeof (max_z_7));


        compressed_data.read (reinterpret_cast<char*> (&size_0), sizeof (size_0));
        compressed_data.read (reinterpret_cast<char*> (&size_1), sizeof (size_1));
        compressed_data.read (reinterpret_cast<char*> (&size_2), sizeof (size_2));
        compressed_data.read (reinterpret_cast<char*> (&size_3), sizeof (size_3));
        compressed_data.read (reinterpret_cast<char*> (&size_4), sizeof (size_4));
        compressed_data.read (reinterpret_cast<char*> (&size_5), sizeof (size_5));
        compressed_data.read (reinterpret_cast<char*> (&size_6), sizeof (size_6));
        compressed_data.read (reinterpret_cast<char*> (&size_7), sizeof (size_7));


        std::cout << "Point Count Octant 0 " << point_count_0 << std::endl;
        std::cout << "Point Count Octant 1 " << point_count_1 << std::endl;
        std::cout << "Point Count Octant 2 " << point_count_2 << std::endl;
        std::cout << "Point Count Octant 3 " << point_count_3 << std::endl;
        std::cout << "Point Count Octant 4 " << point_count_4 << std::endl;
        std::cout << "Point Count Octant 5 " << point_count_5 << std::endl;
        std::cout << "Point Count Octant 6 " << point_count_6 << std::endl;
        std::cout << "Point Count Octant 7 " << point_count_7 << std::endl;



        point_cloud_decoder_1->frame_ID_ = point_cloud_decoder_0->frame_ID_;
        point_cloud_decoder_2->frame_ID_ = point_cloud_decoder_0->frame_ID_;
        point_cloud_decoder_3->frame_ID_ = point_cloud_decoder_0->frame_ID_;
        point_cloud_decoder_4->frame_ID_ = point_cloud_decoder_0->frame_ID_;
        point_cloud_decoder_5->frame_ID_ = point_cloud_decoder_0->frame_ID_;
        point_cloud_decoder_6->frame_ID_ = point_cloud_decoder_0->frame_ID_;
        point_cloud_decoder_7->frame_ID_ = point_cloud_decoder_0->frame_ID_;

        point_cloud_decoder_1->i_frame_ = point_cloud_decoder_0->i_frame_;
        point_cloud_decoder_2->i_frame_ = point_cloud_decoder_0->i_frame_;
        point_cloud_decoder_3->i_frame_ = point_cloud_decoder_0->i_frame_;
        point_cloud_decoder_4->i_frame_ = point_cloud_decoder_0->i_frame_;
        point_cloud_decoder_5->i_frame_ = point_cloud_decoder_0->i_frame_;
        point_cloud_decoder_6->i_frame_ = point_cloud_decoder_0->i_frame_;
        point_cloud_decoder_7->i_frame_ = point_cloud_decoder_0->i_frame_;


        point_cloud_decoder_1->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;
        point_cloud_decoder_2->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;
        point_cloud_decoder_3->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;
        point_cloud_decoder_4->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;
        point_cloud_decoder_5->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;
        point_cloud_decoder_6->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;
        point_cloud_decoder_7->do_voxel_grid_enDecoding_ = point_cloud_decoder_0->do_voxel_grid_enDecoding_;


        point_cloud_decoder_1->data_with_color_ = point_cloud_decoder_0->data_with_color_;
        point_cloud_decoder_2->data_with_color_ = point_cloud_decoder_0->data_with_color_;
        point_cloud_decoder_3->data_with_color_ = point_cloud_decoder_0->data_with_color_;
        point_cloud_decoder_4->data_with_color_ = point_cloud_decoder_0->data_with_color_;
        point_cloud_decoder_5->data_with_color_ = point_cloud_decoder_0->data_with_color_;
        point_cloud_decoder_6->data_with_color_ = point_cloud_decoder_0->data_with_color_;
        point_cloud_decoder_7->data_with_color_ = point_cloud_decoder_0->data_with_color_;



        //
        point_cloud_decoder_0->point_count_ = point_count_0;
        point_cloud_decoder_1->point_count_ = point_count_1;
        point_cloud_decoder_2->point_count_ = point_count_2;
        point_cloud_decoder_3->point_count_ = point_count_3;
        point_cloud_decoder_4->point_count_ = point_count_4;
        point_cloud_decoder_5->point_count_ = point_count_5;
        point_cloud_decoder_6->point_count_ = point_count_6;
        point_cloud_decoder_7->point_count_ = point_count_7;



        //delete tree
        point_cloud_decoder_0->deleteTree();
        point_cloud_decoder_1->deleteTree();
        point_cloud_decoder_2->deleteTree();
        point_cloud_decoder_3->deleteTree();
        point_cloud_decoder_4->deleteTree();
        point_cloud_decoder_5->deleteTree();
        point_cloud_decoder_6->deleteTree();
        point_cloud_decoder_7->deleteTree();




        //set resolution
        point_cloud_decoder_0->setResolution (octree_resolution);
        point_cloud_decoder_1->setResolution (octree_resolution);
        point_cloud_decoder_2->setResolution (octree_resolution);
        point_cloud_decoder_3->setResolution (octree_resolution);
        point_cloud_decoder_4->setResolution (octree_resolution);
        point_cloud_decoder_5->setResolution (octree_resolution);
        point_cloud_decoder_6->setResolution (octree_resolution);
        point_cloud_decoder_7->setResolution (octree_resolution);


        point_cloud_decoder_0->defineBoundingBox(min_x_0,min_y_0,min_z_0,max_x_0,max_y_0,max_z_0);
        point_cloud_decoder_1->defineBoundingBox(min_x_1,min_y_1,min_z_1,max_x_1,max_y_1,max_z_1);
        point_cloud_decoder_2->defineBoundingBox(min_x_2,min_y_2,min_z_2,max_x_2,max_y_2,max_z_2);
        point_cloud_decoder_3->defineBoundingBox(min_x_3,min_y_3,min_z_3,max_x_3,max_y_3,max_z_3);
        point_cloud_decoder_4->defineBoundingBox(min_x_4,min_y_4,min_z_4,max_x_4,max_y_4,max_z_4);
        point_cloud_decoder_5->defineBoundingBox(min_x_5,min_y_5,min_z_5,max_x_5,max_y_5,max_z_5);
        point_cloud_decoder_6->defineBoundingBox(min_x_6,min_y_6,min_z_6,max_x_6,max_y_6,max_z_6);
        point_cloud_decoder_7->defineBoundingBox(min_x_7,min_y_7,min_z_7,max_x_7,max_y_7,max_z_7);


        point_cloud_decoder_0->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_1->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_2->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_3->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_4->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_5->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_6->color_coder_.setBitDepth(color_bit_depth);
        point_cloud_decoder_7->color_coder_.setBitDepth(color_bit_depth);


        point_cloud_decoder_0->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_1->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_2->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_3->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_4->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_5->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_6->point_coder_.setPrecision(static_cast<float> (point_resolution));
        point_cloud_decoder_7->point_coder_.setPrecision(static_cast<float> (point_resolution));





    }
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
    points_test = points_test/100;
    //std::ofstream myFile("./output/exe_time");
    //myFile<< "Exe. Time: "<< tempos_test << std::endl << "Point Cloud Size: "<< size_original_test << std::endl << "Compressed Size: "<<size_compressed_test<< std::endl << "Ratio: " << size_original_test/size_compressed_test << std::endl ;
    //myFile.close();
    ROS_INFO("-----------Acabei------------------------------------------------------------- \n");
    ROS_INFO("Time: %d\n", tempos_test);
    ROS_INFO("Point Cloud Size: %f\n", size_original_test);
    ROS_INFO("Ratio: %f\n", size_original_test/size_compressed_test);
    ROS_INFO("Points: %d\n", points_test);
    x=0;
    size_compressed_test = 0;
    size_original_test = 0;
    points_test = 0;
}



////////////////////////////////////////////
/*
std::cout << "Entrei na Descompressao Multi" << std::endl;

out_cloud->clear();
octant_0->clear();
octant_1->clear();
octant_2->clear();
octant_3->clear();
octant_4->clear();
octant_5->clear();
octant_6->clear();
octant_7->clear();

std::stringstream compressed_data(input.data);

point_cloud_decoder_0->syncToHeader(compressed_data);
point_cloud_decoder_0->switchBuffers();
point_cloud_decoder_0->setOutputCloud(octant_0);


//        //point_cloud_decoder_1->syncToHeader(compressed_data);
//        point_cloud_decoder_1->switchBuffers();
//        point_cloud_decoder_1->setOutputCloud(octant_1);


//        //point_cloud_decoder_2->syncToHeader(compressed_data);
//        point_cloud_decoder_2->switchBuffers();
//        point_cloud_decoder_2->setOutputCloud(octant_2);

//        //point_cloud_decoder_3->syncToHeader(compressed_data);
//        point_cloud_decoder_3->switchBuffers();
//        point_cloud_decoder_3->setOutputCloud(octant_3);

//        //point_cloud_decoder_4->syncToHeader(compressed_data);
//        point_cloud_decoder_4->switchBuffers();
//        point_cloud_decoder_4->setOutputCloud(octant_4);

//        //point_cloud_decoder_5->syncToHeader(compressed_data);
//        point_cloud_decoder_5->switchBuffers();
//        point_cloud_decoder_5->setOutputCloud(octant_5);

//        //point_cloud_decoder_6->syncToHeader(compressed_data);
//        point_cloud_decoder_6->switchBuffers();
//        point_cloud_decoder_6->setOutputCloud(octant_6);

//        //point_cloud_decoder_7->syncToHeader(compressed_data);
//        point_cloud_decoder_7->switchBuffers();
//        point_cloud_decoder_7->setOutputCloud(octant_7);

std::cout << "Fiz Initialization dos Decoders " << std::endl;

my_read_frame_header(compressed_data);

std::cout << "Acabei Read Frame Header " << std::endl;

point_cloud_decoder_0->entropyDecoding(compressed_data);

//        for(int i=0; i<point_cloud_decoder_0->binary_tree_data_vector_.size();i++)
//            printf("%i --> %x \n",i,point_cloud_decoder_0->binary_tree_data_vector_[i]);






std::cout << "Acabei Entropy Decoding " << std::endl;

point_cloud_decoder_0->point_coder_.initializeDecoding();
//        point_cloud_decoder_1->point_coder_.initializeDecoding();
//        point_cloud_decoder_2->point_coder_.initializeDecoding();
//        point_cloud_decoder_3->point_coder_.initializeDecoding();
//        point_cloud_decoder_4->point_coder_.initializeDecoding();
//        point_cloud_decoder_5->point_coder_.initializeDecoding();
//        point_cloud_decoder_6->point_coder_.initializeDecoding();
//        point_cloud_decoder_7->point_coder_.initializeDecoding();

std::cout << "Fiz Initialization Decoding " << std::endl;

point_cloud_decoder_0->output_->points.clear();
//        point_cloud_decoder_1->output_->points.clear();
//        point_cloud_decoder_2->output_->points.clear();
//        point_cloud_decoder_3->output_->points.clear();
//        point_cloud_decoder_4->output_->points.clear();
//        point_cloud_decoder_5->output_->points.clear();
//        point_cloud_decoder_6->output_->points.clear();
//        point_cloud_decoder_7->output_->points.clear();

std::cout << "Acabei Point Clear " << std::endl;

point_cloud_decoder_0->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_0->point_count_));
//        point_cloud_decoder_1->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_1->point_count_));
//        point_cloud_decoder_2->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_2->point_count_));
//        point_cloud_decoder_3->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_3->point_count_));
//        point_cloud_decoder_4->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_4->point_count_));
//        point_cloud_decoder_5->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_5->point_count_));
//        point_cloud_decoder_6->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_6->point_count_));
//        point_cloud_decoder_7->output_->points.reserve(static_cast<std::size_t> (point_cloud_decoder_7->point_count_));

std::cout << "Acabei Point Reserve " << std::endl;

std::vector<char>vec_0 = {point_cloud_decoder_0->binary_tree_data_vector_.begin(), point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0};
std::vector<char>vec_1 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 , point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1};
std::vector<char>vec_2 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2};
std::vector<char>vec_3 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3};
std::vector<char>vec_4 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4};
std::vector<char>vec_5 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5};
std::vector<char>vec_6 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5, point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5 + size_6};
std::vector<char>vec_7 = {point_cloud_decoder_0->binary_tree_data_vector_.begin() + size_0 + size_1 + size_2 + size_3 + size_4 + size_5 + size_6, point_cloud_decoder_0->binary_tree_data_vector_.end()};

for(int i=0; i<vec_0.size();i++)
    printf("%i --> %x \n",i,vec_0[i]);


std::cout << "Acabei Vector Division " << std::endl;

if(point_cloud_decoder_0->i_frame_){
   point_cloud_decoder_0->deserializeTree(vec_0,false);
//           point_cloud_decoder_1->deserializeTree(vec_1,false);
//           point_cloud_decoder_2->deserializeTree(vec_2,false);
//           point_cloud_decoder_3->deserializeTree(vec_3,false);
//           point_cloud_decoder_4->deserializeTree(vec_4,false);
//           point_cloud_decoder_5->deserializeTree(vec_5,false);
//           point_cloud_decoder_6->deserializeTree(vec_6,false);
//           point_cloud_decoder_7->deserializeTree(vec_7,false);
}else{
   point_cloud_decoder_0->deserializeTree(vec_0,true);
//           point_cloud_decoder_1->deserializeTree(vec_1,true);
//           point_cloud_decoder_2->deserializeTree(vec_2,true);
//           point_cloud_decoder_3->deserializeTree(vec_3,true);
//           point_cloud_decoder_4->deserializeTree(vec_4,true);
//           point_cloud_decoder_5->deserializeTree(vec_5,true);
//           point_cloud_decoder_6->deserializeTree(vec_6,true);
//           point_cloud_decoder_7->deserializeTree(vec_7,true);
}

 std::cout << "Acabei Desirialzation " << std::endl;

point_cloud_decoder_0->output_->height = 1;
point_cloud_decoder_0->output_->width = static_cast<uint32_t> (octant_0->points.size ());
point_cloud_decoder_0->output_->is_dense = false;

//        point_cloud_decoder_1->output_->height = 1;
//        point_cloud_decoder_1->output_->width = static_cast<uint32_t> (octant_1->points.size ());
//        point_cloud_decoder_1->output_->is_dense = false;

//        point_cloud_decoder_2->output_->height = 1;
//        point_cloud_decoder_2->output_->width = static_cast<uint32_t> (octant_2->points.size ());
//        point_cloud_decoder_2->output_->is_dense = false;

//        point_cloud_decoder_3->output_->height = 1;
//        point_cloud_decoder_3->output_->width = static_cast<uint32_t> (octant_3->points.size ());
//        point_cloud_decoder_3->output_->is_dense = false;

//        point_cloud_decoder_4->output_->height = 1;
//        point_cloud_decoder_4->output_->width = static_cast<uint32_t> (octant_4->points.size ());
//        point_cloud_decoder_4->output_->is_dense = false;

//        point_cloud_decoder_5->output_->height = 1;
//        point_cloud_decoder_5->output_->width = static_cast<uint32_t> (octant_5->points.size ());
//        point_cloud_decoder_5->output_->is_dense = false;

//        point_cloud_decoder_6->output_->height = 1;
//        point_cloud_decoder_6->output_->width = static_cast<uint32_t> (octant_6->points.size ());
//        point_cloud_decoder_6->output_->is_dense = false;

//        point_cloud_decoder_7->output_->height = 1;
//        point_cloud_decoder_7->output_->width = static_cast<uint32_t> (octant_7->points.size ());
//        point_cloud_decoder_7->output_->is_dense = false;

std::cout << "Octant 0 Size: " << octant_0->points.size() << std::endl;
//        std::cout << "Octant 1 Size: " << octant_1->points.size() << std::endl;
//        std::cout << "Octant 2 Size: " << octant_2->points.size() << std::endl;
//        std::cout << "Octant 3 Size: " << octant_3->points.size() << std::endl;
//        std::cout << "Octant 4 Size: " << octant_4->points.size() << std::endl;
//        std::cout << "Octant 5 Size: " << octant_5->points.size() << std::endl;
//        std::cout << "Octant 6 Size: " << octant_6->points.size() << std::endl;
//        std::cout << "Octant 7 Size: " << octant_7->points.size() << std::endl;



for (auto &point: *octant_0) {
    point._PointXYZRGB::r = 255;
    out_cloud->push_back(point);
}
//        for (auto &point: *octant_1) {
//            out_cloud->push_back(point);
//        }
//        for (auto &point: *octant_2) {
//            out_cloud->push_back(point);
//        }
//        for (auto &point: *octant_3) {
//            out_cloud->push_back(point);
//        }
//        for (auto &point: *octant_4) {
//            out_cloud->push_back(point);
//        }
//        for (auto &point: *octant_5) {
//            out_cloud->push_back(point);
//        }
//        for (auto &point: *octant_6) {
//            out_cloud->push_back(point);
//        }
//        for (auto &point: *octant_7) {
//            out_cloud->push_back(point);
//        }

*/
