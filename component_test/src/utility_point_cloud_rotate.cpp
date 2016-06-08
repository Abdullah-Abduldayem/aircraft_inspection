#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/package.h>

int
main (int argc, char** argv)
{
    ros::init(argc, argv, "utility_point_cloud_rotate");
    ros::NodeHandle ros_node;


    // >>>>>>>>>>>>>>
    // Rotate cloud
    // >>>>>>>>>>>>>>

    /*
    // Load cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/occlusionFreeCloud_1m.pcd", *originalCloud);
    std::cout<<"[] Loaded Model file\n";

    // Duplicate cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points = originalCloud->points;
    cloud.width = 1;
    cloud.height = cloud.points.size();

    // Rotate cloud
    std::cout << "Rotating\n";

    double temp;
    for (int i=0; i<cloud.points.size(); i++){
    	temp = cloud.points[i].y;
    	cloud.points[i].y = -cloud.points[i].z - 5;
    	cloud.points[i].z = temp + 3;
    }
    */


    // >>>>>>>>>>>>>>
    // Create geometric cloud (rectangular prism)
    // >>>>>>>>>>>>>>
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;



    // Save cloud
    std::cout << "Saving...\n";
    pcl::io::savePCDFileASCII ("processed.pcd", cloud);
    std::cout << "Saved\n";

    return (0);
}
