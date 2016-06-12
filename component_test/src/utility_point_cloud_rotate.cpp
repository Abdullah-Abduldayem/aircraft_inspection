#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>

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
    pcl::PointXYZ point;

    // Fill in the cloud data
    float res = 0.1;

    
	for (float k=0; k<4; k += res) {
		for(float theta=0; theta<2*M_PI; theta+=0.005) {
			float r = 10;
			
			point.x = r*sin(theta);
			point.y = 2*r*cos(theta);
			point.z = k;
			
			for (int i=0; i<1; i++) { //Used to pad out the number of points so rviz will accept it
				cloud.points.push_back(point);
			}
		}
    }

    cloud.width    = cloud.points.size();
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    std::string path = ros::package::getPath("component_test") + "/src/pcd/rect.pcd";
    pcl::io::savePCDFileASCII (path, cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points." << std::endl;


    // >>>>>>>>>>>>>
    // Visualize
    // >>>>>>>>>>>>>
    ros::Publisher pub = ros_node.advertise<sensor_msgs::PointCloud2>("original_cloud", 10);

    sensor_msgs::PointCloud2 cloudMsg;
    cloudMsg.header.frame_id = "base_point_cloud";
    pcl::toROSMsg(cloud, cloudMsg);

    pub.publish(cloudMsg);

    return (0);
}
