/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

#include <iostream>
#include <math.h>
#include <cmath>
#include <deque>
#include <cstdlib>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

//#include <voxel_grid_occlusion_estimation.h>
#include <component_test/occlusion_culling.h>
#include "fcl_utility.h"
using namespace fcl;


geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
bool parseInputs(int argc, char **argv);
double rad2deg (double rad);

std::string modelPath;
int maxIterations;
float initial_x;
float initial_y;
float initial_z;
float initial_yaw;

double randomDouble(double min, double max) {
    return ((double) random()/RAND_MAX)*(max-min) + min;
}

int main(int argc, char **argv)
{
    // Parse inputs. Exit if help is used
    //if (parseInputs(argc, argv))
    //    return 0;

	// >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle ros_node;

	// >>>>>>>>>>>>>>>>>
    // Get config parameters
    // >>>>>>>>>>>>>>>>>
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");
    nodeHandle.param<int>("maxIterations", maxIterations, 20 );
    nodeHandle.param<std::string>("modelFilename", modelPath, "etihad_nowheels_densed.pcd");
    nodeHandle.param<float>("initial_x", initial_x, 5);
    nodeHandle.param<float>("initial_y", initial_y, 16);
    nodeHandle.param<float>("initial_z", initial_z, 4);
    nodeHandle.param<float>("initial_yaw", initial_yaw, 0);

    // >>>>>>>>>>>>>>>>>
    // Variable declaration
    // >>>>>>>>>>>>>>>>>

    // Point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> predictedCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr predictedCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr predictedCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matchedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Rviz sensor messages
    sensor_msgs::PointCloud2 cloud1;
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud2 cloud3;
    sensor_msgs::PointCloud2 cloud4;
    sensor_msgs::PointCloud2 cloud5;
    sensor_msgs::PointCloud2 cloud6;
    geometry_msgs::PoseArray viewpoints;

    cloud1.header.frame_id = "base_point_cloud";
    cloud2.header.frame_id = "base_point_cloud";
    cloud3.header.frame_id = "base_point_cloud";
    cloud4.header.frame_id = "base_point_cloud";
    cloud5.header.frame_id = "base_point_cloud";
    cloud6.header.frame_id = "base_point_cloud";
    viewpoints.header.frame_id= "base_point_cloud";

	
    // >>>>>>>>>>>>>>>>>
    // Create publishers
    // >>>>>>>>>>>>>>>>>
    ros::Publisher originalCloudPub          = ros_node.advertise<sensor_msgs::PointCloud2>("original_cloud", 100);
    //ros::Publisher originalFilteredCloudPub  = ros_node.advertise<sensor_msgs::PointCloud2>("original_cloud_filtered", 100);
    ros::Publisher predictedCloudPub         = ros_node.advertise<sensor_msgs::PointCloud2>("predicted_cloud", 100);
    ros::Publisher predictedCloudFilteredPub = ros_node.advertise<sensor_msgs::PointCloud2>("predicted_cloud_filtered", 100);
    //ros::Publisher coverdCloudPub            = ros_node.advertise<sensor_msgs::PointCloud2>("covered_cloud", 100);
    //ros::Publisher matchedCloudPub           = ros_node.advertise<sensor_msgs::PointCloud2>("matched_cloud", 100);
    ros::Publisher sensorPosePub             = ros_node.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);

    std::cout<<"[] Created publishers\n";

    // >>>>>>>>>>>>>>>>>
    // Load model
    // >>>>>>>>>>>>>>>>>
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/" + modelPath, *originalCloud);
    std::cout<<"[] Loaded Model file\n";

    OcclusionCulling occlusionCulling(ros_node, modelPath);
    std::cout<<"[] Initialized occlusion culler\n";

	// Display model

    // >>>>>>>>>>>>>>>>>
    // Iteratively perform exploration
    // >>>>>>>>>>>>>>>>>

    // Camera settings
    Vec3f rpy(0,0.093,0);
    Vec3f xyz(0,0.0,-0.055);

    // Position Variables
    geometry_msgs::Pose pt,loc;
    double yaw;

    Vec3f pos_inc(0,0,0); // Increment to add to position

    pt.position.x = initial_x;
    pt.position.y = initial_y;
    pt.position.z = initial_z;
    yaw = initial_yaw;


    /*
    pt.position.x = 4.27472;
    pt.position.y = 18.9289;
    pt.position.z = 5;
    yaw = -2.96177;
    */


    // Control variables
    bool isSpinning = true;
    bool isTerminating = false;

    double maxRotationRate = M_PI_4;


    for (int viewPointCount=0; viewPointCount<maxIterations; viewPointCount++)
    {
        std::cout << "Viewpoint " << viewPointCount << "\n";

        // Create random pose
        //pt.position.x = randomDouble(10, -10);
        //pt.position.y = randomDouble(30, -30);
        //pt.position.z = randomDouble(0, 10);
        //yaw = randomDouble(-M_PI, M_PI);

        // Initially spin in place to scan the surrounding area
        if (isSpinning) {
            if (yaw >= 2*M_PI)
                isSpinning = false;
            else
                yaw += M_PI_4;
        }

        // Otherwise explore
        else {
            pt.position.x += pos_inc[0];
            pt.position.y += pos_inc[1];
            pt.position.z += pos_inc[2];
            //yaw += randomDouble(-M_PI/8, M_PI/8);
        }

        // Generate quaterion
        tf::Quaternion tf_q ;
        tf_q = tf::createQuaternionFromYaw(yaw);
        pt.orientation.x = tf_q.getX();
        pt.orientation.y = tf_q.getY();
        pt.orientation.z = tf_q.getZ();
        pt.orientation.w = tf_q.getW();

        // Find camera position in UAV frame
        loc= uav2camTransformation(pt,rpy,xyz);

        // Push viewpoint into PoseArray
        viewpoints.poses.push_back(loc);

        // Perform occlusion culling
        pcl::PointCloud<pcl::PointXYZ> tempCloud;
        
        ros::Time tic = ros::Time::now();
        tempCloud = occlusionCulling.extractVisibleSurface(loc); // perform culling
        ros::Time toc = ros::Time::now();
        
        printf("\nOcclusion culling took: %f seconds\n", toc.toSec() - tic.toSec() );
        
        occlusionCulling.visualizeFOV(loc); // visualize FOV



        // Find norm of observed area
        if (tempCloud.points.size() > 2) {
            // Stop spinning, found something
            isSpinning = false;

            std::vector<int> indices;
            Eigen::Vector4f plane_parameters;
            float curvature;

            indices.resize (tempCloud.points.size ());
            for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
                indices[i] = i;

            computePointNormal (tempCloud, indices, plane_parameters, curvature);

            // Calculate normal of surface
            double yaw_new = atan2 (-plane_parameters[1], -plane_parameters[0]);




            //std::cout << "Normal = [" << plane_parameters[0] << ", " << plane_parameters[1] << ", "<< plane_parameters[2] << "]\n";
            printf("Current Position:\n"
            "x = %f \n"
            "y = %f \n"
            "z = %f \n"
            "yaw = %f \n"
            "NewYaw = %f \n", pt.position.x, pt.position.y, pt.position.z, rad2deg(yaw), rad2deg(yaw_new));          



			// If the current viewpoint and normal are far away and require very fast rotation, perform some action
			double angle_difference = fmod(yaw - yaw_new, M_PI);
			std::cout << "Angle difference: " << rad2deg(angle_difference) << "\n";
			
            if (fabs(angle_difference) > maxRotationRate) {
                yaw += copysign(maxRotationRate/2, angle_difference);

                // Move perpendicular to x-y of norm
                /*
                pos_inc[0] = sin(yaw);
                pos_inc[1] = -cos(yaw);
                */

                std::cout << "INFO: Max rotation rate triggered\n";
            }
            
            // Otherwise use the normal as the new angle
            else {
                yaw = yaw_new;

                // Move perpendicular to x-y of norm
                pos_inc[0] = sin(yaw_new);
                pos_inc[1] = -cos(yaw_new);
            }

            // Move vertically opposite of norm (objective is to look directly at flat part of object, not above or below)
            //pos_inc[2] = -plane_parameters[2];
            
            std::cout << "\n";
        }


        // Lost track, didn't see anything, start spinning
        else {
            /*
            if (!isSpinning)
            	isTerminating = true;
            */
            //isSpinning = true;
        }





        // Add points to cloud
        predictedCloud += tempCloud;

        // Update rviz (visualization)
        if (ros::ok())
        {
            pcl::toROSMsg(*originalCloud, cloud1); 	//cloud of original (white) using original cloud
            pcl::toROSMsg(predictedCloud, cloud2); 	//cloud of the not occluded voxels (blue) using occlusion culling

            cloud1.header.frame_id = "base_point_cloud";
            cloud2.header.frame_id = "base_point_cloud";

            cloud1.header.stamp = ros::Time::now();
            cloud2.header.stamp = ros::Time::now();
            viewpoints.header.stamp = ros::Time::now();

            originalCloudPub.publish(cloud1);
            predictedCloudPub.publish(cloud2);
            sensorPosePub.publish(viewpoints);
        }

        if (isTerminating) {
            break;
        }
    }

    predictedCloudPtr->points = predictedCloud.points;


    // >>>>>>>>>>>>>>>>>
    // Original cloud Grid
    // >>>>>>>>>>>>>>>>>
    //used VoxelGridOcclusionEstimationT since the voxelGrid does not include getcentroid function
    float res = 0.5;

    pcl::VoxelGridOcclusionEstimationT originalCloudFilteredVoxels;
    originalCloudFilteredVoxels.setInputCloud (originalCloud);
    originalCloudFilteredVoxels.setLeafSize (res, res, res);
    originalCloudFilteredVoxels.initializeVoxelGrid();
    originalCloudFilteredVoxels.filter(*originalCloudFiltered);
    std::cout<<"original filtered "<<originalCloudFiltered->points.size()<<"\n";

    // >>>>>>>>>>>>>>>>>
    // Predicted cloud Grid
    // >>>>>>>>>>>>>>>>>
    pcl::VoxelGridOcclusionEstimationT predictedCloudFilteredVoxels;
    predictedCloudFilteredVoxels.setInputCloud (predictedCloudPtr);
    predictedCloudFilteredVoxels.setLeafSize (res, res, res);
    predictedCloudFilteredVoxels.initializeVoxelGrid();
    predictedCloudFilteredVoxels.filter(*predictedCloudFiltered);
    std::cout<<"predicted filtered "<<predictedCloudFiltered->points.size()<<"\n";

    float test = (float)predictedCloudFiltered->points.size()/(float)originalCloudFiltered->points.size() *100;
    std::cout<<"TEST predicted coverage percentage : "<<test<<"\n";

    return 0;
}




bool parseInputs(int argc, char **argv) {
	std::string arg;
	
    for (int i = 1; i<argc; i++) {
		arg = std::string(argv[i]); //Cast to string
		
        if (arg == "-m") {
			// Model name
            modelPath = argv[i+1];
        }
        else if (arg == "-i") {
			// Max iterations
            maxIterations = atoi(argv[i+1]);
        }
        else if (arg == "-h") {
			// Help
            std::cout << "Usage:\n\nwall_follower [-i <max iterations>] [-m <model name>]\n\n";
            return true;
        }
    }

    std::cout << "MODEL: " << modelPath << "\n";
    std::cout << "MAX ITERATIONS " << maxIterations << "\n";
    return false;
}



double rad2deg (double rad){
	return rad*180/M_PI;
}



geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz)
{
    Eigen::Matrix4d uav_pose, uav2cam, cam_pose;
    //UAV matrix pose
    Eigen::Matrix3d R;
    Eigen::Vector3d T1(pose.position.x,pose.position.y,pose.position.z);
    tf::Quaternion qt(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);
    uav_pose.setZero ();
    uav_pose.block (0, 0, 3, 3) = R;
    uav_pose.block (0, 3, 3, 1) = T1;
    uav_pose (3, 3) = 1;

    //transformation matrix
    qt = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);
    tf::Matrix3x3 R2(qt);
    Eigen::Vector3d T2(xyz[0],xyz[1],xyz[2]);
    tf::matrixTFToEigen(R2,R);
    uav2cam.setZero ();
    uav2cam.block (0, 0, 3, 3) = R;
    uav2cam.block (0, 3, 3, 1) = T2;
    uav2cam (3, 3) = 1;

    //preform the transformation
    cam_pose = uav_pose * uav2cam;

    Eigen::Matrix4d cam2cam;
    //the transofrmation is rotation by +90 around x axis of the camera
    cam2cam <<   1, 0, 0, 0,
            0, 0,-1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
    Eigen::Matrix4d cam_pose_new = cam_pose * cam2cam;
    geometry_msgs::Pose p;
    Eigen::Vector3d T3;
    Eigen::Matrix3d Rd;
    tf::Matrix3x3 R3;
    Rd = cam_pose_new.block (0, 0, 3, 3);
    tf::matrixEigenToTF(Rd,R3);
    T3 = cam_pose_new.block (0, 3, 3, 1);
    p.position.x=T3[0];
    p.position.y=T3[1];
    p.position.z=T3[2];
    R3.getRotation(qt);
    p.orientation.x = qt.getX();
    p.orientation.y = qt.getY();
    p.orientation.z = qt.getZ();
    p.orientation.w = qt.getW();

    return p;

}
