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
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/features/normal_3d.h>

//#include <voxel_grid_occlusion_estimation.h>
#include <component_test/occlusion_culling.h>
#include "fcl_utility.h"
using namespace fcl;


geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
double rad2deg (double rad);
double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2);
bool checkIfConvex(pcl::PointXYZ sensorPoint, pcl::PointCloud<pcl::PointXYZ> cloud);
Eigen::Vector3f getSummedDirectionToPoint(pcl::PointXYZ point, pcl::PointCloud<pcl::PointXYZ> cloud);

std::string modelPath;
int maxIterations;
float initial_x;
float initial_y;
float initial_z;
float initial_yaw;
float voxelRes;

// Colors for console window
const std::string cc_black("\033[0;30m");
const std::string cc_red("\033[0;31m");
const std::string cc_green("\033[1;32m");
const std::string cc_yellow("\033[1;33m");
const std::string cc_blue("\033[1;34m");
const std::string cc_magenta("\033[0;35m");
const std::string cc_cyan("\033[0;36m");
const std::string cc_white("\033[0;37m");

const std::string cc_bold("\033[1m");
const std::string cc_darken("\033[2m");
const std::string cc_underline("\033[4m");
const std::string cc_background("\033[7m");
const std::string cc_strike("\033[9m");

const std::string cc_erase_line("\033[2K");
const std::string cc_reset("\033[0m");

double randomDouble(double min, double max) {
    return ((double) random()/RAND_MAX)*(max-min) + min;
}

double rad2deg (double rad) {
    return rad*180/M_PI;
}

double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2){
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
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
    nodeHandle.param<float>("voxelRes", voxelRes, 0.5f );

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
    ros::Publisher predictedCloudPub         = ros_node.advertise<sensor_msgs::PointCloud2>("predicted_cloud", 100);
    ros::Publisher sensorPosePub             = ros_node.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);

    std::cout<<"[] Created publishers\n";

    // >>>>>>>>>>>>>>>>>
    // Load model
    // >>>>>>>>>>>>>>>>>
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/" + modelPath, *originalCloud);
    std::cout<<"[] Loaded Model file\n";

    std::cout<<"Initializing occlusion culler...\n";
    OcclusionCulling occlusionCulling(ros_node, modelPath);
    std::cout<<"[] Initialized occlusion culler\n";

    // >>>>>>>>>>>>>>>>>
    // Iteratively perform exploration
    // >>>>>>>>>>>>>>>>>

    // Camera settings
    Vec3f rpy(0,0.093,0);
    Vec3f xyz(0,0.0,-0.055);

    // Position Variables
    geometry_msgs::Pose pt,loc;
    double yaw;
    double yaw_wallNormal = 0;

    // Increment to add to position
    Vec3f pos_inc(0,0,0);
    double yaw_inc = 0;

    pt.position.x = initial_x;
    pt.position.y = initial_y;
    pt.position.z = initial_z;
    yaw = initial_yaw;

    // Control variables
    bool isSpinning = true;
    bool isTerminating = false;
    //double viewingAngle = -M_PI_4/4; //Used to keep the camera looking ahead so it doesn't crash into a wall
    double viewingAngle = 0;
    double maxRotationRate = M_PI_4/2;
    double spinningRate = M_PI_4/4;

    for (int viewPointCount=0; viewPointCount<maxIterations; viewPointCount++)
    {
        std::cout << cc_magenta << cc_bold << "\nViewpoint " << viewPointCount << cc_reset << "\n";

        // Initially spin in place to scan the surrounding area
        if (isSpinning) {
            yaw += spinningRate;

            if (yaw >= 2*M_PI)
                yaw -= 2*M_PI;
        }

        // Otherwise explore
        else {
            pt.position.x += pos_inc[0];
            pt.position.y += pos_inc[1];
            pt.position.z += pos_inc[2];
            yaw += yaw_inc;
        }

        printf("[x, y, z, yaw] = [%f,\t%f,\t%f,\t%f] \n",
               pt.position.x, pt.position.y, pt.position.z, rad2deg(yaw));

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
        tempCloud = occlusionCulling.extractVisibleSurface(loc); // perform culling
        occlusionCulling.visualizeFOV(loc); // visualize FOV




        if (tempCloud.points.size() > 2) {
            // Stop spinning, found something
            isSpinning = false;

            std::vector<int> indices;
            Eigen::Vector4f plane_parameters;
            float curvature;
            Eigen::Vector4f centroid;

            // Find norm of observed area
            indices.resize (tempCloud.points.size ());
            for (int i = 0; i < static_cast<int> (indices.size ()); ++i)
                indices[i] = i;

            computePointNormal (tempCloud, indices, plane_parameters, curvature);
            printf("Normal = [%f, %f, %f]\n", plane_parameters[0], plane_parameters[1], plane_parameters[2]);
            printf("Curvature = %f\n", curvature);

            // Calculate normal angle (yaw) of surface
            yaw_wallNormal = atan2 (plane_parameters[1], plane_parameters[0]);
            yaw_wallNormal = fmod(yaw_wallNormal + 2*M_PI, 2*M_PI); // Make it between 0 to 360

            // Calculate planar distance
            pcl::compute3DCentroid(tempCloud,centroid);
            printf("Centroid = [%f, %f, %f]\n", centroid[0], centroid[1], centroid[2]);
            float wall_distance = sqrt( (centroid[0]-pt.position.x)*(centroid[0]-pt.position.x) + (centroid[1]-pt.position.y)*(centroid[1]-pt.position.y) );
            printf("Distance = %f\n", wall_distance);

            // Move the robot to 1.5m away from the wall if it's too close or too far
            bool wasMoved = false;
            if (wall_distance >= 2.5 || wall_distance <= 1) {
                float wall_direction[2];
                wall_direction[0] = centroid[0]-pt.position.x;
                wall_direction[1] = centroid[1]-pt.position.y;

                pt.position.x += wall_direction[0]*(1-1.5/wall_distance);
                pt.position.y += wall_direction[1]*(1-1.5/wall_distance);

                pos_inc[0] = 0;
                pos_inc[1] = 0;

                std::cout << cc_yellow << "INFO: Moving to 1.5m away from wall\n" << cc_reset;
                wasMoved = true;
            }


            // Is z component the most dominant? (ie. a floor or ceiling?)
            // If so, ignore computed yaw and keep moving
            if (fabs(plane_parameters[2]) > 0.5) {
                std::cout << cc_yellow << "INFO: Ceiling/floor detected\n" << cc_reset;
                continue;
            }


            // Test if shape is concave or convex
            pcl::PointXYZ origin = pcl::PointXYZ (pt.position.x, pt.position.y, pt.position.z);
            bool isConvex = checkIfConvex(origin, tempCloud);
            if (isConvex /*pcl::isPointIn2DPolygon(origin, tempCloud)*/) {
                std::cout << cc_green << "Convex\n" << cc_reset;
            }
            else {
                std::cout << cc_green << "Concave\n" << cc_reset;
                isConvex= true;
            }

            // If the current yaw and desired yaw (wall normal) are far away, add a small increment to the yaw
            double angle_difference = yaw_wallNormal - yaw;
            printf("Desired Yaw = %f\n"
                   "Angle Diff = %f \n", rad2deg(yaw_wallNormal), rad2deg(angle_difference) );

            if (fabs(angle_difference) > maxRotationRate) {
                //yaw_inc = copysign(maxRotationRate, angle_difference);
                yaw_inc = spinningRate;


                std::cout << cc_yellow << "INFO: Max rotation rate triggered\n" << cc_reset;
                isSpinning = true;
            }

            // Otherwise use the normal as the new angle
            else {
                yaw = yaw_wallNormal + copysign(viewingAngle, -angle_difference);
                yaw_inc = 0;
            }

            // Move perpendicular to x-y of norm
            if (!wasMoved) {
                pos_inc[0] = sin(yaw_wallNormal);
                pos_inc[1] = -cos(yaw_wallNormal);
            }

            // Move vertically opposite of norm (objective is to look directly at flat part of object, not above or below)
            //pos_inc[2] = -plane_parameters[2];
        }

        /*
        // Lost track, didn't see anything, start spinning
        else {
            if (!isSpinning)
            	isTerminating = true;
            //isSpinning = true;
        }
        */



        std::cout << "\n";

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
        else {
            std::cout << cc_red << cc_bold << "Couldn't visualize data (rviz)\n";
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


bool checkIfConvex(pcl::PointXYZ sensorPoint, pcl::PointCloud<pcl::PointXYZ> cloud) {
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->points = cloud.points;
	
    // Compute centroid of cloud
    pcl::PointXYZ centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (int p=0; p<cloud.points.size(); p++) {
        centroid.x += cloud.points[p].x;
        centroid.y += cloud.points[p].y;
        centroid.z += cloud.points[p].z;
    }

    centroid.x /= cloud.points.size();
    centroid.y /= cloud.points.size();
    centroid.z /= cloud.points.size();

    cloud.push_back(centroid);


	// Find closest point 
	pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>); 
	tree->setInputCloud(cloud_ptr); 
	std::vector<int> nn_indices (1); 
	std::vector<float> nn_dists (1); 
	
	tree->nearestKSearch(sensorPoint, 1, nn_indices, nn_dists);  
	pcl::PointXYZ closest_pt = cloud_ptr->points[nn_indices[0]]; 

	// Find angles between closest_pt and centroid/sensor
	float angle_sensor = atan2 (closest_pt.y-sensorPoint.y, closest_pt.x-sensorPoint.x);
	float angle_centroid = atan2 (closest_pt.y-centroid.y, closest_pt.x-centroid.x);
	
	angle_sensor = fmod(angle_sensor + 2*M_PI, 2*M_PI); // Make it between 0 to 360
	angle_centroid = fmod(angle_centroid + 2*M_PI, 2*M_PI); // Make it between 0 to 360
	
	
	float angle_diff = fmod(angle_sensor - angle_centroid+ 2*M_PI, 2*M_PI);
	std::cout << cc_green << "Angle Diff: " << angle_diff << "\n" << cc_reset;
	
	/*
	Eigen::Vector3f dir_centroid = getSummedDirectionToPoint(centroid, cloud);
	Eigen::Vector3f dir_sensor = getSummedDirectionToPoint(sensorPoint, cloud);

	printf("Dir1: [%f, %f, %f]\n", dir_centroid[0], dir_centroid[1], dir_centroid[2]);
	printf("Dir2: [%f, %f, %f]\n", dir_sensor[0], dir_sensor[1], dir_sensor[2]);

	// Find angle difference
	float angle1 = atan2 (dir_centroid[1], dir_centroid[0]);
	float angle2 = atan2 (dir_sensor[1], dir_sensor[0]);
	std::cout << cc_green << "Angle1: " << angle1 << "\n" << cc_reset;
	std::cout << cc_green << "Angle2: " << angle2 << "\n" << cc_reset;
	return true;
	*/
}


Eigen::Vector3f getSummedDirectionToPoint(pcl::PointXYZ point, pcl::PointCloud<pcl::PointXYZ> cloud){
	Eigen::Vector3f out(0,0,0);
	
	cout << out[0] << "\n";
	
	int i;
	for (i=0; i<cloud.points.size(); i++){
		out[0] += point.x - cloud.points[i].x;
		out[1] += point.y - cloud.points[i].y;
		out[2] += point.z - cloud.points[i].z;
	}
	
	out.normalize();
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
