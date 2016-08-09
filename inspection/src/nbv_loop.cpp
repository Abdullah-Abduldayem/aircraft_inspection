/* Author: Abdullah Abduldayem
 * Derived from coverage_quantification.cpp
 */

// catkin_make -DCATKIN_WHITELIST_PACKAGES="aircraft_inspection" && rosrun aircraft_inspection wall_follower

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo_msgs/ModelStates.h> //Used for absolute positioning

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

//PCL
//#include <pcl/filters.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/registration/icp.h>

/*
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
*/

/*
#include <math.h>
#include <cmath>
#include <deque>
#include <cstdlib>



#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud.h>

#include <std_msgs/Bool.h>

#include <message_filters/subscriber.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



//#include <voxel_grid_occlusion_estimation.h>
#include <component_test/occlusion_culling.h>
#include "fcl_utility.h"
using namespace fcl;


geometry_msgs::Pose uav2camTransformation(geometry_msgs::Pose pose, Vec3f rpy, Vec3f xyz);
double rad2deg (double rad);
double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2);

std::string modelPath;
int maxIterations;
float initial_x;
float initial_y;
float initial_z;
float initial_yaw;
float voxelRes;
*/

// =========================
// Colors for console window
// =========================
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


// ===================
// === Variables  ====
// ===================
bool isDebug = !true; //Set to true to see debug text
bool isDebugStates = false;

// == Consts
std::string depth_topic   = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string position_topic = "/iris/ground_truth/pose";

// == Termination variables
bool isTerminating = false;
int iteration_count = 0;
int max_iterations = 100;

// == Navigation variables
float distance_threshold = 0.5f;
float angular_threshold = 10.0 * M_PI/180.0;//Degrees to radians

geometry_msgs::Pose mobile_base_pose;
//geometry_msgs::Pose mobile_base_pose_prev;
geometry_msgs::PoseStamped setpoint;

// == Publishers
ros::Publisher pub_global_cloud;
ros::Publisher pub_setpoint;

// == Subsctiptions
tf::TransformListener *listener;


// == Point clouds
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloudPtr;
//pcl::VoxelGrid<pcl::PointXYZRGB> occGrid;

float pc_res = 0.1f; //Voxel grid resolution



// ======================
// Function prototypes (@todo: move to header file)
// ======================

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
void termination_check();
void generate_viewpoints();
void evaluate_viewpoints();
void set_waypoint();
double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current);

// ======================
// Create a state machine
// ======================
enum NBV_STATE {
    NBV_STATE_NOT_STARTED,
    NBV_STATE_INITIALIZING,
    NBV_STATE_IDLE,
    NBV_STATE_TERMINATION_CHECK, NBV_STATE_TERMINATION_MET, NBV_STATE_TERMINATION_NOT_MET,
    NBV_STATE_VIEWPOINT_GENERATION, NBV_STATE_DONE_VIEWPOINT_GENERATION, 
    NBV_STATE_VIEWPOINT_EVALUATION, NBV_STATE_DONE_VIEWPOINT_EVALUATION,
    NBV_STATE_PATH_PLANNING,
    NBV_STATE_MOVING, NBV_STATE_DONE_MOVING
    };
NBV_STATE state = NBV_STATE_NOT_STARTED;

// ================
// Functions
// ================

double randomDouble(double min, double max) {
    return ((double) random()/RAND_MAX)*(max-min) + min;
}

/*
double rad2deg (double rad) {
    return rad*180/M_PI;
}

double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2){
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
}
*/


int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    state = NBV_STATE_INITIALIZING;
    
    std::cout << cc_red << "BEGIN NBV LOOP\n" << cc_reset;

    ros::init(argc, argv, "nbv_loop");
    ros::NodeHandle ros_node;

    // >>>>>>>>>>>>>>>>>
    // Subscribers
    // >>>>>>>>>>>>>>>>>
    
    // Sensor data
    ros::Subscriber sub_kinect = ros_node.subscribe(depth_topic, 1, depthCallback);
    ros::Subscriber sub_pose = ros_node.subscribe(position_topic, 1, positionCallback);
    
    listener = new tf::TransformListener();
    
    // >>>>>>>>>>>>>>>>>
    // Publishers
    // >>>>>>>>>>>>>>>>>
    
    pub_global_cloud = ros_node.advertise<sensor_msgs::PointCloud2>("/global_cloud", 10);
    //pub = ros_node.advertise<sensor_msgs::PointCloud2> ("/voxgrid", 1);
    //pub_pose = ros_node.advertise<geometry_msgs::PoseStamped> ("/voxgrid/pose", 1);

    // Drone setpoints
    pub_setpoint = ros_node.advertise<geometry_msgs::PoseStamped>("/iris/mavros/setpoint_position/local", 10);

    // >>>>>>>>>>>>>>>>>
    // Start the FSM
    // >>>>>>>>>>>>>>>>>
    state = NBV_STATE_IDLE;
    
    ros::Rate loop_rate(10);
    while (ros::ok() && !isTerminating)
    {
        switch(state){
            case NBV_STATE_IDLE:
            case NBV_STATE_DONE_MOVING:
                state = NBV_STATE_TERMINATION_CHECK;
                
                iteration_count++;
                std::cout << cc_yellow << "Iteration: " << iteration_count << "\n" << cc_reset;
                
                termination_check();
                break;
                
            case NBV_STATE_TERMINATION_MET:
                isTerminating = true;
                std::cout << cc_yellow << "Termination condition met\n" << cc_reset;
                break;
                
            case NBV_STATE_TERMINATION_NOT_MET:
                state = NBV_STATE_VIEWPOINT_GENERATION;
                generate_viewpoints();
                break;
                
            case NBV_STATE_DONE_VIEWPOINT_GENERATION:
                state = NBV_STATE_VIEWPOINT_EVALUATION;
                evaluate_viewpoints();
                break;
            
            case NBV_STATE_DONE_VIEWPOINT_EVALUATION:
                state = NBV_STATE_MOVING;
                set_waypoint();
                break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


// Update global position of UGV
void positionCallback(const geometry_msgs::PoseStamped& pose_msg)
{
    if (isDebug && isDebugStates){
        std::cout << cc_magenta << "Grabbing location\n" << cc_reset;
    }
    
    // Save UGV pose
    mobile_base_pose = pose_msg.pose;
    
    /*
    // Save UGV pose
    mobile_base_pose_prev = mobile_base_pose;
    mobile_base_pose = pose_msg.pose[index];

    // Publish
    geometry_msgs::PoseStamped ps;
    ps.pose = mobile_base_pose;
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time::now();

    pub_pose.publish(ps);
    */
}

void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    if (isDebug && isDebugStates){
        std::cout << cc_green << "SENSING\n" << cc_reset;
    }
    
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr;

    // == Convert to pcl pointcloud
    pcl::fromROSMsg (*cloud_msg, cloud);
    cloud_ptr = cloud.makeShared();

    //std::cout << cc_blue << "Number of points detected: " << cloud_ptr->points.size() << "\n" << cc_reset;
    
    // == Remove NAN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_ptr,*cloud_ptr, indices);
    //std::cout << cc_blue << "Number of points remaining: " << cloud_ptr->points.size() << "\n" << cc_reset;
    
    
    
    // == Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_ptr);
    sor.setLeafSize (pc_res, pc_res, pc_res);
    sor.filter (*cloud_filtered);
    
    //std::cout << cc_blue << "Number of points remaining: " << cloud_filtered->points.size() << "\n" << cc_reset;
    
    // == Transform
    try{
        // Listen for transform
        tf::StampedTransform transform;
        listener->lookupTransform("world", "iris/xtion_sensor/camera_depth_optical_frame", ros::Time(0), transform);
        
        // == Convert tf:Transform to Eigen::Matrix4d
        Eigen::Matrix4d tf_eigen;
        
        Eigen::Vector3d T1(
            transform.getOrigin().x(),
            transform.getOrigin().y(),
            transform.getOrigin().z()
        );
        
        Eigen::Matrix3d R;
        tf::Quaternion qt = transform.getRotation();
        tf::Matrix3x3 R1(qt);
        tf::matrixTFToEigen(R1,R);
        
        // Set
        tf_eigen.setZero ();
        tf_eigen.block (0, 0, 3, 3) = R;
        tf_eigen.block (0, 3, 3, 1) = T1;
        tf_eigen (3, 3) = 1;
        
        // == Transform point cloud
        pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, tf_eigen);
        
        // == Add filtered to global
        addToGlobalCloud(cloud_filtered);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
}


void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
    if (isDebug && isDebugStates){
        std::cout << cc_green << "MAPPING\n" << cc_reset;
    }
    
    // Initialize global cloud if not already done so
    if (!globalCloudPtr){
        globalCloudPtr = cloud_in;
        return;
    }

    *globalCloudPtr += *cloud_in;


    // Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (globalCloudPtr);
    sor.setLeafSize (pc_res, pc_res, pc_res);
    sor.filter (*cloud_filtered);

    globalCloudPtr = cloud_filtered;
    
    if (isDebug){
        std::cout << cc_blue << "Number of points in global map: " << globalCloudPtr->points.size() << "\n" << cc_reset;
    }
    
    
    // Publish
    sensor_msgs::PointCloud2 cloud_msg;
    
    pcl::toROSMsg(*globalCloudPtr, cloud_msg); 	//cloud of original (white) using original cloud
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    
    pub_global_cloud.publish(cloud_msg);
}


void termination_check(){
    if (state != NBV_STATE_TERMINATION_CHECK){
        std::cout << cc_red << "ERROR: Attempt to check termination out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Checking termination condition\n" << cc_reset;
    }
    
    
    if (iteration_count > max_iterations){
        state = NBV_STATE_TERMINATION_MET;
    }
    else{
        state = NBV_STATE_TERMINATION_NOT_MET;
    }
}


void generate_viewpoints(){
    if (state != NBV_STATE_VIEWPOINT_GENERATION){
        std::cout << cc_red << "ERROR: Attempt to generate viewpoints out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Generating viewpoints\n" << cc_reset;
    }
    
    state = NBV_STATE_DONE_VIEWPOINT_GENERATION;
}


void evaluate_viewpoints(){
    if (state != NBV_STATE_VIEWPOINT_EVALUATION){
        std::cout << cc_red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Evaluating viewpoints\n" << cc_reset;
    }
    
    
    // Position
    setpoint.pose.position.x = 9 + randomDouble(-4,4);
    setpoint.pose.position.y = 7 + randomDouble(-4,4);
    setpoint.pose.position.z = 4;
    
    // Orientation
    double yaw = randomDouble(-3.14, 3.14);
    yaw = -90;
    tf::Quaternion tf_q;
    tf_q = tf::createQuaternionFromYaw(yaw);
    
    setpoint.pose.orientation.x = tf_q.getX();
    setpoint.pose.orientation.y = tf_q.getY();
    setpoint.pose.orientation.z = tf_q.getZ();
    setpoint.pose.orientation.w = tf_q.getW();
    
    state = NBV_STATE_DONE_VIEWPOINT_EVALUATION;
}


void set_waypoint(){
    if (state != NBV_STATE_MOVING){
        std::cout << cc_red << "ERROR: Attempt to move vehicle out of order\n" << cc_reset;
        return;
    }
    if (isDebug && isDebugStates){
        std::cout << cc_green << "Moving (setting waypoints)\n" << cc_reset;
    }
    
    
    // Publish pose (http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
    setpoint.header.frame_id = "base_footprint";
    setpoint.header.stamp = ros::Time::now();
    pub_setpoint.publish(setpoint);
    
    
    // Convert setpoint to world frame
    //@todo: temp fix. Seems there's a transform here
    geometry_msgs::Pose setpoint_world;
    setpoint_world.position.x = setpoint.pose.position.y;
    setpoint_world.position.y =-setpoint.pose.position.x;
    setpoint_world.position.z = setpoint.pose.position.z;
    
    setpoint_world.orientation.x = setpoint.pose.orientation.x;
    setpoint_world.orientation.y = setpoint.pose.orientation.y;
    setpoint_world.orientation.z = setpoint.pose.orientation.z;
    setpoint_world.orientation.w = setpoint.pose.orientation.w;
    
    // Wait till we've reached the waypoint
    ros::Rate rate(10);
    while(ros::ok() && !isNear(setpoint_world, mobile_base_pose)){
        if (isDebug){
            std::cout << cc_green << "Moving to destination. " <<
                "Distance to target: " << getDistance(setpoint_world, mobile_base_pose) <<
                "\tAngle to target: " << getAngularDistance(setpoint_world, mobile_base_pose) << "\n" << cc_reset;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    std::cout << cc_green << "Done moving\n" << cc_reset;
    
    state = NBV_STATE_DONE_MOVING;
}

bool isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current){
    if (
        getDistance(p_target, p_current) < distance_threshold &&
        fabs(getAngularDistance(p_target, p_current)) < angular_threshold){
        return true;
    }
        
    return false;
}

double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
	return sqrt(
        (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
        (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
        (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    double roll1, pitch1, yaw1;
    double roll2, pitch2, yaw2;
    
    tf::Quaternion q1 (
        p1.orientation.x,
        p1.orientation.y,
        p1.orientation.z,
        p1.orientation.w
        );
        
    tf::Quaternion q2 (
        p2.orientation.x,
        p2.orientation.y,
        p2.orientation.z,
        p2.orientation.w
        );
    
	tf::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
    tf::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);
    
    //@todo: temp fix. there's a rotation here
    yaw2 += M_PI_2;
    
    // Set differnce from -pi to pi
    double yaw_diff = fmod(yaw1 - yaw2, 2*M_PI);
    if (yaw_diff > M_PI){
        //std::cout << cc_green << "Decreased by 360: \n";
        yaw_diff = yaw_diff - 2*M_PI;
    }
    else if (yaw_diff < -M_PI){
        //std::cout << cc_green << "Increased by 360: \n";
        yaw_diff = yaw_diff + 2*M_PI;
    }
        
    //std::cout << cc_green << "Yaw1: " << yaw1*180/M_PI << "\tYaw2: " << yaw2*180/M_PI << "\n" << cc_reset;
    //std::cout << cc_green << "Yaw difference: " << yaw_diff*180/M_PI << "\n" << cc_reset;
    
    return yaw_diff;
}

void transformCam2Robot (const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
    if (&cloud_in != &cloud_out) {
        cloud_out = cloud_in;
    }

    /*
    // Sensor settings
    Eigen::Vector3d rpy(0, 0.18, 3.14);
    Eigen::Vector4d xyz(mobile_base_pose.position.x, mobile_base_pose.position.y, mobile_base_pose.position.z, 1);

    Eigen::Matrix3d R;
    tf::Quaternion qt = tf::createQuaternionFromRPY(-rpy[1],-rpy[0],-rpy[2]);

    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);

    Eigen::Matrix4d tf_matrix;
    tf_matrix.setZero();
    tf_matrix.block (0, 0, 3, 3) = R;
    tf_matrix (3, 3) = 1;

    Eigen::Matrix4d tf_matrix_swap;
    tf_matrix_swap <<   0, 1, 0, 0,
                        0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;


    double roll, pitch, yaw;
    tf::Quaternion q(
        mobile_base_pose.orientation.x,
        mobile_base_pose.orientation.y,
        mobile_base_pose.orientation.z,
        mobile_base_pose.orientation.w
    );
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    tf::Quaternion qt2 = tf::createQuaternionFromRPY(-rpy[1], -yaw, M_PI);

    tf::Matrix3x3 R2(qt2);
    tf::matrixTFToEigen(R2,R);

    Eigen::Matrix4d tf_final;
    tf_final.setZero();
    tf_final.block (0, 0, 3, 3) << tf_matrix.block (0, 0, 3, 3) * R;
    //tf_matrix.block (0, 0, 3, 3) = R;//.transpose();
    tf_final.block (0, 3, 4, 1) << tf_matrix * tf_matrix_swap * xyz;
    //tf_matrix (3, 3) = 1;

    //tf_final = tf_matrix * tf_matrix_swap * tf_final;

    // Perfrom transformation
    for (size_t i = 0; i < cloud_in.points.size (); i++)
    {
        Eigen::Vector4d pt(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z, 1);
        Eigen::Vector4d pt_tf = tf_final*pt;

        cloud_out.points[i].x = pt_tf[0];
        cloud_out.points[i].y = pt_tf[1];
        cloud_out.points[i].z = pt_tf[2];
    }
    */
}
