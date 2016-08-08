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

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
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


// Create a state machine
enum class NBV_STATE {
    NOT_STARTED,
    INITIALIZING,
    IDLE,
    SENSING, DONE_SENSING,
    MAPPING, DONE_MAPPING,
    VIEWPOINT_GENERATION, DONE_VIEWPOINT_GENERATION, 
    VIEWPOINT_EVALUATION, DONE_VIEWPOINT_EVALUATION,
    PATH_PLANNING,
    MOVING, DONE_MOVING,
    TERMINATION_CHECK, TERMINATION_MET, TERMINATION_NOT_MET};
NBV_STATE state = NBV_STATE::NOT_STARTED;

/*
double randomDouble(double min, double max) {
    return ((double) random()/RAND_MAX)*(max-min) + min;
}

double rad2deg (double rad) {
    return rad*180/M_PI;
}

double getDistanceXY(pcl::PointXYZ p1, pcl::PointXYZ p2){
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
}
*/

geometry_msgs::Pose mobile_base_pose;
geometry_msgs::Pose mobile_base_pose_prev;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sensed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloudPtr;

//pcl::VoxelGrid<pcl::PointXYZRGB> occGrid;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloudPtr;

float res = 0.1f; //Voxel grid resolution
int count = 0;


// Prototype
void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
void termination_check();
void generate_viewpoints();
void evaluate_viewpoints();
void set_waypoint();

// Consts
std::string depth_topic   = "/iris/xtion_sensor/iris/xtion_sensor_camera/depth/points";
std::string position_topic = "/iris/ground_truth/pose";

// Termination variables
bool isTerminating = false;
int iteration_count = 0;
int max_iterations = 10;

int main(int argc, char **argv)
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    state = NBV_STATE::INITIALIZING;
    
    std::cout << cc_red << "BEGIN NBV LOOP\n" << cc_reset;

    ros::init(argc, argv, "nbv_loop");
    ros::NodeHandle ros_node;

    ros::Subscriber sub_kinect = ros_node.subscribe(depth_topic, 1, depthCallback);
    ros::Subscriber sub_pose = ros_node.subscribe(position_topic, 1, positionCallback);

    //pub = ros_node.advertise<sensor_msgs::PointCloud2> ("/voxgrid", 1);
    //pub_pose = ros_node.advertise<geometry_msgs::PoseStamped> ("/voxgrid/pose", 1);


    // Start the FSM
    state = NBV_STATE::IDLE;
    
    ros::Rate loop_rate(10);
    while (ros::ok() && !isTerminating)
    {
        switch(state){
            case NBV_STATE::IDLE:
            case NBV_STATE::DONE_MOVING:
                state = NBV_STATE::SENSING;
                
                iteration_count++;
                std::cout << cc_yellow << "Iteration: " << iteration_count << "\n" << cc_reset;
                break;
                
            case NBV_STATE::DONE_SENSING:
                state = NBV_STATE::MAPPING;
                addToGlobalCloud(cloud_sensed);
                break;
            
            case NBV_STATE::DONE_MAPPING:
                state = NBV_STATE::TERMINATION_CHECK;
                termination_check();
                break;
                
            case NBV_STATE::TERMINATION_MET:
                isTerminating = true;
                std::cout << cc_yellow << "Termination condition met\n" << cc_reset;
                break;
                
            case NBV_STATE::TERMINATION_NOT_MET:
                state = NBV_STATE::VIEWPOINT_GENERATION;
                generate_viewpoints();
                break;
                
            case NBV_STATE::DONE_VIEWPOINT_GENERATION:
                state = NBV_STATE::VIEWPOINT_EVALUATION;
                evaluate_viewpoints();
                break;
            
            case NBV_STATE::DONE_VIEWPOINT_EVALUATION:
                state = NBV_STATE::MOVING;
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
    std::cout << cc_magenta << "Grabbing location\n" << cc_reset;
    
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
    if (state != NBV_STATE::SENSING){
        return;
    }
    std::cout << cc_green << "SENSING\n" << cc_reset;
    
    
    //*
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Convert to pcl pointcloud
    pcl::fromROSMsg (*cloud_msg, cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr = cloud.makeShared();

    
    // @todo: anything beyond this point causes the software to crash
    // Looks like the pointcloud doesn't play nice.
    // Try checking if the cloud is even valid. List the points and their coordinates
    
    // Remove NAN points
    std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*cloudPtr,*cloudPtr, indices);

    /*
    // Perform voxelgrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (res, res, res);
    sor.filter (*cloud_filtered);
    //*/
    
    state = NBV_STATE::DONE_SENSING;
    
    /*
    // Add filtered to global
    state = NBV_STATE::MAPPING;
    addToGlobalCloud(cloud_filtered);
    */
}


void addToGlobalCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
    if (state != NBV_STATE::MAPPING){
        return;
    }
    std::cout << cc_green << "MAPPING\n" << cc_reset;
    
    // Initialize global cloud if not already done so
    /*
    if (!globalCloudPtr){
        globalCloudPtr = cloud_in;
        return;
    }

    *globalCloudPtr += *cloud_in;
    */









    /*
    // ==========
    // TRANSFORM
    // ==========
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    transformCam2Robot (*cloud_in, *transformed_cloud);

    // Initialize global cloud if not already done so
    if (!globalCloudPtr) {
        globalCloudPtr = transformed_cloud;
        return;
    }
    */

    /*
    // Preform ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputCloud(transformed_cloud);
    icp.setInputTarget(globalCloudPtr);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    *globalCloudPtr += Final;
    */

    //globalCloudPtr = transformed_cloud;
    //return;

    //*globalCloudPtr += *transformed_cloud;











    // Perform voxelgrid filtering
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (globalCloudPtr);
    sor.setLeafSize (res, res, res);
    sor.filter (*cloud_filtered);

    globalCloudPtr = cloud_filtered;
    */
    
    state = NBV_STATE::DONE_MAPPING;
}


void termination_check(){
    if (state != NBV_STATE::TERMINATION_CHECK){
        std::cout << cc_red << "ERROR: Attempt to check termination out of order\n" << cc_reset;
        return;
    }
    std::cout << cc_green << "Checking termination condition\n" << cc_reset;
    
    
    if (iteration_count > max_iterations){
        state = NBV_STATE::TERMINATION_MET;
    }
    else{
        state = NBV_STATE::TERMINATION_NOT_MET;
    }
}


void generate_viewpoints(){
    if (state != NBV_STATE::VIEWPOINT_GENERATION){
        std::cout << cc_red << "ERROR: Attempt to generate viewpoints out of order\n" << cc_reset;
        return;
    }
    std::cout << cc_green << "Generating viewpoints\n" << cc_reset;
    
    state = NBV_STATE::DONE_VIEWPOINT_GENERATION;
}


void evaluate_viewpoints(){
    if (state != NBV_STATE::VIEWPOINT_EVALUATION){
        std::cout << cc_red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc_reset;
        return;
    }
    std::cout << cc_green << "Evaluating viewpoints\n" << cc_reset;
    
    state = NBV_STATE::DONE_VIEWPOINT_EVALUATION;
}


void set_waypoint(){
    if (state != NBV_STATE::MOVING){
        std::cout << cc_red << "ERROR: Attempt to move vehicle out of order\n" << cc_reset;
        return;
    }
    std::cout << cc_green << "Moving (setting waypoints)\n" << cc_reset;
    
    state = NBV_STATE::DONE_MOVING;
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
