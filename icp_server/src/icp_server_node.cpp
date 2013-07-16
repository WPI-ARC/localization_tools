#include <ros/ros.h>
#include <math.h>
#include <string>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

#include "icp_server_msgs/RunICP.h"

// PCL specific includes
#include <iostream>
#include <fstream>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#define _USE_MATH_DEFINES

ros::Publisher g_debug_input_cloud_publisher;
ros::Publisher g_debug_target_cloud_publisher;
ros::Publisher g_debug_aligned_cloud_publisher;
bool g_debug = true;

pcl::PointCloud<pcl::PointXYZ> PreparePointCloud(sensor_msgs::PointCloud2 ros_pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ> converted_cloud;
    pcl::fromROSMsg(ros_pointcloud, converted_cloud);
    return converted_cloud;
}

pcl::PointCloud<pcl::PointXYZ> CropPointCloudtoBoundingBox(pcl::PointCloud<pcl::PointXYZ> input_cloud, geometry_msgs::Transform bounding_volume_origin, geometry_msgs::Vector3 bounding_volume_dimensions)
{
    double min_X = bounding_volume_origin.translation.x - (bounding_volume_dimensions.x / 2.0);
    double min_Y = bounding_volume_origin.translation.y - (bounding_volume_dimensions.y / 2.0);
    double min_Z = bounding_volume_origin.translation.z - (bounding_volume_dimensions.z / 2.0);
    double max_X = bounding_volume_origin.translation.x + (bounding_volume_dimensions.x / 2.0);
    double max_Y = bounding_volume_origin.translation.y + (bounding_volume_dimensions.y / 2.0);
    double max_Z = bounding_volume_origin.translation.z + (bounding_volume_dimensions.z / 2.0);
    pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
    for (int i = 0; i < input_cloud.size(); i++)
    {
        pcl::PointXYZ point = input_cloud[i];
        if ((point.x >= min_X) && (point.x <= max_X) && (point.y >= min_Y) && (point.y <= max_Y) && (point.z >= min_Z) && (point.z <= max_Z))
        {
            cropped_cloud.push_back(point);
        }
    }
    return cropped_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PreparePointCloud(sensor_msgs::PointCloud2 ros_pointcloud, geometry_msgs::Transform cloud_frame)
{
    pcl::PointCloud<pcl::PointXYZ> converted_cloud;
    pcl::fromROSMsg(ros_pointcloud, converted_cloud);
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(cloud_frame.rotation, orientation);
    tf::Matrix3x3 rotation_matrix(orientation);
    Eigen::Matrix4f transform_matrix;
    transform_matrix(0, 0) = rotation_matrix[0][0];
    transform_matrix(0, 1) = rotation_matrix[0][1];
    transform_matrix(0, 2) = rotation_matrix[0][2];
    transform_matrix(0, 3) = cloud_frame.translation.x;
    transform_matrix(1, 0) = rotation_matrix[1][0];
    transform_matrix(1, 1) = rotation_matrix[1][1];
    transform_matrix(1, 2) = rotation_matrix[1][2];
    transform_matrix(1, 3) = cloud_frame.translation.y;
    transform_matrix(2, 0) = rotation_matrix[2][0];
    transform_matrix(2, 1) = rotation_matrix[2][1];
    transform_matrix(2, 2) = rotation_matrix[2][2];
    transform_matrix(2, 3) = cloud_frame.translation.z;
    transform_matrix(3, 0) = 0.0;
    transform_matrix(3, 1) = 0.0;
    transform_matrix(3, 2) = 0.0;
    transform_matrix(3, 3) = 1.0;
    pcl::transformPointCloud(converted_cloud, converted_cloud, transform_matrix);
    return converted_cloud;
}

pcl::PointCloud<pcl::PointXYZ> LoadPointCloud(std::string pcd_resource, geometry_msgs::Transform cloud_frame)
{
    std::string resource_path = pcd_resource;
    pcl::PointCloud<pcl::PointXYZ> loaded_cloud;
    if (pcl::io::loadPCDFile(resource_path.c_str(), loaded_cloud) != 0)
    {
        throw std::invalid_argument("Pointcloud file could not be loaded");
    }
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(cloud_frame.rotation, orientation);
    tf::Matrix3x3 rotation_matrix(orientation);
    Eigen::Matrix4f transform_matrix;
    transform_matrix(0, 0) = rotation_matrix[0][0];
    transform_matrix(0, 1) = rotation_matrix[0][1];
    transform_matrix(0, 2) = rotation_matrix[0][2];
    transform_matrix(0, 3) = cloud_frame.translation.x;
    transform_matrix(1, 0) = rotation_matrix[1][0];
    transform_matrix(1, 1) = rotation_matrix[1][1];
    transform_matrix(1, 2) = rotation_matrix[1][2];
    transform_matrix(1, 3) = cloud_frame.translation.y;
    transform_matrix(2, 0) = rotation_matrix[2][0];
    transform_matrix(2, 1) = rotation_matrix[2][1];
    transform_matrix(2, 2) = rotation_matrix[2][2];
    transform_matrix(2, 3) = cloud_frame.translation.z;
    transform_matrix(3, 0) = 0.0;
    transform_matrix(3, 1) = 0.0;
    transform_matrix(3, 2) = 0.0;
    transform_matrix(3, 3) = 1.0;
    pcl::transformPointCloud(loaded_cloud, loaded_cloud, transform_matrix);
    return loaded_cloud;
}

pcl::PointCloud<pcl::PointXYZ> GenerateInputCloud(std::string mesh_resource, double mesh_scaling, geometry_msgs::Transform input_transform)
{
    pcl::PointCloud<pcl::PointXYZ> generated_cloud;
    ;
    std::invalid_argument("Method not implemented");
    ;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(input_transform.rotation, orientation);
    tf::Matrix3x3 rotation_matrix(orientation);
    Eigen::Matrix4f transform_matrix;
    transform_matrix(0, 0) = rotation_matrix[0][0];
    transform_matrix(0, 1) = rotation_matrix[0][1];
    transform_matrix(0, 2) = rotation_matrix[0][2];
    transform_matrix(0, 3) = input_transform.translation.x;
    transform_matrix(1, 0) = rotation_matrix[1][0];
    transform_matrix(1, 1) = rotation_matrix[1][1];
    transform_matrix(1, 2) = rotation_matrix[1][2];
    transform_matrix(1, 3) = input_transform.translation.y;
    transform_matrix(2, 0) = rotation_matrix[2][0];
    transform_matrix(2, 1) = rotation_matrix[2][1];
    transform_matrix(2, 2) = rotation_matrix[2][2];
    transform_matrix(2, 3) = input_transform.translation.z;
    transform_matrix(3, 0) = 0.0;
    transform_matrix(3, 1) = 0.0;
    transform_matrix(3, 2) = 0.0;
    transform_matrix(3, 3) = 1.0;
    pcl::transformPointCloud(generated_cloud, generated_cloud, transform_matrix);
    return generated_cloud;
}

pcl::PointCloud<pcl::PointXYZ> GenerateInputCloud(shape_msgs::SolidPrimitive primitive_shape, geometry_msgs::Vector3 discretization, geometry_msgs::Transform input_transform)
{
    double Xdisc = discretization.x;
    double Ydisc = discretization.y;
    double Zdisc = discretization.z;
    pcl::PointCloud<pcl::PointXYZ> generated_cloud;
    if (primitive_shape.type == primitive_shape.BOX)
    {
        double X = primitive_shape.dimensions[0];
        double Y = primitive_shape.dimensions[1];
        double Z = primitive_shape.dimensions[2];
        int points_per_dim_X = (int)(X / Xdisc);
        int points_per_dim_Y = (int)(Y / Ydisc);
        int points_per_dim_Z = (int)(Z / Zdisc);
        for (int i = 0; i < points_per_dim_X; i++)
        {
            for (int j = 0; j < points_per_dim_Y; j++)
            {
                for (int k = 0; k < points_per_dim_Z; k++)
                {
                    if ((i == 0 || i == (points_per_dim_X - 1)) || (j == 0 || j == (points_per_dim_Y - 1)) || (k == 0 || k == (points_per_dim_Z - 1)))
                    {
                        double x = (double)i * Xdisc;
                        double y = (double)j * Ydisc;
                        double z = (double)k * Zdisc;
                        pcl::PointXYZ new_point(x, y, z);
                        generated_cloud.push_back(new_point);
                    }
                }
            }
        }
    }
    else if (primitive_shape.type == primitive_shape.CYLINDER)
    {
        double radius = primitive_shape.dimensions[0];
        double height = primitive_shape.dimensions[1];
        int points_per_dim_A = (int)((2.0 * M_PI) / Xdisc);
        int points_per_dim_H = (int)(height / Zdisc);
        int points_per_dim_R = (int)(radius / Ydisc);
        for (int a = 0; a < points_per_dim_A; a++)
        {
            for (int h = 0; h < points_per_dim_H; h++)
            {
                if (h == 0 || h == (points_per_dim_H - 1))
                {
                    for (int r = 0; r < points_per_dim_R; r++)
                    {
                        double R = (double)r * Ydisc;
                        double A = (double)a * Xdisc;
                        double x = R * cos(A);
                        double y = R * sin(A);
                        double z = (double)h * Zdisc;
                        pcl::PointXYZ new_point(x, y, z);
                        generated_cloud.push_back(new_point);
                    }
                }
                else
                {
                    double A = (double)a * Xdisc;
                    double x = radius * cos(A);
                    double y = radius * sin(A);
                    double z = (double)h * Zdisc;
                    pcl::PointXYZ new_point(x, y, z);
                    generated_cloud.push_back(new_point);
                }
            }
        }
    }
    else if (primitive_shape.type == primitive_shape.SPHERE)
    {
        double radius = primitive_shape.dimensions[0];
        int points_per_dim_A = (int)((2.0 * M_PI) / Xdisc);
        for (int a = 0; a < points_per_dim_A; a++)
        {
            double pan_angle = (double)a * Xdisc;
            double tilt_angle = (double)a * Xdisc;
            double projected_radius = cos(tilt_angle) * radius;
            double x = projected_radius * cos(pan_angle);
            double y = projected_radius * sin(pan_angle);
            double z = radius * sin(tilt_angle);
            pcl::PointXYZ new_point(x, y, z);
            generated_cloud.push_back(new_point);
        }
    }
    else if (primitive_shape.type == primitive_shape.CONE)
    {
        throw std::invalid_argument("Cones are not currently supported");
    }
    else
    {
        throw std::invalid_argument("Invalid primitive shape type requested");
    }
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(input_transform.rotation, orientation);
    tf::Matrix3x3 rotation_matrix(orientation);
    Eigen::Matrix4f transform_matrix;
    transform_matrix(0, 0) = rotation_matrix[0][0];
    transform_matrix(0, 1) = rotation_matrix[0][1];
    transform_matrix(0, 2) = rotation_matrix[0][2];
    transform_matrix(0, 3) = input_transform.translation.x;
    transform_matrix(1, 0) = rotation_matrix[1][0];
    transform_matrix(1, 1) = rotation_matrix[1][1];
    transform_matrix(1, 2) = rotation_matrix[1][2];
    transform_matrix(1, 3) = input_transform.translation.y;
    transform_matrix(2, 0) = rotation_matrix[2][0];
    transform_matrix(2, 1) = rotation_matrix[2][1];
    transform_matrix(2, 2) = rotation_matrix[2][2];
    transform_matrix(2, 3) = input_transform.translation.z;
    transform_matrix(3, 0) = 0.0;
    transform_matrix(3, 1) = 0.0;
    transform_matrix(3, 2) = 0.0;
    transform_matrix(3, 3) = 1.0;
    pcl::transformPointCloud(generated_cloud, generated_cloud, transform_matrix);
    return generated_cloud;
}


bool ICPServiceCB(icp_server_msgs::RunICP::Request& req, icp_server_msgs::RunICP::Response& res)
{
    icp_server_msgs::ICPRequest request = req.Request;
    icp_server_msgs::ICPResponse response = res.Response;
    pcl::PointCloud<pcl::PointXYZ> target_cloud = PreparePointCloud(request.TargetPointcloud);
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    if (request.InputType == request.POINTCLOUD)
    {
        input_cloud = PreparePointCloud(request.InputPointCloud, request.InputTransform);
        if (request.UseBoundingVolume)
        {
            input_cloud = CropPointCloudtoBoundingBox(input_cloud, request.BoundingVolumeOrigin, request.BoundingVolumeDimensions);
        }
    }
    else if (request.InputType == request.POINTCLOUDFILE)
    {
        try
        {
            input_cloud = LoadPointCloud(request.InputResource, request.InputTransform);
        }
        catch (...)
        {
            ROS_ERROR("Requested PCD resource could not be loaded");
            return false;
        }
    }
    else if (request.InputType == request.MESH)
    {
        try
        {
            input_cloud = GenerateInputCloud(request.InputResource, request.InputMeshScaling, request.InputTransform);
        }
        catch (...)
        {
            ROS_ERROR("Requested mesh resource could not be loaded");
            return false;
        }
    }
    else if (request.InputType == request.SHAPE)
    {
        try
        {
            input_cloud = GenerateInputCloud(request.InputShape, request.Discretization, request.InputTransform);
        }
        catch (...)
        {
            ROS_ERROR("Requested shape is invalid");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Unrecognized InputType: %d [options are POINTCLOUD, MESH, or SHAPE]", request.InputType);
        return false;
    }
    if (g_debug)
    {
        sensor_msgs::PointCloud2 debug_input_cloud;
        sensor_msgs::PointCloud2 debug_target_cloud;
        pcl::toROSMsg(input_cloud, debug_input_cloud);
        pcl::toROSMsg(target_cloud, debug_input_cloud);
        g_debug_input_cloud_publisher.publish(debug_input_cloud);
        g_debug_target_cloud_publisher.publish(debug_target_cloud);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target_cloud(&target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_cloud(&input_cloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_solver;
    icp_solver.setMaxCorrespondenceDistance(request.MaxICPCoorespondenceDistance);
    icp_solver.setMaximumIterations(request.MaxICPIterations);
    icp_solver.setInputTarget(pcl_target_cloud);
    icp_solver.setInputCloud(pcl_input_cloud);
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    icp_solver.align(final_cloud);
    Eigen::Matrix4f final_transform_matrix = icp_solver.getFinalTransformation();
    response.Converged = icp_solver.hasConverged();
    response.AlignmentFitness = icp_solver.getFitnessScore();
    pcl::toROSMsg(final_cloud, response.AlignedCloud);
    if (g_debug)
    {
        g_debug_aligned_cloud_publisher.publish(response.AlignedCloud);
    }
    // Convert the PCL transform to geometry_msgs/Transform
    tf::Matrix3x3 final_rotation_matrix(final_transform_matrix(0,0), final_transform_matrix(0,1), final_transform_matrix(0,2), final_transform_matrix(1,0), final_transform_matrix(1,1), final_transform_matrix(1,2), final_transform_matrix(2,0), final_transform_matrix(2,1), final_transform_matrix(2,2));
    geometry_msgs::Transform final_transform;
    final_transform.translation.x = final_transform_matrix(0, 3);
    final_transform.translation.y = final_transform_matrix(1, 3);
    final_transform.translation.z = final_transform_matrix(2, 3);
    tf::Quaternion final_rotation;
    final_rotation_matrix.getRotation(final_rotation);
    final_transform.rotation.x = final_rotation.x();
    final_transform.rotation.y = final_rotation.y();
    final_transform.rotation.z = final_rotation.z();
    final_transform.rotation.w = final_rotation.w();
    response.AlignmentTransform = final_transform;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_server");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ROS_INFO("Starting ICP server...");
    nhp.getParam("enable_debug", g_debug);
    if (g_debug)
    {
        ROS_INFO("ICP server debug enabled");
    }
    else
    {
        ROS_INFO("ICP server debug not enabled");
    }
    g_debug_input_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(nh.getNamespace() + "/input_pointcloud", 1);
    g_debug_target_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(nh.getNamespace() + "/target_pointcloud", 1);
    g_debug_aligned_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(nh.getNamespace() + "/aligned_pointcloud", 1);
    ros::ServiceServer server = nh.advertiseService(nh.getNamespace() + "/run_icp", ICPServiceCB);
    ROS_INFO("ICP server loaded");
    while (ros::ok())
    {
        ros::spinOnce();
    }
    ROS_INFO("Shutting down ICP server");
    return 0;
}
