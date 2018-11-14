#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <ros_cpp_utils/utils.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <color_segmentation/SegmentArray.h>

#include "perception_manager/TabletopObject.h"
#include "perception_manager/GetMetricCoordinate.h"

#include <tf/transform_listener.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

PointCloudRGB::Ptr cloud (new PointCloudRGB);


using namespace std;

string cloud_topic = "/kinect2/qhd/points";

string base_frame = "/world";

tf::TransformListener *tf_listener;


vector<perception_manager::TabletopObject> tabletop_objects;

void cloud_callback(PointCloudRGB::ConstPtr msg)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    //*cloud = *msg;

    if(tf_listener->waitForTransform(base_frame, msg->header.frame_id, ros::Time::now(), ros::Duration(5.0)))
    {
        pcl_ros::transformPointCloud(base_frame, *msg, *cloud, *tf_listener);

    }
    else
    {
        ROS_ERROR("Transform from %s to %s cannot be received! Quitting...",msg->header.frame_id.data(),base_frame.data());
        ros::shutdown();
    }




    // Eigen::Affine3f transmat = pcl::getTransformation(0,0,0,roll_angle,pitch_angle,yaw_angle);


    //pcl::transformPointCloud(*cloud,*cloud,transmat);


}


vector<float> getPointCloudCoordinates(int pix_x, int pix_y,const PointCloudRGB& cloud)
{
    vector<float> coordinates(3);

    int index = (pix_y)*cloud.width;
    index += pix_x;

    if(index <0 || cloud.points.size() <= index)
    {
        coordinates[0] = -999.0;
        return coordinates;
    }

    if(cloud.points[index].x != cloud.points[index].x)
    {
        coordinates[0] = -999.0;
        return coordinates;

    }

    coordinates[0] = cloud.points[index].x;
    coordinates[1] = cloud.points[index].y;
    coordinates[2] = cloud.points[index].z;


    // std::cout<<index<<" "<<coordinates[0]<<" "<<coordinates[1]<<std::endl;


    return coordinates;



}
vector<float> calculateCenterPosition(int center_x,int center_y,const PointCloudRGB& cloud)
{
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;
    int count = 0;

    vector<float> res(3);


    vector<float> coordinates = getPointCloudCoordinates(center_x,center_y,cloud);
    if(coordinates[0] != -999.0)
    {
        sum_x += coordinates[0];
        sum_y += coordinates[1];
        sum_z += coordinates[2];
        count++;
    }
    coordinates = getPointCloudCoordinates(center_x+1,center_y,cloud);
    if(coordinates[0] != -999.0)
    {
        sum_x += coordinates[0];
        sum_y += coordinates[1];
        sum_z += coordinates[2];
        count++;
    }
    coordinates = getPointCloudCoordinates(center_x-1,center_y,cloud);
    if(coordinates[0] != -999.0)
    {
        sum_x += coordinates[0];
        sum_y += coordinates[1];
        sum_z += coordinates[2];
        count++;
    }
    coordinates = getPointCloudCoordinates(center_x,center_y+1,cloud);
    if(coordinates[0] != -999.0)
    {
        sum_x += coordinates[0];
        sum_y += coordinates[1];
        sum_z += coordinates[2];
        count++;
    }
    coordinates = getPointCloudCoordinates(center_x,center_y-1,cloud);
    if(coordinates[0] != -999.0)
    {
        sum_x += coordinates[0];
        sum_y += coordinates[1];
        sum_z += coordinates[2];
        count++;
    }

    if(count == 0){
        res[0] = -999.0;
        return res;
    }

    res[0] = sum_x/count;
    res[1] = sum_y/count;
    res[2] = sum_z/count;

    return res;

}

double angle2rad(float angle)
{
    return angle*M_PI/180;
}

bool getMetricCoordinateService(perception_manager::GetMetricCoordinate::Request & req, perception_manager::GetMetricCoordinate::Response& res)
{

    vector<float> center_coordinates = calculateCenterPosition(req.x_coord,req.y_coord,*cloud);

    geometry_msgs::PointStamped point;

    point.header.stamp = ros::Time::now ();

    point.point.x = center_coordinates[0];
    point.point.y = center_coordinates[1];
    point.point.z = center_coordinates[2];

    res.point = point;

    //if(center_coordinates[0] == -999.0) return object;


    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_to_metric_coordinates_node");

    ros::NodeHandle nh;

    // Private node handle
    ros::NodeHandle pnh("~");

    pnh.getParam("cloud_topic",cloud_topic);
    pnh.getParam("base_frame",base_frame);


    tf_listener = new tf::TransformListener(nh);

    ros::Subscriber pcl_sub = nh.subscribe<PointCloudRGB>(cloud_topic.data(), 1, cloud_callback);

    ros::ServiceServer service = nh.advertiseService("perception_manager/get_metric_coordinate",getMetricCoordinateService);

    ROS_INFO("Get Metric Coordinate Service is running...");

    ros::spin();


    return 0;


}
