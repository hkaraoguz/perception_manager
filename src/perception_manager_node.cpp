#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



#include <color_segmentation/SegmentArray.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

PointCloudRGB::Ptr cloud (new PointCloudRGB);

int workspace_min_x = 0;
int workspace_max_x = 0;

int workspace_min_y = 0;
int workspace_max_y = 0;

int table_topleft_x = 0;
int table_topleft_y = 0;




// These are the parameters required to align the object positions w.r.t desired frame of reference
double roll_angle = 0.0;

double pitch_angle = 0.0;

double yaw_angle = 0.0;

double workspace_metric_offset_x = 0;

double workspace_metric_offset_y = 0;

double workspace_metric_offset_z = 0;

using namespace std;





// Get the home path to save data
string getHomePath()
{
    uid_t uid = getuid();
    struct passwd *pw = getpwuid(uid);

    if (pw == NULL) {
        ROS_ERROR("Failed to get homedir. Cannot save configuration file\n");
        return "";
    }

    // printf("%s\n", pw->pw_dir);
    string str(pw->pw_dir);
    return str;

}


bool readWorkspaceConfig(int* minX, int* maxX, int* minY, int* maxY, double* roll, double* pitch, double* yaw,int *topleft_x, int *topleft_y)
{
    string configpath = getHomePath();

    configpath += "/.ros/workspace_segmentation/";

    configpath += "workspace.txt";

    ifstream stream(configpath.data());

    if(stream.is_open())
    {
        string str;
        int count = 0;
        while(getline(stream, str))
        {

            std::istringstream ss(str);

            //std::cout<<str<<endl;

            switch(count)
            {
            case 0:
                *topleft_x = atoi(str.data());
            case 1:
                *topleft_y = atoi(str.data());
            case 2:
                *minX = atoi(str.data());
            case 3:
                *maxX = atoi(str.data());
            case 4:
                *minY  = atoi(str.data());
            case 5:
                *maxY = atoi(str.data());
            case 6:
                *roll = atof(str.data());
            case 7:
                *pitch = atof(str.data());
            case 8:
                *yaw = atof(str.data());
            default:
                break;

            }

            count++;

        }


        stream.close();



    }
    else
    {
        return false;
    }

    return true;
}


void cloud_callback(PointCloudRGB::ConstPtr msg)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    *cloud = *msg;


    Eigen::Affine3f transmat = pcl::getTransformation(0,0,0,roll_angle,pitch_angle,yaw_angle);


    pcl::transformPointCloud(*cloud,*cloud,transmat);


}

void saveObjectPositions(vector<cv::Point2f> poses, cv::Point2f anchorpose)
{

    string configpath = getHomePath();

    configpath += "/perception_manager/";

    boost::filesystem::path dir(configpath);

    if(!(boost::filesystem::exists(dir)))
    {
        std::cout<<"Doesn't Exists"<<std::endl;
    }

    if (boost::filesystem::create_directory(dir))
        std::cout << "....Successfully Created !" << std::endl;

    stringstream ss;

    ss<<ros::Time::now();

    configpath += "positions_";
    configpath += ss.str();
    configpath+= ".txt";

    ofstream stream(configpath.data());



    if(stream.is_open())
    {
        for(size_t i =0; i < poses.size() ; i++)
        {
            float diffx = workspace_metric_offset_x-(anchorpose.x-poses[i].x  );
            float diffy = workspace_metric_offset_y-(anchorpose.y-poses[i].y );

            stream<<diffx<<" "<<diffy<<"\n\n";
            std::cout<<diffx<<" "<<diffy<<std::endl;
        }



        stream.close();

        ROS_INFO("Positions Successfully Saved!");


    }
    else
    {
        ROS_ERROR("Text file cannot be opened!!");

    }






}




void color_segments_callback(const color_segmentation::SegmentArrayConstPtr& segments)
{
    if(cloud && cloud->points.size() > 0)
    {
        vector<cv::Point2f> positions(segments->segments.size());

        cv::Point2f anchorpose;

        int indexWorkspaceTopLeft = (table_topleft_y)*cloud->width + table_topleft_x;

        if(indexWorkspaceTopLeft >= cloud->points.size())
        {
            ROS_ERROR("The index of workspace top left corner is not valid! Quitting ");
            ros::shutdown();
            return;
        }

        anchorpose.x = cloud->points[indexWorkspaceTopLeft].x;
        anchorpose.y = cloud->points[indexWorkspaceTopLeft].y;


        for(size_t k = 0; k < segments->segments.size(); k++)
        {

            int index = ((int)segments->segments[k].pixelposcentery)*cloud->width;
            index +=(int)segments->segments[k].pixelposcenterx;


            // std::cout<<index<<std::endl;
            // std::cout<<index1<<std::endl;
            std::cout<<k<<std::endl;
            std::cout<<cloud->points[index].x<<" "<<cloud->points[index].y<<" "<<cloud->points[index].z<<std::endl;
            std::cout<<cloud->points[indexWorkspaceTopLeft].x<<" "<<cloud->points[indexWorkspaceTopLeft].y<<" "<<cloud->points[indexWorkspaceTopLeft].z<<std::endl;

            positions[k].x = cloud->points[index].x;
            positions[k].y = cloud->points[index].y;

            boost::shared_ptr<sensor_msgs::Image> image (new sensor_msgs::Image(segments->segments[k].image));

            cv::imshow("Object",cv_bridge::toCvShare(image, "bgr8")->image);

            cv::waitKey(1);

            ros::Duration(1.0).sleep();
        }

        saveObjectPositions(positions,anchorpose);

    }



}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "perception_manager_node");
    ros::NodeHandle nh;
    // Private node handle
    ros::NodeHandle pnh("~");


    if(!readWorkspaceConfig(&workspace_min_x,&workspace_max_x,&workspace_min_y,&workspace_max_y,&roll_angle,&pitch_angle,&yaw_angle,&table_topleft_x,&table_topleft_y))
    {
        ROS_WARN("Could not read workspace dimensions! Working on whole image");
    }


    pnh.getParam("workspace_metric_offset_x",workspace_metric_offset_x);
    pnh.getParam("workspace_metric_offset_y",workspace_metric_offset_y);
    pnh.getParam("workspace_metric_offset_z",workspace_metric_offset_z);


    ros::Subscriber pcl_sub = nh.subscribe<PointCloudRGB>("/kinect2/hd/points", 1, cloud_callback);

    ros::Subscriber segment_sub = nh.subscribe<color_segmentation::SegmentArray>("color_segmentation/segments", 1, color_segments_callback);

    cv::namedWindow("Object");
    cv::startWindowThread();

    ros::spin();

    cv::destroyWindow("Object");

    return 0;


}
