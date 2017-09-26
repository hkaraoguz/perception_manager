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
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <color_segmentation/SegmentArray.h>

#include "perception_manager/TabletopObject.h"
#include "perception_manager/QueryObjects.h"
//#include <perception_manager/QueryObjectsRequest.h>
//#include <perception_manager/QueryObjectsResponse.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

PointCloudRGB::Ptr cloud (new PointCloudRGB);

int workspace_min_x = 0;
int workspace_max_x = 0;

int workspace_min_y = 0;
int workspace_max_y = 0;

int table_topleft_x = 0;
int table_topleft_y = 0;

bool visualize = false;

cv::Point2f anchorpose;


// These are the parameters required to align the object positions w.r.t desired frame of reference
double roll_angle = 0.0;

double pitch_angle = 0.0;

double yaw_angle = 0.0;

double workspace_metric_offset_x = 0;

double workspace_metric_offset_y = 0;

double workspace_metric_offset_z = 0;

using namespace std;


vector<perception_manager::TabletopObject> tabletop_objects;


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

void saveObjectPositions(vector<perception_manager::TabletopObject> objects, cv::Point2f anchorpose)
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
        for(size_t i =0; i < objects.size() ; i++)
        {
            //float diffx = workspace_metric_offset_x-(anchorpose.x-poses[i].x  );
            //float diffy = workspace_metric_offset_y-(anchorpose.y-poses[i].y );
            // std::cout<<anchorpose.x<<" "<<anchorpose.y<<std::endl;
            // std::cout<<objects[i].metricposcenterx<<" "<<objects[i].metricposcentery<<std::endl;

            float diffx = (objects[i].metricposcenterx- workspace_metric_offset_x);
            float diffy = (objects[i].metricposcentery- workspace_metric_offset_y);
            stream<<-diffy<<" "<<-diffx<<"\n\n";
            //std::cout<<diffx<<" "<<diffy<<std::endl;
        }



        stream.close();

        ROS_INFO("Positions Successfully Saved!");


    }
    else
    {
        ROS_ERROR("Text file cannot be opened!!");

    }






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
perception_manager::TabletopObject createTableTopObjectFromColorSegment(const color_segmentation::Segment& segment,const PointCloudRGB& cloud, int id )
{
    perception_manager::TabletopObject object;

    // Right now the object is invalid
    object.id = -1;

    vector<float> center_coordinates = calculateCenterPosition(segment.pixelposcenterx,segment.pixelposcentery,cloud);//getPointCloudCoordinates(segment.pixelposcenterx,segment.pixelposcentery,cloud);

    if(center_coordinates[0] == -999.0) return object;

    //std::cout<<anchorpose.x<<" "<<anchorpose.y<<std::endl;

    object.metricposcenterx = workspace_metric_offset_x + (center_coordinates[0] - anchorpose.x);

    object.metricposcentery = workspace_metric_offset_y + (center_coordinates[1] - anchorpose.y);

    object.metricpostablecenterx = -(center_coordinates[1] - anchorpose.y);

    object.metricpostablecentery = -(center_coordinates[0] - anchorpose.x);


    object.pixelposcenterx = segment.pixelposcenterx;
    object.pixelposcentery = segment.pixelposcentery;

    object.id = id;

    object.image = segment.image;
    object.angle = angle2rad(segment.angle);
    object.averagehue = segment.averagehue;
    object.pixelhull = segment.pixelhull;

    vector<float> cordx(segment.pixelhull.size());
    vector<float> cordy(segment.pixelhull.size());

    float min_x = 10000;
    float max_x = 0;

    float min_y = 10000;
    float max_y = 0;

    int min_x_index=0;
    int max_x_index=0;

    int min_y_index=0;
    int max_y_index=0;

    for(size_t i = 0 ; i < segment.pixelhull.size(); i++)
    {
        cordx[i] = segment.pixelhull[i].x;
        cordy[i] = segment.pixelhull[i].y;

        if(cordx[i] < min_x)
        {
            min_x = cordx[i];
            min_x_index = i;

        }

        if(cordx[i] > max_x)
        {
            max_x = cordx[i];
            max_x_index = i;

        }

        if(cordy[i] < min_y)
        {
            min_y= cordy[i];
            min_y_index = i;

        }

        if(cordy[i] > max_y)
        {
            min_y = cordy[i];
            min_y_index = i;

        }

    }

    /* int min_x = (int)*std::min_element(cordx.begin(),cordx.end());
    int max_x = (int)*std::max_element(cordx.begin(),cordx.end());

     std::cout<<min_x<<" "<<max_x<<std::endl;

    int min_y = (int)*std::min_element(cordy.begin(),cordy.end());
    int max_y = (int)*std::max_element(cordy.begin(),cordy.end());*/


    vector<float> coordinates_min_x = getPointCloudCoordinates(segment.pixelhull[min_x_index].x,segment.pixelhull[min_x_index].y,cloud);
    vector<float> coordinates_max_x = getPointCloudCoordinates(segment.pixelhull[max_x_index].x,segment.pixelhull[max_x_index].y,cloud);

    if(coordinates_min_x[0] != -999.0 && coordinates_max_x[0] != -999.0 )
    {
        double width = fabs(coordinates_max_x[0] - coordinates_min_x[0]);

        //width*=fabs(asin(angle2rad(segment.angle)));

        object.width = width;

        // double length = fabs(coordinates_max[1] - coordinates_min[1]);

        //length*=fabs(acos(angle2rad(segment.angle)));

        // object.length = length;

    }

    vector<float> coordinates_min_y = getPointCloudCoordinates(segment.pixelhull[min_y_index].x,segment.pixelhull[min_y_index].y,cloud);
    vector<float> coordinates_max_y = getPointCloudCoordinates(segment.pixelhull[max_y_index].x,segment.pixelhull[max_y_index].y,cloud);

    if(coordinates_min_y[0] != -999.0 && coordinates_max_y[0] != -999.0 )
    {
        double length = fabs(coordinates_max_y[1] - coordinates_min_y[1]);

        //width*=fabs(asin(angle2rad(segment.angle)));

        object.length = length;

        // double length = fabs(coordinates_max[1] - coordinates_min[1]);

        //length*=fabs(acos(angle2rad(segment.angle)));

        // object.length = length;

    }


    return object;

}
bool querySceneService(perception_manager::QueryObjects::Request & req, perception_manager::QueryObjects::Response& res)
{

    res.objects = tabletop_objects;


    return true;
}


void color_segments_callback(const color_segmentation::SegmentArrayConstPtr& segments)
{

    tabletop_objects.clear();

    if(cloud && cloud->points.size() > 0)
    {

        vector<cv::Point2f> positions(segments->segments.size());


        int indexWorkspaceTopLeft = (table_topleft_y)*cloud->width + table_topleft_x;

        if(indexWorkspaceTopLeft >= cloud->points.size())
        {
            ROS_ERROR("The index of workspace top left corner is not valid! Quitting ");
            ros::shutdown();
            return;
        }

        anchorpose.x = cloud->points[indexWorkspaceTopLeft].x;
        anchorpose.y = cloud->points[indexWorkspaceTopLeft].y;

        if(anchorpose.x != anchorpose.x)
        {
            ROS_WARN("NaN received for the table top left corner position! Cannot analyze the scene...");
            return;
        }

        int count = 0;

        for(size_t k = 0; k < segments->segments.size(); k++)
        {
            perception_manager::TabletopObject object = createTableTopObjectFromColorSegment(segments->segments[k],*cloud,count);
            // std::cout<<count<<std::endl;
            // It is a valid object
            if(object.id >= 0)
            {
                tabletop_objects.push_back(object);
                count += 1;
            }


            /*  int index = ((int)segments->segments[k].pixelposcentery)*cloud->width;
            index +=(int)segments->segments[k].pixelposcenterx;

            // If we can determine the position of the segment
            if(cloud->points[index].x != cloud->points[index].x)
            {


                // std::cout<<index<<std::endl;
                // std::cout<<index1<<std::endl;
                std::cout<<k<<std::endl;
                std::cout<<cloud->points[index].x<<" "<<cloud->points[index].y<<" "<<cloud->points[index].z<<std::endl;



                std::cout<<cloud->points[indexWorkspaceTopLeft].x<<" "<<cloud->points[indexWorkspaceTopLeft].y<<" "<<cloud->points[indexWorkspaceTopLeft].z<<std::endl;

                positions[k].x = cloud->points[index].x;
                positions[k].y = cloud->points[index].y;





            }*/
        }

        for(size_t i=0 ; i < tabletop_objects.size(); i++)
        {
            /*   ROS_INFO("Tabletop Object with id: %d metric pos x: %.2f metric pos y: %.2f angle: %.2f. Width: %.2f Height %.2f",
                     tabletop_objects[i].id, tabletop_objects[i].metricposcenterx, tabletop_objects[i].metricposcentery,tabletop_objects[i].angle,tabletop_objects[i].width,tabletop_objects[i].length);
 */
            if(visualize)
            {

                boost::shared_ptr<sensor_msgs::Image> image (new sensor_msgs::Image(tabletop_objects[i].image));

                cv::imshow("Object",cv_bridge::toCvShare(image, "bgr8")->image);

                cv::waitKey(1);

                ros::Duration(1.0).sleep();
            }
        }

        //  saveObjectPositions(tabletop_objects,anchorpose);

    }
    else
    {
        ROS_WARN("No point cloud received!! Cannot analyze the scene...");
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
    pnh.getParam("visualize",visualize);


    ros::Subscriber pcl_sub = nh.subscribe<PointCloudRGB>("/kinect2/hd/points", 1, cloud_callback);

    ros::Subscriber segment_sub = nh.subscribe<color_segmentation::SegmentArray>("color_segmentation/segments", 1, color_segments_callback);

    ros::ServiceServer service = nh.advertiseService("perception_manager/query_objects",querySceneService);

    ROS_INFO("Perception Manager is running...");

    if(visualize)
    {
        cv::namedWindow("Object");
        cv::startWindowThread();
    }
    ros::spin();

    cv::destroyWindow("Object");

    return 0;


}
