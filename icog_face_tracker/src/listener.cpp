#include "../utility/pietslib.h"
#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
using namespace std;
using namespace cv;

#include <iostream>
#include <fstream>
#include <sstream>

//launch-prefix="gdb -ex run --args"
cv::Mat image;
geometry_msgs::Vector3Stamped laserField;
geometry_msgs::Vector3Stamped laserData;

void laserCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserData = msg;
}

void laserReconCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserField = msg;
}

//Callback vom USB CAM "image_raw" topic
void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
    if (!image.empty())
    {
        if (laserField.vector.z != 0)
            rectangle(image,
                      Rect(laserField.vector.x - laserField.vector.z / 2,
                           laserField.vector.y - laserField.vector.z / 2,
                           laserField.vector.z, laserField.vector.z),
                      CV_RGB(0, 0, 255), 1);
        stringstream ss;
        ss << laserData.vector.x;
        putText(
            image, "Distance [mm]: " + ss.str(),
            Point(10, 10),
            FONT_HERSHEY_PLAIN,
            1.0,
            CV_RGB(0, 255, 0),
            2);

        imshow("tracking Data", image);
        waitKey(3);
    }

    //Einlesen des Bildes aus der Kamera Topic in einen Opencv Frame Pointer
    cv_bridge::CvImagePtr cvPtr;
    try
    {
        // cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::cvtColor(cvPtr->image, image, cv::COLOR_GRAY2BGR);
}

void trackerCB(const icog_face_tracker::facegeos &msg)
{
    if (!image.empty())
    {
        icog_face_tracker::facegeos faces = msg;
        for (int i = 0; i < faces.facegeos.size(); i++)
        {
            stringstream ss;
            ss << i;
            //Save Facebox to geometryBuffer
            if (!faces.facegeos[i].facingAway)
            {
                rectangle(image, Rect(faces.facegeos[i].face.x, faces.facegeos[i].face.y, faces.facegeos[i].face.width, faces.facegeos[i].face.height),
                          CV_RGB(255, 0, 0), 1);
            }
            else
            {
                rectangle(image, Rect(faces.facegeos[i].face.x, faces.facegeos[i].face.y, faces.facegeos[i].face.width, faces.facegeos[i].face.height),
                          CV_RGB(0, 255, 0), 1);
            }
            //cout << faces.facegeos.size() << endl;
            if (!faces.facegeos[i].facingAway && !faces.facegeos[i].closeMouthTracking)
                for (int u = 0; u < faces.facegeos[i].face_points.size(); u++)
                {
                    circle(image, Point(faces.facegeos[i].face_points[u].x, faces.facegeos[i].face_points[u].y), 2,
                           CV_RGB(255, 0, 0), CV_FILLED);
                }
            putText(
                image, "Face No.: " + ss.str(),
                Point(faces.facegeos[i].face.x, faces.facegeos[i].face.y + 10),
                FONT_HERSHEY_PLAIN,
                1.0,
                CV_RGB(0, 255, 0),
                2);
        }
        if (laserField.vector.z != 0)
        {
            rectangle(image,
                      Rect(laserField.vector.x - laserField.vector.z / 2,
                           laserField.vector.y - laserField.vector.z / 2,
                           laserField.vector.z, laserField.vector.z),
                      CV_RGB(0, 0, 255), 1);
            stringstream ss;
            ss << laserData.vector.x;
            putText(
                image, "Distance [mm]: " + ss.str(),
                Point(10, 10),
                FONT_HERSHEY_PLAIN,
                1.0,
                CV_RGB(0, 255, 0),
                2);
        }

        imshow("tracking Data", image);
        waitKey(3);
    }
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/facegeos", 1, trackerCB);
    ros::Subscriber subLaserRecon = nh.subscribe("/mouthTracking/cameraCoordinates", 1, laserReconCB);
    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);
    //debugpub = nh.advertise<string>("/debug", 5);
    //den Subcriber initiieren und dem Topic /usb_cam_node/image_raw subscriben
    ros::Subscriber sub2 = nh.subscribe("/d435/InfraredImage", 1, imageCB);
    ros::spin();
}