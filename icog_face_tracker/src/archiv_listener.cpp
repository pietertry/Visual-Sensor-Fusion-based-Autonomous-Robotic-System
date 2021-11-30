#include "../utility/pietslib.h"
#include "ros/ros.h"

using namespace std;
using namespace dlib;

#include <iostream>
#include <fstream>
#include <sstream>

dlib::image_window win;

//launch-prefix="gdb -ex run --args"
icog_face_tracker::facegeos faces;
//Callback vom USB CAM "image_raw" topic
void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
    //Einlesen des Bildes aus der Kamera Topic in einen Opencv Frame Pointer
    cv_bridge::CvImagePtr cvPtr;
    try
    {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //CV Pointer to CV_Matrix image
    win.clear_overlay();
    dlib::cv_image<bgr_pixel> imgout(cvPtr->image);
    win.set_image(imgout);

    for (int i = 0; i < faces.facegeos.size(); i++)
    {
        stringstream ss;
        ss << i;
        //Save Facebox to geometryBuffer
        dlib::rectangle dFace(
            faces.facegeos[i].face.x, faces.facegeos[i].face.y, faces.facegeos[i].face.x + faces.facegeos[i].face.width,
            faces.facegeos[i].face.y + faces.facegeos[i].face.height);
        win.add_overlay(dFace, rgb_pixel(0, 255, 0), "Face No.: " + ss.str());
        //cout << faces.facegeos.size() << endl;
        for (int u = 0; u < faces.facegeos[i].face_points.size(); u++)
        {
            dlib::rectangle rect(
                faces.facegeos[i].face_points[u].x, faces.facegeos[i].face_points[u].y, faces.facegeos[i].face_points[u].x + 2, faces.facegeos[i].face_points[u].y + 2);
            win.add_overlay(rect, rgb_pixel(0, 255, 0));
        }
    }
    icog_face_tracker::facegeos empty;
    faces = empty;
}

void trackerCB(const icog_face_tracker::facegeos &msg)
{
    faces = msg;
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/facegeos", 1, trackerCB);
    //debugpub = nh.advertise<string>("/debug", 5);
    //den Subcriber initiieren und dem Topic /usb_cam_node/image_raw subscriben
    ros::Subscriber sub2 = nh.subscribe("/usb_cam_node/image_raw", 1, imageCB);
    ros::spin();
}