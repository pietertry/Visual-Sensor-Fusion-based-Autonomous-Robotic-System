#include "../utility/pietslib.h"

using namespace std;
using namespace dlib;

//launch-prefix="gdb -ex run --args"

//Deklaration ROS Objekte
ros::Publisher pub;
ros::Publisher debugpub;
bool showPreview;

//Count the Frames till next new Face tracking
int newFaceCounter = 0;
const int COUNTSFORNEWSEARCH = 60;

//FaceTracker
pietslibrary::facetracker myFacetracker;

//Callback vom USB CAM "image_raw" topic
void imageCB(const sensor_msgs::ImageConstPtr &msg)
{
    //Einlesen des Bildes aus der Kamera Topic in einen Opencv Frame Pointer
    cv_bridge::CvImagePtr cvPtr;
    try
    {
        //  cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //CV Pointer to CV_Matrix image
    cv::Mat opencvimg = cvPtr->image;

    //Neue Gesichter finden Modus
    if (myFacetracker.getNumbFaces() < 1)
    {
        if (myFacetracker.findFaces(opencvimg))
            pub.publish(myFacetracker.getAllFaces());
    }
    //alle (60sek) Gesichter Tracken und neue Gesichter suchen
    else if (newFaceCounter > COUNTSFORNEWSEARCH)
    {
        if (myFacetracker.trackFaces(opencvimg) | myFacetracker.findFaces(opencvimg))
        {
        }
        pub.publish(myFacetracker.getAllFaces());
        newFaceCounter = 0;
    }
    //nur bekannte Gesichter tracken
    else
    {
        newFaceCounter++;
        if (myFacetracker.trackFaces(opencvimg))
        {
        }
        pub.publish(myFacetracker.getAllFaces());
    }
}

int main(int argc, char **argv)
{

    //Ros Handle initiieren
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    nh.param("/tracker_node/show_preview", showPreview, false);
    myFacetracker.init(showPreview);
    //den Publisher an das topic faces initiieren
    pub = nh.advertise<icog_face_tracker::facegeos>("/facegeos", 5);
    //debugpub = nh.advertise<string>("/debug", 5);
    //den Subcriber initiieren und dem Topic /usb_cam_node/image_raw subscriben
    ros::Subscriber sub = nh.subscribe("/d435/InfraredImage", 1, imageCB);
    ros::spin();
}