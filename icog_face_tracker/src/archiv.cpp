
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <icog_face_tracker/facegeometry.h>
#include <icog_face_tracker/facegeos.h>
#include <icog_face_tracker/myPoint.h>

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include "../utility/pietslib.h"

using namespace std;
using namespace dlib;

//Deklaration ROS Objekte
ros::Publisher pub;
bool showPreview;

//opencv Cascadenfilter
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier mouth_cascade;

//Deklaration Dlib Face Feature Detector
dlib::image_window win;
//frontal_face_detector detector = get_frontal_face_detector();
dlib::shape_predictor pose_model;

//Rectangle converter from CV to dlib
static dlib::rectangle openCVRectToDlib(cv::Rect r);
static cv::Rect dlibRectangleToOpenCV(dlib::rectangle r);

//fail counter
int fails;

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
    cv::Mat opencvimg = cvPtr->image;

    //Gesichtserkennung mittels Haar Cascade mit Opencv
    cv::vector<cv::Rect> cvfaces;
    cv::Mat bufferMat;
    cv::cvtColor(opencvimg, bufferMat, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(bufferMat, bufferMat);
    face_cascade.detectMultiScale(bufferMat, cvfaces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(bufferMat.rows / 4, bufferMat.rows / 4));

    //Opencv Picture Matrix conversion to dlib image type
    cv_image<bgr_pixel> cimg(opencvimg);

    // Convert CV Rect to Dlib rectangle
    std::vector<rectangle> faces;
    for (int i = 0; i < cvfaces.size(); i++)
        faces.push_back(openCVRectToDlib(cvfaces[i]));

    // Find the pose of each face.
    std::vector<rectangle> blops;
    std::vector<full_object_detection> shape;
    for (unsigned long i = 0; i < faces.size(); ++i)
    {
        shape.push_back(pose_model(cimg, faces[i]));
        //Mundwinkelpunkte speichern für die Anzeige für die spätere Anzeige
        blops.push_back(rectangle(shape[i].part(48).x(), shape[i].part(48).y(), shape[i].part(48).x() + 3, shape[i].part(48).y() + 3));
        blops.push_back(rectangle(shape[i].part(54).x(), shape[i].part(54).y(), shape[i].part(54).x() + 3, shape[i].part(54).y() + 3));
    }

    //publish on topic
    icog_face_tracker::facegeometry output;

    //fill facepoints
    if (!shape.empty())
    {
        for (int i = 0; i < shape[0].num_parts(); i++)
        {
            icog_face_tracker::myPoint pointbuff;
            pointbuff.x = shape[0].part(i).x();
            pointbuff.y = shape[0].part(i).y();
            output.face_points.push_back(pointbuff);
        }
        //relativer Abstand der Punkte 39 und 42 und der Punkte 0 und 16
        output.relDist_EE = sqrt(pow((shape[0].part(39).x() - shape[0].part(42).x()), 2) + pow((shape[0].part(39).y() + shape[0].part(42).y()), 2));
        output.failed_attempts = fails;
    }
    else
    {
        fails++;
        output.failed_attempts = fails;
    }

    // Display it all on the screen
    if (showPreview)
    {
        win.clear_overlay();
        win.set_image(cimg);
        win.add_overlay(render_face_detections(shape));
        win.add_overlay(faces, rgb_pixel(0, 255, 0));
        win.add_overlay(blops, rgb_pixel(0, 255, 0));
    }
    pub.publish(output);
}

int main(int argc, char **argv)
{
    //get Path of Home and Catkin_Ordner
    const char *home = getenv("HOME");
    string path;
    path.assign(home);
    path.append("/catkin_ws/src/icog_face_tracker/utility");
    string path2 = path;
    path.append("/shape_predictor_68_face_landmarks.dat");
    path2.append("/haarcascade_frontalface_alt.xml");

    //dlib landmarkdetector init
    dlib::deserialize(path) >> pose_model;

    //Laden der cascaden Filter
    try
    {
        face_cascade.load(path2);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    //Ros Handle initiieren
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    nh.param("/tracker_node/show_preview", showPreview, false);

    //den Publisher an das topic faces initiieren
    pub = nh.advertise<icog_face_tracker::facegeometry>("/facegeometry", 5);

    //den Subcriber initiieren und dem Topic /usb_cam_node/image_raw subscriben
    ros::Subscriber sub = nh.subscribe("/usb_cam_node/image_raw", 1, imageCB);
    ros::spin();

    fails = 0;
}

static dlib::rectangle openCVRectToDlib(cv::Rect r)
{
    return dlib::rectangle((long)r.tl().x, (long)r.tl().y, (long)r.br().x - 1, (long)r.br().y - 1);
}

static cv::Rect dlibRectangleToOpenCV(dlib::rectangle r)
{
    return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
}

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>

#include <std_msgs/String.h>
#include <icog_face_tracker/facegeometry.h>
#include <icog_face_tracker/facegeos.h>
#include <icog_face_tracker/myPoint.h>
#include <kinova_msgs/PoseVelocity.h>

using namespace std;

//Globale Variablen
bool laserOK;
icog_face_tracker::facegeos faces;

double laserDist;

//Callback vom USB CAM "image_raw" topic
void laserCB(const geometry_msgs::Vector3 &msg)
{
    try
    {
        if (msg.y == 0)
        {
            laserDist = msg.x;
            laserOK = true;
        }
        else
        {
            laserDist = msg.x;
            laserOK = false;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void trackerCB(const icog_face_tracker::facegeos &msg)
{
    try
    {
        faces = msg;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "jaco_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber facesSub = nh.subscribe("/facegeos", 1, trackerCB);
    //ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);

    //publisher initialisieren
    ros::Publisher jacoCon = nh.advertise<kinova_msgs::PoseVelocity>("j2s7s300_driver/in/cartesian_velocity", 5);
    ///j2s7s300_driver/in/cartesian_vecity kinova_msgs/PoseVelocity

    ros::Rate r(100); // 100 hz
    while (ros::ok())
    {
        float rotY, rotX;
        int b1 = 640 / 2 - 25; //   4
        int b2 = 480 / 2 - 25; //1     3
        int b3 = 640 / 2 + 25; //   2
        int b4 = 480 / 2 + 25;
        if (faces.facegeos[0].face_points[51].x < b1)
        {
            rotY = 0.1;
        }
        else if (faces.facegeos[0].face_points[51].x > b3)
        {
            rotY = -0.1;
        } /*
        else if (faces.facegeos[0].face_points[51].x < b1)
        {
            rotX = 0.02;
        }
        else if (faces.facegeos[0].face_points[51].x < b1)
        {
            rotX = 0.02;
        }*/

        kinova_msgs::PoseVelocity output;
        output.twist_angular_y = rotY;
        jacoCon.publish(output);

        ros::spinOnce();
        r.sleep();
    }

    //ros::spin();
}

//
///

//
////
//
///

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>

#include <std_msgs/String.h>
#include <icog_face_tracker/facegeometry.h>
#include <icog_face_tracker/facegeos.h>
#include <icog_face_tracker/myPoint.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "../utility/pid.h"

using namespace std;

//Globale Variablen
//Variable die den Status des Lasers speichert
icog_face_tracker::facegeos faces;       //Data from face Tracking
kinova_msgs::PoseVelocity output;        //Variable for moving the Kinova Robot
geometry_msgs::Vector3Stamped laserData; //Data from Laser Sensor (Distance, Status, ROI)
geometry_msgs::PoseStamped toolPose;     //Tool Pose
sensor_msgs::Joy joyData;

bool newTrackingData = false; //Variable zum auslösen des Roboter Reglers
bool newPoseData = false;
bool newJoyData = false;
bool newLaserData = false;
bool newLaserReconData = false;

geometry_msgs::Vector3 HOMEPOSITION;

geometry_msgs::Vector3 target_position;
geometry_msgs::Vector3 target_orientation;
geometry_msgs::Vector3 toolPose_position;
geometry_msgs::Vector3Stamped laserField;

int mode = 0; //Steuerungsmodus

//Funktionsdeklaration
float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2);

//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

//Callback vom USB CAM "image_raw" topic
void laserCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserData = msg;
    newLaserData = true;
}

void trackerCB(const icog_face_tracker::facegeos &msg)
{

    faces = msg;
    newTrackingData = true;
}

void toolPoseCB(const geometry_msgs::PoseStamped &msg)
{
    toolPose = msg;
    newPoseData = true;
}

void joyCB(const sensor_msgs::Joy &msg)
{
    joyData = msg;
    newJoyData = true;
}

void laserReconCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserField = msg;
    newLaserReconData = true;
}
int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "jaco_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber facesSub = nh.subscribe("/facegeos", 1, trackerCB);
    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);
    ros::Subscriber toolPoseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCB);
    ros::Subscriber joySub = nh.subscribe("/joy", 1, joyCB);
    ros::Subscriber laserReconSub = nh.subscribe("/mouthTracking/cameraCoordinates", 1, laserReconCB);

    //Home Position
    HOMEPOSITION.x = -0.145167;
    HOMEPOSITION.y = -0.366662;
    HOMEPOSITION.z = 0.0252345;

    //PID Controller for Robot controlls
    PID rotHorizontalPID = PID(0.01, 10, -10, 0.016, 0.0, 0.0004);
    PID linearVertPID = PID(0.01, 10, -10, 0.0035, 0.0, 0.000001);

    PID faceAlignmentPID = PID(0.01, 0.1, -0.1, 0.1, 0.0, 0.0001);
    PID moveTowardsUserPID = PID(0.01, 0.05, -0.05, 1.3, 0.0, 0.0005);

    PID angularOrientZPID = PID(0.01, 10.0, -10.0, 2.0, 0.0, 0.001); //Controller for Tool ROll
    PID angularOrientXPID = PID(0.01, 10.0, -10.0, 2.0, 0.0, 0.001); //Controller for Tool PITCH
    PID angularOrientYPID = PID(0.01, 10.0, -10.0, 2.0, 0.0, 0.001); //Controller for Tool YAW
    PID positionXPID = PID(0.01, 0.2, -0.2, 2.5, 0.0, 0.008);        //Controller for Tool ROll
    PID positionYPID = PID(0.01, 0.2, -0.2, 2.5, 0.0, 0.008);        //Controller for Tool PITCH
    PID positionZPID = PID(0.01, 0.2, -0.2, 2.5, 0.0, 0.008);        //Controller for Tool YAW

    //publisher initialisieren
    ros::Publisher jacoCon = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 5);
    ros::Publisher laser_setROI = nh.advertise<geometry_msgs::Vector3>("/VL53L1X/setROI", 5);
    //Robotername und steuerungstopic j2s7s300_driver/in/cartesian_vecity kinova_msgs/PoseVelocity

    //set initial Toolpose
    target_position.x = -0.145167;
    target_position.y = -0.366662;
    target_position.z = 0.0252345;
    target_orientation.y = 0;
    target_orientation.x = (double)M_PI / 2.0;
    target_orientation.z = 0;

    //laserROI
    std::vector<int> laserROI = {199, 04, 04};

    //state machine variables
    int state = 0;
    double cupDistance = 0;
    double mouthTargetDistance = 0;
    double positionError = 10;
    bool usePosController = false;
    bool useRotController = false;
    double cupDistSum = 0;
    int numCupRangings = 0;

    //Kinova publisher
    ros::Rate r(100); // 100 hz
    while (ros::ok())
    {
        //Reset Movement to Stopp
        output.twist_linear_x = 0;
        output.twist_linear_y = 0;
        output.twist_linear_z = 0;
        output.twist_angular_x = 0;
        output.twist_angular_y = 0;
        output.twist_angular_z = 0;

        usePosController = false;
        useRotController = false;
        //Get the Orientation of the robot tool and convert from quaternion to euler angles for easier control
        geometry_msgs::Vector3 orientation;
        geometry_msgs::Vector3 position;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(toolPose.pose.orientation, quat);
        quat.normalize();
        tf::Matrix3x3(quat).getRPY(orientation.x, orientation.y, orientation.z);
        position.x = toolPose.pose.position.x;
        position.y = toolPose.pose.position.y;
        position.z = toolPose.pose.position.z;

        //Stopp with A Button on joy
        if (newJoyData && joyData.buttons[2] == 1)
        {
            state = 0;
            newJoyData = false;
        }

        switch (state)
        {
            //CASE 0___________________________________________________________________________________
        case 0:
            //Move to home position and stay on standby
            cupDistance = 0;
            mouthTargetDistance = 0;
            target_position = HOMEPOSITION;
            target_orientation.z = -1.0 * (double)M_PI / 2.0;
            usePosController = true;
            useRotController = true;
            //transition condition
            if (newJoyData && joyData.buttons[0] == 1 && positionError < 0.01)
            {
                state = 1;
                ROS_INFO_STREAM("State: " << state << "  check for suitable faces");
            }
            break;
            //CASE 1___________________________________________________________________________________

        case 1:
            if (!faces.facegeos.empty())
                if (faces.facegeos[0].face.width > 50)
                {
                    state = 2;
                    ROS_INFO_STREAM("State: " << state);
                }
                else
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  User too far away");
                }
            break;
            //CASE 2___________________________________________________________________________________
        case 2:
        {
            laserROI = {193, 4, 4};
            if (newLaserData == true && laserData.vector.z == (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
            {
                if (laserData.vector.y == 0)
                {
                    //get the average of 10 values
                    numCupRangings++;
                    cupDistSum += laserData.vector.x / 1000;
                    //ROS_INFO_STREAM(numCupRangings << ": " << cupDistSum << cupDistSum / numCupRangings);
                    if (numCupRangings >= 10)
                    {
                        cupDistance = cupDistSum / numCupRangings; //distance in meter
                        if (0.22 > cupDistance > 0.010)
                        {
                            mouthTargetDistance = cupDistance + 0.005; //distance + cup thickness
                            numCupRangings = 0;
                            cupDistSum = 0;
                            state = 21;
                            ROS_INFO_STREAM("State: " << state << "   Cup distance: " << cupDistance);
                        }
                        else
                        {
                            cupDistance = 0;
                            numCupRangings = 0;
                            cupDistSum = 0;
                            state = 0;
                            ROS_INFO_STREAM("State: " << state << "   Ranging Error. Please try again.");
                        }
                    }
                }
                else
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "   No Cup Detected");
                }
            }
        }
        break;
        case 21:
        {
            laserROI = {199, 4, 4};
            if (laserData.vector.z == (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
            {
                state = 3;
                ROS_INFO_STREAM("State: " << state << "   Laser ROI change back to 1990404");
            }
        }
        break;

            //CASE 3___________________________________________________________________________________
        case 3:
        {
            //if (newTrackingData && newLaserReconData)
            if (!faces.facegeos.empty())
            {
                //inital estimate for laser meaurement position
                int targetX = 260;
                int targetY = 150;
                if (laserField.vector.x + laserField.vector.y != 0)
                {
                    //Position of laser measurement area
                    targetX = laserField.vector.x;
                    targetY = laserField.vector.y;
                }
                //Drehe den Roboter zum Nutzer
                if (!faces.facegeos[0].facingAway && (faces.facegeos[0].failed_attempts < 2))
                {
                    //Regler für Rotation in der Horizontalen
                    output.twist_angular_y = rotHorizontalPID.calculate(targetX, faces.facegeos[0].face_points[51].x);
                    //Regler zur linearen Bewegung in der Vertikalen
                    output.twist_linear_z += linearVertPID.calculate(targetY, faces.facegeos[0].face_points[51].y);
                    //Regelung um den Becher gerade zum Gesicht zu orientieren
                    double controllerValue = faceAlignmentPID.calculate(0, -faces.facegeos[0].yaw);
                    output.twist_linear_x += controllerValue * cos(-orientation.z);
                    output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);

                    if (0.005 > sqrt(pow(output.twist_linear_x, 2) + pow(output.twist_linear_y, 2) + pow(output.twist_linear_z, 2)))
                    {
                        ros::Duration(1, 0).sleep();
                        state = 4;
                        ROS_INFO_STREAM("State: " << state << "   Aligned Robot with face");
                    }
                }
            }
        }
        break;
            //CASE 4___________________________________________________________________________________
        case 4:
        {
            int targetX = 260;
            int targetY = 150;
            if (laserField.vector.x != 0.0)
            {
                targetX = laserField.vector.x;
                targetY = laserField.vector.y;
            }
            //move towards user
            double forward = moveTowardsUserPID.calculate(mouthTargetDistance, laserData.vector.x / 1000.0);
            output.twist_linear_x += forward * sin(-orientation.z);
            output.twist_linear_y += forward * cos(-orientation.z);
            //Regler für Rotation in der Horizontalen
            output.twist_angular_y = rotHorizontalPID.calculate(targetX, faces.facegeos[0].face_points[51].x);
            //ROS_INFO_STREAM(targetX << ";  " << faces.facegeos[0].face_points[51].x);
            //Regler zur linearen Bewegung in der Vertikalen
            output.twist_linear_z += 0.3 * linearVertPID.calculate(targetY, faces.facegeos[0].face_points[51].y);
            //Regelung um den Becher gerade zum Gesicht zu orientieren
            /*
            double controllerValue = faceAlignmentPID.calculate(0, -faces.facegeos[0].yaw);
            output.twist_linear_x += controllerValue * cos(-orientation.z);
            output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);*/

            //Stopp if vision says so
            if ((laserData.vector.x / 1000.0 - mouthTargetDistance) > 0.015)
            {
                if (((int)faces.facegeos[0].failed_attempts > 20) || faces.facegeos[0].facingAway)
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  User stopped the approach" << (int)faces.facegeos[0].failed_attempts << "; " << (int)faces.facegeos[0].facingAway);
                }
            }
            //transition condition
            if ((laserData.vector.x / 1000.0 - mouthTargetDistance) < 0.002)
            {
                state = 5;

                target_position = position;
                target_position.z += 0.018;
                ROS_INFO_STREAM("State: " << state << "  reached user");
            }
        }
        break;
            //CASE 5___________________________________________________________________________________
        case 5:
        {
            usePosController = true;
            //Stopp with A Button on joy
            if (newJoyData && joyData.buttons[2] == 1)
            {
                state = 0;
                newJoyData = false;
            }
        }
        break;
        }

        // Position and Orientation Controller
        if (newPoseData)
        {
            if (usePosController)
            {
                //velocity controller for position
                output.twist_linear_x = positionXPID.calculate(target_position.x, position.x);
                output.twist_linear_y = positionYPID.calculate(target_position.y, position.y);
                output.twist_linear_z = positionZPID.calculate(target_position.z, position.z);
            }
            if (useRotController)
            {
                //controller for roll and pitch of the tool
                output.twist_angular_z = -1 * angularOrientZPID.calculate(target_orientation.y, orientation.y);
                output.twist_angular_x = angularOrientXPID.calculate(target_orientation.x, orientation.x);
                output.twist_angular_y = angularOrientYPID.calculate(target_orientation.z, orientation.z);
                //}
                positionError = sqrt(pow(positionXPID.getError(), 2) +
                                     pow(positionYPID.getError(), 2) +
                                     pow(positionZPID.getError(), 2));
            }
        }

        jacoCon.publish(output);

        //set ROI of laserSensor
        if (laserData.vector.z != (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
        {
            geometry_msgs::Vector3 vec;
            vec.x = laserROI.at(2);
            vec.y = laserROI.at(1);
            vec.z = laserROI.at(0);
            laser_setROI.publish(vec);
        }

        //reset CB confirmations
        newTrackingData = false; //Variable zum auslösen des Roboter Reglers
        //newPoseData = false;
        newJoyData = false;
        newLaserData = false;
        newLaserReconData = false;
        ros::spinOnce();
        r.sleep();
    }
}

float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2)
{
    return double(sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2)));
}

//Joystick Steuerung
/*
 //Joystick steuerung zum Randfahren an das Gesicht
            if (newJoyData)
            {
                //RS
                output.twist_angular_y += joyData.axes[3] * 0.5;
                output.twist_angular_x += joyData.axes[4] * 0.5;
                //LS
                output.twist_linear_x += -1 * joyData.axes[7] * 0.10 * sin(-orientation.z);
                output.twist_linear_y += -1 * joyData.axes[7] * 0.10 * cos(-orientation.z);
                output.twist_linear_x += joyData.axes[6] * 0.15 * sin(-orientation.z + M_PI / 2.0);
                output.twist_linear_y += joyData.axes[6] * 0.15 * cos(-orientation.z + M_PI / 2.0);
                //L1
                output.twist_linear_z += (1 - joyData.axes[5]) * 0.8;
                //R1
                output.twist_linear_z -= (1 - joyData.axes[2]) * 0.8;
            }


*/

/*
 //Roboter Regler mittels Face Tracking daten (~3hz)
        if (!faces.facegeos.empty())
        {
            //target Rectangle boundries
            int boundrySize = getDistance(faces.facegeos[0].face_points[49], faces.facegeos[0].face_points[53]) / 2;
            int targetX = 480 / 2;
            int targetY = 270 / 2 + 10;

            //Drehe den Roboter zum Nutzer
            if (!faces.facegeos[0].facingAway && (faces.facegeos[0].failed_attempts < 1))
            {
                //cout << faces.facegeos[0].facingAway && (faces.facegeos[0].failed_attempts << endl;
                //Regler für Rotation in der Horizontalen
                output.twist_angular_y = rotHorizontalPID.calculate(targetX, faces.facegeos[0].face_points[51].x);
                //Regler zur linearen Bewegung in der Vertikalen
                output.twist_linear_z = linearVertPID.calculate(targetY, faces.facegeos[0].face_points[51].y);
                //Regelung um den Becher gerade zum Gesicht zu orientieren
                double controllerValue = faceAlignmentPID.calculate(0, -faces.facegeos[0].yaw);
                output.twist_linear_x += controllerValue * cos(-orientation.z);
                output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);
            }
        }
*/

/*
<node pkg="rosserial_python" type="serial_node.py" name="laser_rosserial">
		<param name="port" value="/dev/ttyACM0" />
		<param name="baud" value="115200" />
	</node>

*/

//Trinkvorgang 09.11
//
//
////
//
///
///
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <icog_face_tracker/facegeometry.h>
#include <icog_face_tracker/facegeos.h>
#include <icog_face_tracker/myPoint.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "../utility/pid.h"

using namespace std;

//Globale Variablen
//Variable die den Status des Lasers speichert
icog_face_tracker::facegeos faces;       //Data from face Tracking
kinova_msgs::PoseVelocity output;        //Variable for moving the Kinova Robot
geometry_msgs::Vector3Stamped laserData; //Data from Laser Sensor (Distance, Status, ROI)
geometry_msgs::PoseStamped toolPose;     //Tool Pose
sensor_msgs::Joy joyData;
int controllerStatus;

bool newTrackingData = false; //Variable zum auslösen des Roboter Reglers
bool newPoseData = false;
bool newJoyData = false;
bool newLaserData = false;
bool newLaserReconData = false;

geometry_msgs::Vector3 HOMEPOSITION;

geometry_msgs::Vector3 target_position;
geometry_msgs::Vector3 target_orientation;
geometry_msgs::Vector3 toolPose_position;
geometry_msgs::Vector3Stamped laserField;

int mode = 0; //Steuerungsmodus

//Funktionsdeklaration
float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2);

//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

//Callback vom USB CAM "image_raw" topic
void laserCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserData = msg;
    newLaserData = true;
}

void trackerCB(const icog_face_tracker::facegeos &msg)
{

    faces = msg;
    newTrackingData = true;
}

void toolPoseCB(const geometry_msgs::PoseStamped &msg)
{
    toolPose = msg;
    newPoseData = true;
}

void joyCB(const sensor_msgs::Joy &msg)
{
    joyData = msg;
    newJoyData = true;
}

void laserReconCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserField = msg;
    newLaserReconData = true;
}

void controllerStatusCB(const std_msgs::Int8 &msg)
{
    controllerStatus = msg.data;
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "jaco_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber facesSub = nh.subscribe("/facegeos", 1, trackerCB);
    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);
    ros::Subscriber toolPoseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCB);
    ros::Subscriber joySub = nh.subscribe("/joy", 1, joyCB);
    ros::Subscriber laserReconSub = nh.subscribe("/mouthTracking/cameraCoordinates", 1, laserReconCB);
    ros::Subscriber roboControllerStatusSub = nh.subscribe("/mouthTrackig/roboComStatus", 1, controllerStatusCB);

    //Home Position
    HOMEPOSITION.x = 0.20701;
    HOMEPOSITION.y = -0.39760;
    HOMEPOSITION.z = 0.00223;

    //PID Controller for Robot controlls
    PID rotHorizontalPID = PID(0.01, 10, -10, 0.016, 0.0, 0.0004);
    PID linearVertPID = PID(0.01, 10, -10, 0.0035, 0.0, 0.000001);

    PID faceAlignmentPID = PID(0.01, 0.1, -0.1, 0.1, 0.0, 0.0001);
    PID moveTowardsUserPID = PID(0.01, 0.05, -0.05, 1.3, 0.0, 0.0005);

    PID angularOrientZPID = PID(0.01, 10.0, -10.0, 2.0, 0.0, 0.001); //Controller for Tool ROll
    PID angularOrientXPID = PID(0.01, 10.0, -10.0, 2.0, 0.0, 0.001); //Controller for Tool PITCH
    PID angularOrientYPID = PID(0.01, 10.0, -10.0, 2.0, 0.0, 0.001); //Controller for Tool YAW
    PID positionXPID = PID(0.01, 0.2, -0.2, 2.5, 0.0, 0.008);        //Controller for Tool ROll
    PID positionYPID = PID(0.01, 0.2, -0.2, 2.5, 0.0, 0.008);        //Controller for Tool PITCH
    PID positionZPID = PID(0.01, 0.2, -0.2, 2.5, 0.0, 0.008);        //Controller for Tool YAW

    //publisher initialisieren
    ros::Publisher jacoCon = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 5);
    ros::Publisher laser_setROI = nh.advertise<geometry_msgs::Vector3>("/VL53L1X/setROI", 5);
    ros::Publisher imageServoTargetPub = nh.advertise<geometry_msgs::Vector3>("/mouthTracking/imageServoingTarget", 5);
    ros::Publisher controllerModePub = nh.advertise<geometry_msgs::Vector3>("/mouthTracking/mode", 5);
    ros::Publisher toolPoseTargetPub = nh.advertise<geometry_msgs::Pose>("/mouthTracking/toolPoseTarget", 5);

    //Robotername und steuerungstopic j2s7s300_driver/in/cartesian_vecity kinova_msgs/PoseVelocity

    //set initial Toolpose
    target_position.x = 0.20701;
    target_position.y = -0.39760;
    target_position.z = 0.00223;
    target_orientation.y = 2;
    target_orientation.x = 1;
    target_orientation.z = 3;

    //laserROI
    std::vector<int> laserROI = {199, 04, 04};

    //state machine variables
    int state = 0;
    double cupDistance = 0;
    double mouthTargetDistance = 0;
    double positionError = 10;
    double cupDistSum = 0;
    int numCupRangings = 0;

    //timer to save last time
    uint64_t timer = 0;

    //Kinova publisher
    ros::Rate r(100); // 100 hz
    while (ros::ok())
    {

        //Stopp with A Button on joy
        if (newJoyData && joyData.buttons[2] == 1)
        {
            state = 0;
            newJoyData = false;
        }

        switch (state)
        {
            //CASE 0___________________________________________________________________________________
        case 0:
        {
            //Move to home position and stay on standby
            cupDistance = 0;
            mouthTargetDistance = 0;
            target_position = HOMEPOSITION;
            target_orientation.z = 3;

            //set robot controller mode
            geometry_msgs::Vector3 conMode;
            conMode.x = 0;
            conMode.y = 1;
            conMode.z = 0;
            //set target position
            geometry_msgs::Pose targetPose;
            targetPose.position.x = target_position.x;
            targetPose.position.y = target_position.y;
            targetPose.position.z = target_position.z;
            tf::Quaternion quat;
            quat.setEuler(target_orientation.z, target_orientation.x, target_orientation.y);
            quat.normalize();
            tf::quaternionTFToMsg(quat, targetPose.orientation);

            toolPoseTargetPub.publish(targetPose);
            controllerModePub.publish(conMode);
            //transition condition
            //ROS_INFO_STREAM(positionXPID.getError() << "  " << positionYPID.getError() << "  " << positionZPID.getError());
            if (newJoyData && joyData.buttons[0] == 1 && controllerStatus == 0)
            {
                geometry_msgs::Vector3 conMode;
                conMode.x = 0;
                conMode.y = 0;
                conMode.z = 0;
                controllerModePub.publish(conMode);
                state = 1;
                ROS_INFO_STREAM("State: " << state << "  check for suitable faces");
            }
            break;
            //CASE 1___________________________________________________________________________________
        }
        case 1:
        {
            if (!faces.facegeos.empty())
                if (faces.facegeos[0].face.width > 50)
                {
                    state = 2;
                    ROS_INFO_STREAM("State: " << state);
                }
                else
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  User too far away");
                }
            break;
        }
            //CASE 2___________________________________________________________________________________
        case 2:
        {
            laserROI = {193, 4, 4};
            if (newLaserData == true && laserData.vector.z == (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
            {
                if (laserData.vector.y == 0)
                {
                    //get the average of 10 values
                    numCupRangings++;
                    cupDistSum += laserData.vector.x / 1000;
                    //ROS_INFO_STREAM(numCupRangings << ": " << cupDistSum << cupDistSum / numCupRangings);
                    if (numCupRangings >= 10)
                    {
                        cupDistance = cupDistSum / numCupRangings; //distance in meter
                        if (0.22 > cupDistance > 0.010)
                        {
                            mouthTargetDistance = cupDistance + 0.010; //distance + cup thickness
                            numCupRangings = 0;
                            cupDistSum = 0;
                            state = 21;
                            ROS_INFO_STREAM("State: " << state << "   Cup distance: " << cupDistance);
                        }
                        else
                        {
                            cupDistance = 0;
                            numCupRangings = 0;
                            cupDistSum = 0;
                            state = 0;
                            ROS_INFO_STREAM("State: " << state << "   Ranging Error. Please try again.");
                        }
                    }
                }
                else
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "   No Cup Detected");
                }
            }
        }
        break;
        case 21:
        {
            laserROI = {199, 4, 4};
            if (laserData.vector.z == (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
            {
                state = 3;
                ROS_INFO_STREAM("State: " << state << "   Laser ROI change back to 1990404");
            }
        }
        break;

            //CASE 3___________________________________________________________________________________
        case 3:
        {
            //if (newTrackingData && newLaserReconData)
            if (!faces.facegeos.empty())
            {
                geometry_msgs::Vector3 conMode;
                conMode.x = 10.0;
                conMode.y = 0;
                conMode.z = 0;
                //inital estimate for laser meaurement position
                geometry_msgs::Vector3 imageTarget;
                imageTarget.x = 260;
                imageTarget.y = 150;
                if (laserField.vector.x + laserField.vector.y != 0)
                {
                    //Position of laser measurement area
                    imageTarget.x = laserField.vector.x;
                    imageTarget.y = laserField.vector.y;
                }
                imageServoTargetPub.publish(imageTarget);
                controllerModePub.publish(conMode);
                //Transition condition
                if (controllerStatus == 0 && timer == 0)
                {
                    timer = ros::Time::now().toNSec();
                }
                else if (controllerStatus == 0)
                {
                    if (2000000000 < (ros::Time::now().toNSec() - timer))
                    {
                        timer = 0;
                        state = 4;
                        ROS_INFO_STREAM("State: " << state << "   Aligned Robot with face");
                        geometry_msgs::Vector3 conMode;
                        conMode.x = 0;
                        conMode.y = 0;
                        conMode.z = 1;
                        controllerModePub.publish(conMode);
                    }
                }
                else
                {
                    timer = 0;
                }
            }
        }
        break;
            //CASE 4___________________________________________________________________________________
        case 4:
        {
            geometry_msgs::Vector3 imageTarget;
            imageTarget.x = 260;
            imageTarget.y = 150;
            if (laserField.vector.x != 0.0)
            {
                imageTarget.x = laserField.vector.x;
                imageTarget.y = laserField.vector.y;
            }
            imageTarget.z = mouthTargetDistance;

            //Stopp if vision says so
            if ((laserData.vector.x / 1000.0 - mouthTargetDistance) > 0.010)
            {
                if (((int)faces.facegeos[0].failed_attempts > 20) || faces.facegeos[0].facingAway)
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  User stopped the approach" << (int)faces.facegeos[0].failed_attempts << "; " << (int)faces.facegeos[0].facingAway);
                }
                if ((laserData.vector.x / 1000.0 - mouthTargetDistance) < -0.005)
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  user too close");
                }
            }
            //transition condition
            if ((laserData.vector.x / 1000.0 - mouthTargetDistance) < 0.002)
            {
                state = 5;

                ROS_INFO_STREAM("State: " << state << "  reached user");
            }
        }
        break;
            //CASE 5___________________________________________________________________________________
        case 5:
        {

            //Stopp with A Button on joy
            if (newJoyData && joyData.buttons[2] == 1)
            {
                state = 0;
                newJoyData = false;
            }
        }
        break;
        }

        // jacoCon.publish(output);

        //set ROI of laserSensor
        if (laserData.vector.z != (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
        {
            geometry_msgs::Vector3 vec;
            vec.x = laserROI.at(2);
            vec.y = laserROI.at(1);
            vec.z = laserROI.at(0);
            laser_setROI.publish(vec);
        }

        //reset CB confirmations
        newTrackingData = false; //Variable zum auslösen des Roboter Reglers
        //newPoseData = false;
        newJoyData = false;
        newLaserData = false;
        newLaserReconData = false;
        ros::spinOnce();
        r.sleep();
    }
}

float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2)
{
    return double(sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2)));
}

//Joystick Steuerung
/*
 //Joystick steuerung zum Randfahren an das Gesicht
            if (newJoyData)
            {
                //RS
                output.twist_angular_y += joyData.axes[3] * 0.5;
                output.twist_angular_x += joyData.axes[4] * 0.5;
                //LS
                output.twist_linear_x += -1 * joyData.axes[7] * 0.10 * sin(-orientation.z);
                output.twist_linear_y += -1 * joyData.axes[7] * 0.10 * cos(-orientation.z);
                output.twist_linear_x += joyData.axes[6] * 0.15 * sin(-orientation.z + M_PI / 2.0);
                output.twist_linear_y += joyData.axes[6] * 0.15 * cos(-orientation.z + M_PI / 2.0);
                //L1
                output.twist_linear_z += (1 - joyData.axes[5]) * 0.8;
                //R1
                output.twist_linear_z -= (1 - joyData.axes[2]) * 0.8;
            }


*/

/*
 //Roboter Regler mittels Face Tracking daten (~3hz)
        if (!faces.facegeos.empty())
        {
            //target Rectangle boundries
            int boundrySize = getDistance(faces.facegeos[0].face_points[49], faces.facegeos[0].face_points[53]) / 2;
            int targetX = 480 / 2;
            int targetY = 270 / 2 + 10;

            //Drehe den Roboter zum Nutzer
            if (!faces.facegeos[0].facingAway && (faces.facegeos[0].failed_attempts < 1))
            {
                //cout << faces.facegeos[0].facingAway && (faces.facegeos[0].failed_attempts << endl;
                //Regler für Rotation in der Horizontalen
                output.twist_angular_y = rotHorizontalPID.calculate(targetX, faces.facegeos[0].face_points[51].x);
                //Regler zur linearen Bewegung in der Vertikalen
                output.twist_linear_z = linearVertPID.calculate(targetY, faces.facegeos[0].face_points[51].y);
                //Regelung um den Becher gerade zum Gesicht zu orientieren
                double controllerValue = faceAlignmentPID.calculate(0, -faces.facegeos[0].yaw);
                output.twist_linear_x += controllerValue * cos(-orientation.z);
                output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);
            }
        }
*/

/*
<node pkg="rosserial_python" type="serial_node.py" name="laser_rosserial">
		<param name="port" value="/dev/ttyACM0" />
		<param name="baud" value="115200" />
	</node>




  // Position and Orientation Controller
        if (newPoseData)
        {
            if (usePosController)
            {
                //velocity controller for position
                output.twist_linear_x = positionXPID.calculate(target_position.x, position.x);
                output.twist_linear_y = positionYPID.calculate(target_position.y, position.y);
                output.twist_linear_z = positionZPID.calculate(target_position.z, position.z);
            }
            if (useRotController)
            {
                //controller for roll and pitch of the tool
                output.twist_angular_z = -1 * angularOrientZPID.calculate(target_orientation.y, orientation.y);
                output.twist_angular_x = angularOrientXPID.calculate(target_orientation.x, orientation.x);
                output.twist_angular_y = angularOrientYPID.calculate(target_orientation.z, orientation.z);
                //}
                positionError = sqrt(pow(positionXPID.getError(), 2) +
                                     pow(positionYPID.getError(), 2) +
                                     pow(positionZPID.getError(), 2));
            }
        }
*/

// RObot controller 9.11
//
//
//
//
//
//
//

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>

#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <icog_face_tracker/facegeometry.h>
#include <icog_face_tracker/facegeos.h>
#include <icog_face_tracker/myPoint.h>
#include <kinova_msgs/PoseVelocity.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "../utility/pid.h"

using namespace std;

//Globale Variablen
//Variable die den Status des Lasers speichert
geometry_msgs::Vector3 imageTarget;
geometry_msgs::Pose poseTarget;
geometry_msgs::PoseStamped toolPose;
geometry_msgs::Vector3 mode;
icog_face_tracker::facegeos faces;
geometry_msgs::Vector3Stamped laserData; //Data from Laser Sensor (Distance, Status, ROI)

bool newTrackingData;
bool newToolPoseData;
bool newImageTargetCommand;
bool newKinovaPoseVel;
bool newLaserData = false;

int timeOutImageServo, timeOutToolPose, timeOutAll;

//Funktionsdeklaration
float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2);

//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

//Callback vom USB CAM "image_raw" topic
void imageServoTargetCB(const geometry_msgs::Vector3 &msg)
{
    imageTarget = msg;
    newImageTargetCommand = true;
}

void poseTargetCB(const geometry_msgs::Pose &msg)
{
    poseTarget = msg;
    newKinovaPoseVel = true;
}

void toolPoseCB(const geometry_msgs::PoseStamped &msg)
{
    toolPose = msg;
    newToolPoseData = true;
}

void modeCB(const geometry_msgs::Vector3 &msg)
{
    mode = msg;
}

void trackerCB(const icog_face_tracker::facegeos &msg)
{

    faces = msg;
    newTrackingData = true;
}

void laserCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserData = msg;
    newLaserData = true;
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "jaco_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber stateModeSub = nh.subscribe("/mouthTracking/mode", 1, modeCB);
    ros::Subscriber imageServoTargetSub = nh.subscribe("/mouthTracking/imageServoingTarget", 1, imageServoTargetCB);
    ros::Subscriber toolPoseTargetSub = nh.subscribe("/mouthTracking/toolPoseTarget", 1, poseTargetCB);

    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);
    ros::Subscriber facesSub = nh.subscribe("/facegeos", 1, trackerCB);
    ros::Subscriber toolPoseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCB);

    //PID Controller for Robot controlls
    //PID Controller for Robot controlls
    PID rotHorizontalPID = PID(0.01, 0.3, -0.3, 0.007, 0.0, 0.00001);
    PID linearVertPID = PID(0.01, 0.1, -0.1, 0.005, 0.0, 0.000001);

    PID faceAlignmentPID = PID(0.01, 0.1, -0.1, 0.3, 0.0, 0.0001);
    PID moveTowardsUserPID = PID(0.01, 0.05, -0.05, 1, 0.0, 0.0005);

    PID angularOrientZPID = PID(0.01, 0.3, -0.3, 2.6, 0.0, 0.001); //Controller for Tool ROll
    PID angularOrientXPID = PID(0.01, 0.3, -0.3, 2.6, 0.0, 0.001); //Controller for Tool PITCH
    PID angularOrientYPID = PID(0.01, 0.3, -0.3, 2.0, 0.0, 0.001); //Controller for Tool YAW
    PID positionXPID = PID(0.01, 0.1, -0.1, 2.5, 0.0, 0.002);      //Controller for Tool ROll
    PID positionYPID = PID(0.01, 0.1, -0.1, 2.5, 0.0, 0.002);      //Controller for Tool PITCH
    PID positionZPID = PID(0.01, 0.1, -0.1, 2.5, 0.0, 0.002);      //Controller for Tool YAW

    //publisher initialisieren
    ros::Publisher jacoCon = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 5);
    ros::Publisher confirmPub = nh.advertise<std_msgs::Int8>("/mouthTrackig/roboComStatus", 5);

    //Kinova publisher
    ros::Rate r(100); // 100 hz
    while (ros::ok())
    {
        //Reset Movement to Stopp
        kinova_msgs::PoseVelocity output;
        output.twist_linear_x = 0;
        output.twist_linear_y = 0;
        output.twist_linear_z = 0;
        output.twist_angular_x = 0;
        output.twist_angular_y = 0;
        output.twist_angular_z = 0;

        if (newToolPoseData)
        {
            //Get the Orientation (in RPY) and position of the robot tool
            geometry_msgs::Vector3 orientation;
            geometry_msgs::Vector3 position;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(toolPose.pose.orientation, quat);
            quat.normalize();
            tf::Matrix3x3(quat).getRPY(orientation.x, orientation.y, orientation.z);
            position.x = toolPose.pose.position.x;
            position.y = toolPose.pose.position.y;
            position.z = toolPose.pose.position.z;

            if (10.0 <= mode.x && mode.x < 20.0 && newTrackingData && newImageTargetCommand && !faces.facegeos.empty())
            {
                switch ((int)mode.x)
                {
                case 10: //Roboter Ausrichten
                {
                    //Regler für Rotation in der Horizontalen
                    output.twist_angular_y = rotHorizontalPID.calculate(imageTarget.x, faces.facegeos[0].face_points[51].x);
                    //Regler zur linearen Bewegung in der Vertikalen
                    output.twist_linear_z = linearVertPID.calculate(imageTarget.y, faces.facegeos[0].face_points[51].y);
                    //Regelung um den Becher gerade zum Gesicht zu orientieren
                    double controllerValue = faceAlignmentPID.calculate(0, -faces.facegeos[0].yaw);
                    output.twist_linear_x += controllerValue * cos(-orientation.z);
                    output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);

                    double positionError = sqrt(pow(output.twist_linear_x, 2) +
                                                pow(output.twist_linear_y, 2) +
                                                pow(output.twist_linear_z, 2));
                    if (positionError < 0.01)
                    {
                        confirmPub.publish(0);
                    }
                    else
                    {
                        confirmPub.publish(1);
                    }
                }
                break;
                case 11:
                {
                    //move towards user
                    double forward = moveTowardsUserPID.calculate(imageTarget.z, laserData.vector.x / 1000.0);
                    output.twist_linear_x += forward * sin(-orientation.z);
                    output.twist_linear_y += forward * cos(-orientation.z);
                    //Regler für Rotation in der Horizontalen
                    output.twist_angular_y = rotHorizontalPID.calculate(imageTarget.x, faces.facegeos[0].face_points[51].x);
                    //ROS_INFO_STREAM(targetX << ";  " << faces.facegeos[0].face_points[51].x);
                    //Regler zur linearen Bewegung in der Vertikalen
                    output.twist_linear_z += 0.3 * linearVertPID.calculate(imageTarget.y, faces.facegeos[0].face_points[51].y);
                    break;
                }
                }
            }
            if (mode.y == 1)
            {
                //convert geo msgs Pose to 2 Vector3 types
                geometry_msgs::Vector3 target_position, target_orientation;
                tf::Quaternion quat2;
                tf::quaternionMsgToTF(poseTarget.orientation, quat2);
                quat2.normalize();
                tf::Matrix3x3(quat2).getRPY(target_orientation.y, target_orientation.x, target_orientation.z);
                target_position.x = poseTarget.position.x;
                target_position.y = poseTarget.position.y;
                target_position.z = poseTarget.position.z;

                //velocity controller for position
                //output.twist_linear_x = positionXPID.calculate(target_position.x, position.x);
                //output.twist_linear_y = positionYPID.calculate(target_position.y, position.y);
                //output.twist_linear_z = positionZPID.calculate(target_position.z, position.z);

                //controller for roll and pitch of the tool
                output.twist_angular_z = angularOrientZPID.calculate(target_orientation.y, orientation.y); //roll
                ROS_INFO_STREAM(target_orientation << "  " << orientation);
                //output.twist_angular_x = angularOrientXPID.calculate(target_orientation.x, orientation.x);
                // output.twist_angular_y = angularOrientYPID.calculate(target_orientation.z, orientation.z);

                double positionError = sqrt(pow(positionXPID.getError(), 2) +
                                            pow(positionYPID.getError(), 2) +
                                            pow(positionZPID.getError(), 2));

                if (positionError < 0.01)
                {
                    confirmPub.publish(0);
                }
                else
                {
                    confirmPub.publish(1);
                }
            }
            //permanent controller
            if (mode.z >= 10 && mode.y == 0.0)
            {
            }
        }

        //publish data to move the robot and sleep
        jacoCon.publish(output);
        ros::spinOnce();
        r.sleep();
    }
}

float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2)
{
    return double(sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2)));
}
