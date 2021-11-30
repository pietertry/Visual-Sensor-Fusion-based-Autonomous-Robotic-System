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
#include <geometry_msgs/Transform.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/utils.h>

using namespace std;

//Globale Variablen
//Variable die den Status des Lasers speichert
icog_face_tracker::facegeos faces;       //Data from face Tracking
kinova_msgs::PoseVelocity output;        //Variable for moving the Kinova Robot
geometry_msgs::Vector3Stamped laserData; //Data from Laser Sensor (Distance, Status, ROI)
geometry_msgs::PoseStamped toolPose;     //Tool Pose

bool newTrackingData = false; //Variable zum auslösen des Roboter Reglers
bool newPoseData = false;
bool newLaserData = false;
bool gotCameraInfo = false;

//Transformation between Lasersensor and IR Camera 1 (the second one)
cv::Mat_<double> RTMatrix = cv::Mat_<double>(3, 4);
geometry_msgs::Transform cameraLaserTranformation;

//Matrix for Intrinsics of the Camera
cv::Mat_<double> A = cv::Mat_<double>(3, 3);

//global parameters
geometry_msgs::Vector3 target_orientation; //Tool orientation in euler
geometry_msgs::Vector3 toolPose_position;  //tool position

//Publisher
ros::Publisher camCoordinatePub;

//Funktionsdeklaration
float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2);
void setRTMatrix();
double setLaserAngle(int ROI);

//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

void laserCB(const geometry_msgs::Vector3Stamped &msg)
{
    laserData = msg;
    newLaserData = true;
}

void toolPoseCB(const geometry_msgs::PoseStamped &msg)
{
    toolPose = msg;
    newPoseData = true;
}

void infraCB(const sensor_msgs::CameraInfo &msg)
{
    if (!gotCameraInfo)
    {
        vector<double> P(msg.P.begin(), msg.P.end());

        int z = 0;
        for (int i = 0; i < A.rows; i++)
        {
            for (int j = 0; j < A.cols; j++)
            {
                A.at<double>(i, j) = P.at(z);
                z++;
            }
            z++;
        }

        gotCameraInfo = true;
    }
}

int main(int argc, char **argv)
{

    //Ros Handle initiieren
    ros::init(argc, argv, "laserRecon_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 5, laserCB);
    ros::Subscriber toolPoseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 5, toolPoseCB);
    ros::Subscriber cameraInfoSub = nh.subscribe("/d435/CameraInfo/Infra", 5, infraCB);

    //Publisher
    camCoordinatePub = nh.advertise<geometry_msgs::Vector3>("/facegeos/camCoordinate", 5);
    ros::Publisher cameraCoordinates = nh.advertise<geometry_msgs::Vector3Stamped>("/mouthTracking/cameraCoordinates", 5);
    ros::Publisher cameraLaserTransform = nh.advertise<geometry_msgs::Transform>("/mouthTracking/camerTranformation", 5);
    //Set RT Rotation and Translation Matrix
    setRTMatrix();

    //Kinova publisher
    ros::Rate r(30); // 100 hz
    while (ros::ok())
    {

        if (newLaserData && !A.empty() && laserData.vector.y == 0)
        {
            geometry_msgs::Vector3Stamped output;
            double laserAngle = setLaserAngle((int)laserData.vector.z);

            //calculate XYZ Point (in Lasercoordinate)
            cv::Vec4d point;
            point[0] = 0;
            point[1] = sin(laserAngle / 180 * M_PI) * laserData.vector.x / 1000;
            point[2] = cos(laserAngle / 180 * M_PI) * laserData.vector.x / 1000;
            point[3] = 1;

            //calculate XYZ Point in Camera coordinate System
            cv::Mat coordinate = RTMatrix * cv::Mat(point);
            coordinate = coordinate / coordinate.at<double>(2, 0); //X UND Y MIT Z NORMIEREN!!!!!!!!!!!!!!!!!
            //calculate frame coordinate uv (x = u, y = v, z = width of Laser)
            cv::Mat pixelCoord = A * coordinate;

            //width of laser in camerapixels
            cv::Vec4d width;
            width[0] = tan(4.0 / 180.0 * M_PI) * laserData.vector.x / 1000;
            width[1] = sin(laserAngle / 180 * M_PI) * laserData.vector.x / 1000;
            width[2] = cos(laserAngle / 180 * M_PI) * laserData.vector.x / 1000;
            width[3] = 1;
            cv::Mat widthCoordinate = RTMatrix * cv::Mat(width);
            widthCoordinate = widthCoordinate / widthCoordinate.at<double>(2, 0);
            cv::Mat widthCoord = A * widthCoordinate;
            //cout << point[0] - width[0] << endl;
            //get width by calculating the distance between the pixels
            output.vector.z = 2 * sqrt(pow(widthCoord.at<double>(0, 0) -
                                               pixelCoord.at<double>(0, 0),
                                           2) +
                                       pow(widthCoord.at<double>(1, 0) -
                                               pixelCoord.at<double>(1, 0),
                                           2));
            //Save frame coordinate to msg
            output.header.frame_id = "/camera_reconstruction";
            output.header.stamp = ros::Time::now();
            output.vector.x = pixelCoord.at<double>(0, 0);
            output.vector.y = pixelCoord.at<double>(1, 0);
            //output.vector.z = pixelCoord.at<double>(2, 0);

            //Publish frame coordinates
            cameraCoordinates.publish(output);
            //Publish laser to camera tranformation
            cameraLaserTransform.publish(cameraLaserTranformation);
        }
        else
        {
            geometry_msgs::Vector3Stamped output;
            //Publish frame coordinates
            cameraCoordinates.publish(output);
        }

        //camCoordinatePub.publish(output);
        ros::spinOnce();
        r.sleep();
    }
}

float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2)
{
    return double(sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2)));
}

double setLaserAngle(int ROI)
{
    double laserAngle;
    switch (ROI)
    {
    case 1930404:
        laserAngle = 7.4688;
        break;
    case 1960404:
        laserAngle = 3.7107;
        break;
    case 1990404:
        laserAngle = 0;
        break;
    case 610404:
        laserAngle = -4.2399;
        break;
    case 580404:
        laserAngle = -7.9242;
        break;
    default:
        laserAngle = 0;
        break;
    }
    return laserAngle;
}

void setRTMatrix()
{
    geometry_msgs::Vector3 laserToCamera;
    laserToCamera.x = 0.0175;
    laserToCamera.y = -0.021;
    laserToCamera.z = -0.02205;

    //Rotation Translation Matrix (in form of a vector) for Transformation between laser sensor and Camera
    //rotation -7.5° around x axis
    std::vector<double> dummyData = {1.0000000, 0.0000000, 0.0000000, laserToCamera.x,
                                     0.0000000, 0.9914449, 0.1305262, laserToCamera.y,
                                     0.0000000, -0.1305262, 0.9914449, laserToCamera.z};

    //copy vector to CV Matrix form
    int z = 0;
    for (int i = 0; i < RTMatrix.rows; i++)
    {
        for (int j = 0; j < RTMatrix.cols; j++)
        {
            RTMatrix.at<double>(i, j) = dummyData.at(z);
            z++;
        }
    }
    tf2::Quaternion rotationQuat;
    rotationQuat.setRPY(-7.0 / 180 * M_PI, 0, 0);
    rotationQuat.normalize();
    cameraLaserTranformation.translation = laserToCamera;
    cameraLaserTranformation.rotation = tf2::toMsg(rotationQuat);
}

/***** output a matrix in cout
cout << endl;
for (int i = 0; i < A.rows; i++)
{
    for (int j = 0; j < A.cols; j++)
    {
        cout << A.at<double>(i, j) << "; ";
    }
    cout << endl;
}
cout << "___________________________" << endl;
*/

/*
    if (newPoseData)
{
    //Get the Orientation of the robot tool and convert from quaternion to euler angles for easier control
    geometry_msgs::Vector3 orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(toolPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(orientation.x, orientation.y, orientation.z);
    newPoseData = false;
}
*/