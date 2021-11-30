#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

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

//Variable die den Status des Lasers speichert
icog_face_tracker::facegeos faces;       //Data from face Tracking
geometry_msgs::Vector3Stamped laserData; //Data from Laser Sensor (Distance, Status, ROI)
//geometry_msgs::PoseStamped toolPose;     //Tool Pose
sensor_msgs::Joy joyData;
geometry_msgs::Vector3 tacterionData;
int controllerStatus;

geometry_msgs::Vector3 toolPose_position;
geometry_msgs::Vector3Stamped laserField;
geometry_msgs::Vector3 lastTacterionData;

geometry_msgs::Pose moCapHead;
geometry_msgs::Pose moCapRobo;
double tacterionCapacativeDiff;

//Variables for Tacterion average
double tacterionSum = 0;
double tacterionAverage = 0;
int numTacterion = 0;

bool newTrackingData = false; //Variable zum auslösen des Roboter Reglers
bool newPoseData = false;
bool newJoyData = false;
bool newLaserData = false;
bool newLaserReconData = false;
bool newTacterionData = false;

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
    //toolPose = msg;
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

void headPoseCB(const geometry_msgs::PoseStamped &msg)
{
    moCapHead = msg.pose;
}

void roboPoseCB(const geometry_msgs::PoseStamped &msg)
{
    moCapRobo = msg.pose;
}

void tacterionCB(const geometry_msgs::Vector3Stamped &msg)
{
    tacterionData = msg.vector;

    if (numTacterion < 20)
    {
        tacterionSum += tacterionData.x;
        numTacterion++;
        tacterionAverage = tacterionSum / (double)numTacterion;
    }
    else
    {
        tacterionCapacativeDiff = (tacterionData.x - lastTacterionData.x) / 10.0;
        lastTacterionData = tacterionData;
        newTacterionData = true;
    }
}

using namespace std;

double correctAngle(double value, double buffer)
{
    double output = 9999999;
    if (signbit(value) && !signbit(buffer))
        output = buffer - M_PI * 2.0;

    if (!signbit(value) && signbit(buffer))
        output = buffer + M_PI * 2.0;

    if (output == 9999999)
        output = buffer;

    if (value < M_PI / 4.0 && value > -M_PI / 4.0)
        output = buffer;

    //value = buffer;
    return output;
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "record_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber facesSub = nh.subscribe("/facegeos", 1, trackerCB);                                           //Face Tracking Data
    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);                                         //Ranging Sensor Data
                                                                                                                  // ros::Subscriber toolPoseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCB);                  //Current Tool Pose
    ros::Subscriber joySub = nh.subscribe("/joy", 1, joyCB);                                                      //Joystick Sub
    ros::Subscriber laserReconSub = nh.subscribe("/mouthTracking/cameraCoordinates", 1, laserReconCB);            //Laser projection transformation sub
    ros::Subscriber roboControllerStatusSub = nh.subscribe("/mouthTrackig/roboComStatus", 1, controllerStatusCB); //Jaco Controller Status
    ros::Subscriber tacterionSub = nh.subscribe("/tacterion/data", 1, tacterionCB);                               //Subscriber for Tacterion capacative values
    ros::Subscriber headPoseSub = nh.subscribe("/qualisys/head_pose_/pose", 1, headPoseCB);                       //Subscriber for Tacterion capacative values
    ros::Subscriber roboPoseSub = nh.subscribe("/qualisys/kinova_pose_/pose", 1, roboPoseCB);                     //Subscriber for Tacterion capacative values

    ros::spinOnce();

    int fileCounter = 0;
    ros::Rate r(100); // 100 hz
    while (ros::ok())
    {

        //start recording
        if (newJoyData && joyData.buttons[5] == 1)
        {
            fileCounter++;
            ROS_INFO_STREAM("start Recording number: " << fileCounter);
            double startTime = ros::Time::now().toSec();
            double timeOut = 0;
            std::ofstream myFile, myFileShort;
            int timestamp = (int)ros::Time::now().toSec();
            std::string filename = "/home/lukas/Pieter_Records/record_" + to_string(fileCounter) + "___" + to_string(timestamp) + ".csv";
            std::string filenameShort = "/home/lukas/Pieter_Records/short_record_" + to_string(fileCounter) + "___" + to_string(timestamp) + ".csv";

            ROS_INFO_STREAM(filename);
            myFile.open(filename);
            myFileShort.open(filenameShort);
            //Excel Sheet Header
            myFile << "Time,"
                   << "Roll_Head,"
                   << "Pitch_Head,"
                   << "Yaw_Head,"
                   << "Roll_Robo,"
                   << "Pitch_Robo,"
                   << "Yaw_Robo,"
                   << "isFaceTracked,"
                   << "isFacingCamera,"
                   << "failed Frames,"
                   << "stopped"
                   << endl;

            myFileShort << "Time,"
                        << "Roll_Head,"
                        << "Pitch_Head,"
                        << "Yaw_Head,"
                        << "Roll_Robo,"
                        << "Pitch_Robo,"
                        << "Yaw_Robo,"
                        << "isFaceTracked,"
                        << "isFacingCamera,"
                        << "failed Frames,"
                        << "stopped"
                        << endl;

            geometry_msgs::Vector3 moCapHeadOrientation;
            geometry_msgs::Point moCapHeadPosition;
            geometry_msgs::Vector3 moCapRoboOrientation;
            geometry_msgs::Point moCapRoboPosition;

            bool wasStopped = false;
            bool startOrientationSet = false;
            int numbStops = 0;

            while (ros::ok() && joyData.buttons[4] == 0)
            {
                //Get the Orientation (in RPY) and position of the robot tool

                tf::Quaternion quat;
                tf::quaternionMsgToTF(moCapHead.orientation, quat);
                quat.normalize();
                geometry_msgs::Vector3 buf1;
                tf::Matrix3x3(quat).getRPY(buf1.x, buf1.y, buf1.z);
                moCapHeadPosition = moCapHead.position;
                moCapHeadOrientation.x = correctAngle(moCapHeadOrientation.x, buf1.x);
                moCapHeadOrientation.y = correctAngle(moCapHeadOrientation.y, buf1.y);
                moCapHeadOrientation.z = correctAngle(moCapHeadOrientation.z, buf1.z);

                tf::Quaternion quat2;
                tf::quaternionMsgToTF(moCapRobo.orientation, quat);
                quat2.normalize();
                geometry_msgs::Vector3 buf2;
                tf::Matrix3x3(quat).getRPY(buf2.x, buf2.y, buf2.z);
                moCapRoboPosition = moCapRobo.position;
                moCapRoboOrientation.x = correctAngle(moCapRoboOrientation.x, buf2.x);
                moCapRoboOrientation.y = correctAngle(moCapRoboOrientation.y, buf2.y);
                moCapRoboOrientation.z = correctAngle(moCapRoboOrientation.z, buf2.z);

                //ROS_INFO_STREAM(moCapHeadOrientation.x * 180 / M_PI << ": " << moCapHeadOrientation.y * 180 / M_PI << ": " << moCapHeadOrientation.z * 180 / M_PI << ": " << moCapHeadOrientation.z * 180 / M_PI);

                //Get Variables from the Face Detection
                bool tracked = false;
                bool stopped = false;
                bool facingCamera = false;

                int failedFrames = 0;

                if (!faces.facegeos.empty())
                {
                    tracked = true;
                    facingCamera = faces.facegeos[0].facingAway;
                    failedFrames = (int)faces.facegeos[0].failed_attempts;
                    if (((int)faces.facegeos[0].failed_attempts > 10) || faces.facegeos[0].facingAway)
                    {
                        stopped = true;
                    }
                    else
                    {
                        wasStopped = false;
                    }
                }

                //<<","<<
                myFile << (ros::Time::now().toSec() - startTime) << ","
                       << moCapHeadOrientation.x / M_PI * 180.0 << ","
                       << moCapHeadOrientation.y / M_PI * 180.0 << ","
                       << moCapHeadOrientation.z / M_PI * 180.0 << ","
                       << moCapRoboOrientation.x / M_PI * 180.0 << ","
                       << moCapRoboOrientation.y / M_PI * 180.0 << ","
                       << moCapRoboOrientation.z / M_PI * 180.0 << ","
                       << tracked << ","
                       << facingCamera << ","
                       << failedFrames << ","
                       << stopped
                       << endl;

                //Startorientierung festlegen
                if (newJoyData && joyData.buttons[3] == 1 && !startOrientationSet)
                {
                    ROS_INFO_STREAM("Startorientierung festegelegt");
                    myFileShort << (ros::Time::now().toSec() - startTime) << ","
                                << moCapHeadOrientation.x / M_PI * 180.0 << ","
                                << moCapHeadOrientation.y / M_PI * 180.0 << ","
                                << moCapHeadOrientation.z / M_PI * 180.0 << ","
                                << moCapRoboOrientation.x / M_PI * 180.0 << ","
                                << moCapRoboOrientation.y / M_PI * 180.0 << ","
                                << moCapRoboOrientation.z / M_PI * 180.0 << ","
                                << tracked << ","
                                << facingCamera << ","
                                << failedFrames << ","
                                << stopped
                                << endl;
                    startOrientationSet = true;
                }

                if ((ros::Time::now().toSec() - timeOut) > 3)
                    timeOut = 0;
                if (stopped && !wasStopped && timeOut == 0)
                {
                    numbStops++;
                    ROS_INFO_STREAM(numbStops
                                    << ": Abbruchsignal erkannt: MissedFrames: "
                                    << (int)faces.facegeos[0].failed_attempts
                                    << "  Profilgesicht: "
                                    << (int)faces.facegeos[0].facingAway);

                    timeOut = ros::Time::now().toSec();
                    wasStopped = true;
                    myFileShort << (ros::Time::now().toSec() - startTime) << ","
                                << moCapHeadOrientation.x / M_PI * 180.0 << ","
                                << moCapHeadOrientation.y / M_PI * 180.0 << ","
                                << moCapHeadOrientation.z / M_PI * 180.0 << ","
                                << moCapRoboOrientation.x / M_PI * 180.0 << ","
                                << moCapRoboOrientation.y / M_PI * 180.0 << ","
                                << moCapRoboOrientation.z / M_PI * 180.0 << ","
                                << tracked << ","
                                << facingCamera << ","
                                << failedFrames << ","
                                << stopped
                                << endl;
                }

                ros::spinOnce();
                r.sleep();
            }
            ROS_INFO_STREAM("Stop Recording number: " << fileCounter);
            myFile.close();
            myFileShort.close();
        }
        ros::spinOnce();
        r.sleep();
    }
}