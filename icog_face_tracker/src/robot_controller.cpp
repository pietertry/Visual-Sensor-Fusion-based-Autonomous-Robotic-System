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
    PID rotHorizontalPID = PID(0.01, 0.3, -0.3, 0.015, 0.0002, 0.00003);
    PID linearVertPID = PID(0.01, 0.1, -0.1, 0.002, 0.0001, 0.000001);
    PID striveHorizontalPID = PID(0.01, 0.1, -0.1, 0.0018, 0.0008, 0.00001);

    PID faceAlignmentPID = PID(0.01, 0.08, -0.08, 0.20, 0.001, 0.001);
    PID moveTowardsUserPID = PID(0.01, 0.03, -0.03, 2.5, 0.001, 0.0002);

    PID angularOrientZPID = PID(0.01, 0.3, -0.3, 2.6, 0.0, 0.001); //Controller for Tool ROll
    PID angularOrientXPID = PID(0.01, 0.3, -0.3, 2.6, 0.0, 0.001); //Controller for Tool PITCH
    PID angularOrientYPID = PID(0.01, 0.3, -0.3, 2.0, 0.0, 0.001); //Controller for Tool YAW
    PID positionXPID = PID(0.01, 0.1, -0.1, 2.5, 0.0, 0.002);      //Controller for Tool ROll
    PID positionYPID = PID(0.01, 0.1, -0.1, 2.5, 0.0, 0.002);      //Controller for Tool PITCH
    PID positionZPID = PID(0.01, 0.1, -0.1, 2.5, 0.0, 0.002);      //Controller for Tool YAW

    //publisher initialisieren
    ros::Publisher jacoCon = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 5);
    ros::Publisher confirmPub = nh.advertise<std_msgs::Int8>("/mouthTrackig/roboComStatus", 5);

    double setYawAngle;

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
                    output.twist_angular_y = rotHorizontalPID.calculate(imageTarget.x, faces.facegeos[0].face_points[62].x);
                    //Regler zur linearen Bewegung in der Vertikalen
                    output.twist_linear_z = linearVertPID.calculate(imageTarget.y, faces.facegeos[0].face_points[62].y);
                    //Regelung um den Becher gerade zum Gesicht zu orientieren
                    double controllerValue = faceAlignmentPID.calculate(0, -faces.facegeos[0].yaw);
                    output.twist_linear_x += controllerValue * cos(-orientation.z);
                    output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);

                    double positionError = sqrt(pow(output.twist_linear_x, 2) +
                                                pow(output.twist_linear_y, 2));
                    //Roll and Pitch controller
                    output.twist_angular_z = -angularOrientZPID.calculate(0, orientation.y);
                    output.twist_angular_x = angularOrientXPID.calculate(M_PI / 2, orientation.x);
                    //ROS_INFO_STREAM(positionError << "   " << faceAlignmentPID.getError());
                    if (abs(faceAlignmentPID.getError()) < 0.3)
                    {
                        setYawAngle = orientation.z;
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
                    output.twist_angular_y = angularOrientYPID.calculate(setYawAngle, orientation.z);
                    //Regler zur linearen Bewegung in der Vertikalen
                    output.twist_linear_z += linearVertPID.calculate(imageTarget.y, faces.facegeos[0].face_points[62].y);
                    //Regler für seitliche horizontale bewegung
                    double controllerValue = striveHorizontalPID.calculate(imageTarget.x, faces.facegeos[0].face_points[62].x);
                    output.twist_linear_x += controllerValue * cos(-orientation.z);
                    output.twist_linear_y += -1 * controllerValue * sin(-orientation.z);

                    //Roll and Pitch controller
                    output.twist_angular_z = -angularOrientZPID.calculate(0, orientation.y);
                    output.twist_angular_x = angularOrientXPID.calculate(M_PI / 2, orientation.x);

                    //Get position Error
                    double positionError = sqrt(pow(output.twist_linear_x, 2) +
                                                pow(output.twist_linear_y, 2));
                    if (positionError < 0.03)
                    {
                        confirmPub.publish(0);
                    }
                    else
                    {
                        confirmPub.publish(1);
                    }

                    break;
                }
                }
            }
            if (10 <= mode.y && mode.y < 20)
            {
                switch ((int)mode.y)
                {
                case 10:
                {
                    //convert geometry msgs Pose to position and orientation (Pose.orientation is usually given in quaternion, however these are euler angles)
                    geometry_msgs::Vector3 target_position, target_orientation;
                    target_position.x = poseTarget.position.x;
                    target_position.y = poseTarget.position.y;
                    target_position.z = poseTarget.position.z;
                    target_orientation.x = poseTarget.orientation.x;
                    target_orientation.y = poseTarget.orientation.y;
                    target_orientation.z = poseTarget.orientation.z;

                    //velocity controller for position
                    output.twist_linear_x = positionXPID.calculate(target_position.x, position.x);
                    output.twist_linear_y = positionYPID.calculate(target_position.y, position.y);
                    output.twist_linear_z = positionZPID.calculate(target_position.z, position.z);

                    //controller for roll and pitch of the tool
                    output.twist_angular_z = -angularOrientZPID.calculate(target_orientation.y, orientation.y); //roll
                    output.twist_angular_x = angularOrientXPID.calculate(target_orientation.x, orientation.x);
                    output.twist_angular_y = angularOrientYPID.calculate(target_orientation.z, orientation.z);

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
                break;
                case 11:
                {
                }
                break;
                }
            }
            //permanent controller
            if (mode.z >= 10 && mode.y == 0.0)
            {
            }
        }

        //publish data to move the robot and sleep
        if (mode.x != 0 || mode.y != 0 || mode.z != 0)
        {
            jacoCon.publish(output);
        }
        ros::spinOnce();
        r.sleep();
    }
}

float getDistance(icog_face_tracker::myPoint point1, icog_face_tracker::myPoint point2)
{
    return double(sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2)));
}

/*
  //move towards user
                    double forward = moveTowardsUserPID.calculate(imageTarget.z, laserData.vector.x / 1000.0);
                    output.twist_linear_x += forward * sin(-orientation.z);
                    output.twist_linear_y += forward * cos(-orientation.z);
                    //Regler für Rotation in der Horizontalen
                    output.twist_angular_y = rotHorizontalPID.calculate(imageTarget.x, faces.facegeos[0].face_points[51].x);
                    //ROS_INFO_STREAM(targetX << ";  " << faces.facegeos[0].face_points[51].x);
                    //Regler zur linearen Bewegung in der Vertikalen
                    output.twist_linear_z += 0.3 * linearVertPID.calculate(imageTarget.y, faces.facegeos[0].face_points[51].y);

*/