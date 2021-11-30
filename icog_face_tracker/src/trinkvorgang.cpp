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
geometry_msgs::Vector3Stamped laserData; //Data from Laser Sensor (Distance, Status, ROI)
//geometry_msgs::PoseStamped toolPose;     //Tool Pose
sensor_msgs::Joy joyData;
geometry_msgs::Vector3 tacterionData;
int controllerStatus;

bool newTrackingData = false; //Variable zum auslösen des Roboter Reglers
bool newPoseData = false;
bool newJoyData = false;
bool newLaserData = false;
bool newLaserReconData = false;
bool newTacterionData = false;
//Variables for Tacterion average
double tacterionSum = 0;
double tacterionAverage = 0;
int numTacterion = 0;

geometry_msgs::Vector3 toolPose_position;
geometry_msgs::Vector3Stamped laserField;

//int mode = 0; //Steuerungsmodus

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
        newTacterionData = true;
    }
}

int main(int argc, char **argv)
{
    //Ros Handle initiieren
    ros::init(argc, argv, "jaco_node");
    ros::NodeHandle nh;

    //Subscriber initialisieren
    ros::Subscriber facesSub = nh.subscribe("/facegeos", 1, trackerCB);                                           //Face Tracking Data
    ros::Subscriber laserSub = nh.subscribe("/VL53L1X/data", 1, laserCB);                                         //Ranging Sensor Data
                                                                                                                  // ros::Subscriber toolPoseSub = nh.subscribe("/j2s7s300_driver/out/tool_pose", 1, toolPoseCB);                  //Current Tool Pose
    ros::Subscriber joySub = nh.subscribe("/joy", 1, joyCB);                                                      //Joystick Sub
    ros::Subscriber laserReconSub = nh.subscribe("/mouthTracking/cameraCoordinates", 1, laserReconCB);            //Laser projection transformation sub
    ros::Subscriber roboControllerStatusSub = nh.subscribe("/mouthTrackig/roboComStatus", 1, controllerStatusCB); //Jaco Controller Status
    ros::Subscriber tacterionSub = nh.subscribe("/tacterion/data", 1, tacterionCB);                               //Subscriber for Tacterion capacative values
    //publisher initialisieren
    ros::Publisher jacoCon = nh.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 5);
    ros::Publisher laser_setROI = nh.advertise<geometry_msgs::Vector3>("/VL53L1X/setROI", 5);
    ros::Publisher imageServoTargetPub = nh.advertise<geometry_msgs::Vector3>("/mouthTracking/imageServoingTarget", 5);
    ros::Publisher controllerModePub = nh.advertise<geometry_msgs::Vector3>("/mouthTracking/mode", 5);
    ros::Publisher toolPoseTargetPub = nh.advertise<geometry_msgs::Pose>("/mouthTracking/toolPoseTarget", 5);

    //Robotername und steuerungstopic j2s7s300_driver/in/cartesian_vecity kinova_msgs/PoseVelocity

    //Kinova publisher
    ros::Rate r(60); // 100 hz
    while (ros::ok())
    {
        ROS_INFO_STREAM("State Machine Reseted");
        //state machine variables
        int state = 0;
        double cupDistance = 0;
        double mouthTargetDistance = 0;
        //double positionError = 10;
        double cupDistSum = 0;
        int numCupRangings = 0;

        geometry_msgs::Vector3 lastTacterionData;
        double tacterionCapacativeDiff;
        //timer to save last time
        uint64_t timer = 0;
        //laserROI
        std::vector<int> laserROI = {199, 04, 04};

        //Home Position and Orientation
        geometry_msgs::Point HOMEPOSITION;
        HOMEPOSITION.x = -0.00638998206704855;
        HOMEPOSITION.y = -0.6172771453857422;
        HOMEPOSITION.z = 0.05030285567045212;
        geometry_msgs::Quaternion HOMEORIENTATION;
        HOMEORIENTATION.y = 0;        //Roll
        HOMEORIENTATION.x = M_PI / 2; //Pitch
        HOMEORIENTATION.z = 0;        //YAW

        state = 999;
        //state 0 reset all values
        //state 999 go to home Position
        //state 1 check for user
        //state 2 get Cup Position
        //state 21 change back Laser ROI
        //state 3 align robot with users face
        //state 4 approach user
        //state 5 stop and stare
        while (state != 0 && ros::ok())
        {
            //Stopp with A Button on joy
            if (newJoyData && joyData.buttons[2] == 1)
            {
                state = 0;
                newJoyData = false;
            }

            if (newTacterionData)
            {
                tacterionCapacativeDiff = (tacterionData.x - lastTacterionData.x) / 10.0;
                lastTacterionData = tacterionData;
            }

            switch (state)
            {
                //CASE 0___________________________________________________________________________________
            case 999:
            {
                //Move to home position and stay on standby

                //set robot controller mode
                geometry_msgs::Vector3 conMode;
                conMode.x = 0;
                conMode.y = 10;
                conMode.z = 0;
                //set target position and orientation to HOME
                geometry_msgs::Pose targetPose;
                targetPose.position = HOMEPOSITION;
                targetPose.orientation = HOMEORIENTATION;
                //Publish commands to Jaco_node
                toolPoseTargetPub.publish(targetPose);
                controllerModePub.publish(conMode);
                //transition condition: Joystick A Button is pressed and Home Position is reached
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
                //Check if user is in the frame and reasonably close and if the user is facing the robot
                if (!faces.facegeos.empty() && !faces.facegeos[0].facingAway)
                    if (faces.facegeos[0].face.width > 60 && !faces.facegeos[0].facingAway)
                    {
                        //Skip measuring the Cup for now, since the cup will not be removed
                        state = 21;
                        //Set distance to cup/ mouth target distance
                        mouthTargetDistance = 0.223;
                        ROS_INFO_STREAM("State: " << state);
                        ROS_INFO_STREAM("State 21: Changing the laser ROI to 1990404");
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
                //Check if Laser ROI is set to Center 193 Width 4 Height 4 and then procede to measure distance to cup
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
                                mouthTargetDistance = cupDistance + 0.00; //distance + cup thickness
                                cupDistance = 0;
                                numCupRangings = 0;
                                cupDistSum = 0;
                                state = 21;
                                ROS_INFO_STREAM("State: " << state << "   Mouthtarget distance: " << mouthTargetDistance);
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
                        cupDistance = 0;
                        numCupRangings = 0;
                        cupDistSum = 0;
                        state = 0;
                        ROS_INFO_STREAM("State: " << state << "   No Cup Detected");
                    }
                }
            }
            break;
            case 21:
            {
                laserROI = {199, 4, 4};
                //Transition to state 3 if the Laser ROI is set back to center 199 width 4 height 4
                if (laserData.vector.z == (laserROI.at(0) * 10000 + laserROI.at(1) * 100 + laserROI.at(2)))
                {
                    state = 3;
                    ROS_INFO_STREAM("State: " << state);
                    ROS_INFO_STREAM("Align the Robot with the users face");
                }
            }
            break;

                //CASE 3___________________________________________________________________________________
            case 3:
            {
                //continue when there is valid data
                if (!faces.facegeos.empty())
                {
                    //set Robot Controller to visual servoing for face alignment mode
                    geometry_msgs::Vector3 conMode;
                    conMode.x = 10.0;
                    conMode.y = 0;
                    conMode.z = 0;
                    //inital estimate for laser measurement position
                    geometry_msgs::Vector3 imageTarget;
                    imageTarget.x = 260;
                    imageTarget.y = 150;
                    //use Laser position in image when available
                    if (laserField.vector.x + laserField.vector.y != 0)
                    {
                        //Position of laser measurement area
                        imageTarget.x = laserField.vector.x;
                        imageTarget.y = laserField.vector.y;
                    }
                    //Publish commands to Jaco_node
                    imageServoTargetPub.publish(imageTarget);
                    controllerModePub.publish(conMode);
                    //Stop Condition: Face Tracking failed 5 times (5*1/30s =166ms)
                    if (((int)faces.facegeos[0].failed_attempts > 4) || faces.facegeos[0].facingAway)
                    {
                        state = 0;
                        ROS_INFO_STREAM("State: " << state << "  User stopped the Alignment process:  " << (int)faces.facegeos[0].failed_attempts << "; " << (int)faces.facegeos[0].facingAway);
                    }

                    //Transition condition: If the robot is aligned for 2 seconds, then start approach
                    if (controllerStatus == 0 && timer == 0)
                    {
                        //start timer when it is not running
                        timer = ros::Time::now().toNSec();
                    }
                    else if (controllerStatus == 0)
                    {
                        //ROS_INFO_STREAM(controllerStatus << "  " << (ros::Time::now().toNSec() - timer) << "  " << timer << "  " << ros::Time::now().toNSec());
                        //if face still aligned, then check if timer is run out
                        if (2000000000 < (ros::Time::now().toNSec() - timer))
                        {
                            timer = 0;
                            state = 4;
                            ROS_INFO_STREAM("State: " << state);
                            ROS_INFO_STREAM("Approaching the user");
                            //Pause RObot
                            geometry_msgs::Vector3 conMode;
                            conMode.x = 0;
                            conMode.y = 0;
                            conMode.z = 0;
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
                //Set controller mode
                geometry_msgs::Vector3 conMode;
                conMode.x = 11;
                conMode.y = 0;
                conMode.z = 0;

                geometry_msgs::Vector3 imageTarget;
                imageTarget.x = 240;
                imageTarget.y = 150;

                if ((laserField.vector.x + laserField.vector.y) != 0)
                {
                    imageTarget.x = laserField.vector.x;
                    imageTarget.y = laserField.vector.y;
                }
                if ((laserData.vector.x / 1000.0 - mouthTargetDistance) < 0.02)
                {
                    imageTarget.x -= 10;
                    imageTarget.y += 40;
                }

                imageTarget.z = mouthTargetDistance;

                //Publish commands to Jaco_node
                imageServoTargetPub.publish(imageTarget);
                controllerModePub.publish(conMode);

                //Stopp Condition
                if ((laserData.vector.x / 1000.0 - mouthTargetDistance) > 0.015)
                {
                    if (((int)faces.facegeos[0].failed_attempts > 10) || faces.facegeos[0].facingAway)
                    {
                        state = 0;
                        ROS_INFO_STREAM("State: " << state << "  User stopped the approach" << (int)faces.facegeos[0].failed_attempts << "; " << (int)faces.facegeos[0].facingAway);
                    }
                }
                //Stopp if user is too close
                if ((laserData.vector.x / 1000.0 - mouthTargetDistance) < -0.012)
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  user too close  " << mouthTargetDistance << "; " << (laserData.vector.x / 1000.0));
                }

                //Transition condition: When Position is reached and the Capacative Sensors triggers
                if (controllerStatus == 0 && tacterionCapacativeDiff < -0.3)
                {
                    timer = 0;
                    state = 5;
                    ROS_INFO_STREAM("State: " << state << "  reached user");
                    ROS_INFO_STREAM(mouthTargetDistance << "; " << (laserData.vector.x / 1000.0));

                    geometry_msgs::Vector3 conMode;
                    conMode.x = 0;
                    conMode.y = 0;
                    conMode.z = 0;
                    controllerModePub.publish(conMode);
                }
            }
            break;
                //CASE 5___________________________________________________________________________________
            case 5:
            {
                //Stopp if user is too close
                if (false && ((laserData.vector.x / 1000.0 - mouthTargetDistance) < -0.012))
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  user too close  " << mouthTargetDistance << "; " << (laserData.vector.x / 1000.0));
                }

                //Stop if the lips detach from the tacterion sensor
                if (false && ((tacterionCapacativeDiff > 0.3 && tacterionData.x > (tacterionAverage - 30)) || (tacterionCapacativeDiff > 0.15 && tacterionData.x > (tacterionAverage - 10))))
                {
                    state = 0;
                    ROS_INFO_STREAM("State: " << state << "  user detached from cup");
                }

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
            newTacterionData = false;
            ros::spinOnce();
            r.sleep();
        }
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

/*
                if ((controllerStatus == 0 && timer == 0) || !(tacterionCapacativeDiff > 0.2))
                {
                    timer = ros::Time::now().toNSec();
                }
                else if (controllerStatus == 0 || tacterionCapacativeDiff > 0.2)
                {
                    //ROS_INFO_STREAM(controllerStatus << "  " << (ros::Time::now().toNSec() - timer) << "  " << timer << "  " << ros::Time::now().toNSec());

                    if ((500000000 < (ros::Time::now().toNSec() - timer)) || tacterionCapacativeDiff > 0.3)
                    {
                        //ROS_INFO_STREAM("Reset");

                        timer = 0;
                        state = 5;
                        ROS_INFO_STREAM("State: " << state << "  reached user");
                        ROS_INFO_STREAM(mouthTargetDistance << "; " << (laserData.vector.x / 1000.0));

                        geometry_msgs::Vector3 conMode;
                        conMode.x = 0;
                        conMode.y = 0;
                        conMode.z = 1;
                        controllerModePub.publish(conMode);
                    }
                }
                else
                {
                    //ROS_INFO_STREAM("Reset");
                    timer = 0;
                }
                */