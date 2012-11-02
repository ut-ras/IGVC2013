#ifndef _NAOJOY_CONTROL_
#define _NAOJOY_CONTROL_

#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/PointHeadGoal.h>

#include "PR2JointController/ArmVelocities.h"
//#include "PR2JointController/HeadVelocities.h"

#include "BrownJoyControlBase.h"


using namespace std;
using namespace brown_ros;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

namespace brown_ros
{

class PR2JoyControlNode : public BrownJoyControlBase
{
    //Command codes for robot's actions. There are different commands for different
    //sources - buttons and axes
    //Important note: command code is also an index of the corresponding string
    //in the m_arrConfigCommandStrings array
    enum CommandCodes 
    {
        ccConfig0 = 0,          //Enable configuration 0
        ccConfig1 = 1,

        ccMoveForward = 2,      //Button: walk forwards
        ccMoveBackward = 3,     //Button: walk backwards
        ccTurnLeft = 4,         //Button: turn right
        ccTurnRight = 5,        //Button: turn right
        ccAxisMove = 6,         //Axis: walk forwards/backwards
        ccAxisTurn = 7,         //Axis: turn right/left

        ccHeadUp = 8,           //Button: head up
        ccHeadDown = 9,         //Button: head down
        ccHeadLeft = 10,        //Button: head left
        ccHeadRight = 11,       //Button: head right
        ccAxisHeadPitch = 12,   //Axis: control head pitch
        ccAxisHeadYaw = 13,     //Axis: control head yaw

        ccRUpperArmRollRight = 14,  //Button: roll right upper arm rightwards
        ccRUpperArmRollLeft = 15,   //Button: roll right upper arm leftwards
        ccRShoulderRollRight = 16,  //Button: roll right shoulder rightwards
        ccRShoulderRollLeft = 17,   //Button: roll right shoulder leftwards
        ccRShoulderPitchUp = 18,    //Button: pitch right shoulder upwards
        ccRShoulderPitchDown = 19,  //Button: pitch right shoulder downwards
        ccRForearmRollRight = 20,   //Button: roll right elbow rightwards
        ccRForearmRollLeft = 21,    //Button: roll right elbow leftwards
        ccRElbowFlexRight = 22,     //Button: yaw right elbow rightwards
        ccRElbowFlexLeft = 23,      //Button: yaw right elbow leftwards

        ccAxisRUpperArmRoll = 24,   //Axis: control right upper arm roll
        ccAxisRShoulderRoll = 25,   //Axis: control right sholder roll 
        ccAxisRShoulderPitch = 26,  //Axis: control right sholder pitch
        ccAxisRForearmRoll = 27,    //Axis: control right elbow roll
        ccAxisRElbowFlex = 28,      //Axis: control right elbow yaw
        ccAxisRWristRoll = 29,      //Axis: control right wrist roll
        ccAxisRWristPitch = 30,     //Axis: control right wrist pitch

        ccRWristRollLeft = 31,      //Button: turn right wrist leftwards
        ccRWristRollRight = 32,     //Button: turn right wrist rightwards
        ccRWristPitchUp = 33,       //Button: turn right wrist up
        ccRWristPitchDown = 34,     //Button: turn right wrist down
        ccRGraspCommand = 35,       //Button: open/close right hand

        ccLUpperArmRollRight = 36,  //Button: roll left upper arm rightwards
        ccLUpperArmRollLeft = 37,   //Button: roll left upper arm rightwards
        ccLShoulderRollRight = 38,  //Button: roll left shoulder rightwards
        ccLShoulderRollLeft = 39,   //Button: roll left shoulder leftwards
        ccLShoulderPitchUp = 40,    //Button: pitch left shoulder upwards
        ccLShoulderPitchDown = 41,  //Button: pitch left shoulder downwards
        ccLForearmRollRight = 42,   //Button: roll left elbow rightwards
        ccLForearmRollLeft = 43,    //Button: roll left elbow leftwards
        ccLElbowFlexRight = 44,     //Button: yaw left elbow rightwards
        ccLElbowFlexLeft = 45,      //Button: yaw left elbow leftwards

        ccAxisLUpperArmRoll = 46,   //Axis: control right upper arm roll
        ccAxisLShoulderRoll = 47,   //Axis: control left sholder roll 
        ccAxisLShoulderPitch = 48,  //Axis: control left sholder pitch
        ccAxisLForearmRoll = 49,    //Axis: control left elbow roll
        ccAxisLElbowFlex = 50,      //Axis: control left elbow yaw
        ccAxisLWristRoll = 51,      //Axis: control left wrist roll
        ccAxisLWristPitch = 52,     //Axis: control left wrist roll

        ccLWristRollLeft = 53,      //Button: turn left wrist leftwards
        ccLWristRollRight = 54,     //Button: turn left wrist rightwards
        ccLWristPitchUp = 55,       //Button: turn right wrist up
        ccLWristPitchDown = 56,     //Button: turn right wrist down
        ccLGraspCommand =  57,      //Button: open/close left hand
    
        ccStop = 58,
        ccUnknown = -1
    } m_cmdCodeEnum;

    //Total number of control codes that can be sent to the robot from the
    //joystick
    static const int m_iNumControlCodes = 60;

    //Configuration strings that denote robot's actions. These strings will be used to map
    //robot's actions to joystick buttons and axes via ROS configuration parameters.
    //
    //IMPORTANT NOTE:
    // Strings are mapped to commands in the order of their command codes
    // E.g. 0-th string in the ConfigCommandStrings denotes command with command code 0,
    // 1st string - command with code 2 
    // and so on
    static const string m_arrConfigCommandStrings[m_iNumControlCodes]; 

    ///
    /// Min and max values for head angles:
    ///
    // Head yaw: +- 90 degrees
    static const double cdblMinHeadYaw=-1.571;
    static const double cdblMaxHeadYaw=1.571;
    static const double cdblMinHeadPitch=-1;
    static const double cdblMaxHeadPitch=1;
    static const double cdblCameraHeight = 1.25;

    private:
    ///
    ///Publishers for NAO control topics:
    ///
    ///Walk - walking control
    ros::Publisher m_pubBase;
    //HeadP - head movement control
    //ros::Publisher m_pubHead;

    //RArmAngles topic - right arm control 
    ros::Publisher m_pubRArmEffort;
    //LArmAngles topic - left arm control 
    ros::Publisher m_pubLArmEffort;

    //Right gripper control 
    ros::Publisher m_pubRGripper;
    //Left gripper control 
    ros::Publisher m_pubLGripper;

    //Pointer to the robot head client
    PointHeadClient* m_pPointHeadClient;


    ///
    ///Values of commands to be sent to the robot
    ///
    //Motion command
    double m_dbl_vx;
    double m_dbl_vy;
    double m_dbl_vz;
    //Head command
    double m_dblHeadXCommand;
    double m_dblHeadYCommand;

    //Right hand commands
    double m_dblRUpperArmRoll;
    double m_dblRShoulderPitch;
    double m_dblRShoulderRoll;
    double m_dblRForearmRoll;
    double m_dblRElbowFlex;
    double m_dblRWristRoll;
    double m_dblRWristPitch;
    //Current state of the right grasper
    int m_iRGraspState;
    
    //Left hand commands
    double m_dblLUpperArmRoll;
    double m_dblLShoulderPitch;
    double m_dblLShoulderRoll;
    double m_dblLForearmRoll;
    double m_dblLElbowFlex;
    double m_dblLWristRoll;
    double m_dblLWristPitch;

    //Current state of the left grasper
    int m_iLGraspState;
    
    //Current and previous head angles
    double m_dblCurrentHeadYaw;
    double m_dblCurrentHeadPitch;
    double m_dblPrevHeadYaw;
    double m_dblPrevHeadPitch;
    //Current state of the controller for head pitch/yaw 
    //0: no command
    //+-1 - command is issued
    int m_iHeadPitchState;
    int m_iHeadYawState;
    
    ///
    /// Properties of commands to be sent to the robot
    ///
    //Gripper options: open/closed position and closing effort
    double m_dblGripperOpenPosition;
    double m_dblGripperClosePosition;
    double m_dblGripperMaxEffort;
    //Increment for head movement
    double m_dblHeadAngleInc;
    //Robot linear and angular speed values
    double m_dblLinearSpeed;
    double m_dblAngularSpeed;

    //Robot joint effort values
    double m_dblUpperArmEffort;
    double m_dblShoulderEffort;
    double m_dblElbowEffort;
    double m_dblForearmEffort;
    double m_dblWristEffort;

    public:
    PR2JoyControlNode();
    ~PR2JoyControlNode();

    void ReadParameters();
    void Initialize();
        
    virtual int GetNumControlCodes();
    virtual string GetStringForControlCode(int iIdx);
    virtual void InitializeCommand();
    virtual void OnCommandReceived(int iCommandID, double dblCommandValue);
    virtual void SendCommandToRobot();
    
    void SetHeadCoordinates(double dblPitch, double dblYaw);
};

//Initializing configuration strings array. Cannot do this inside of class
const string PR2JoyControlNode::m_arrConfigCommandStrings[m_iNumControlCodes] = 
{
    "config0",  //Enable configuration 0
    "config1",  //Enable configuration 1
        
    "moveforward",  
    "movebackward",
    "turnleft",
    "turnright",
    "axis_move",
    "axis_turn",
        
    "headup",
    "headdown",
    "headleft",
    "headright",
    "axis_headpitch",
    "axis_headyaw",

    "rupperarmrollright",
    "rupperarmrollleft",
    "rshoulderrollright",
    "rshoulderrollleft",
    "rshoulderpitchup",
    "rshoulderpitchdown",
    "rforearmrollright",
    "rforearmrollleft",
    "relbowflexright",
    "relbowflexleft",

    "axis_rupperarmroll",
    "axis_rshoulderroll",
    "axis_rshoulderpitch",
    "axis_rforearmroll",
    "axis_relbowflex",
    "axis_rwristroll",
    "axis_rwristpitch",

    "rwristrollleft",
    "rwristrollright",
    "rwristpitchup",
    "rwristpitchdown",
    "rgraspcommand",

    "lupperarmrollright",
    "lupperarmrollleft",
    "lshoulderrollright",
    "lshoulderrollleft",
    "lshoulderpitchup",
    "lshoulderpitchdown",
    "lforearmrollright",
    "lforearmrollleft",
    "lelbowflexright",
    "lelbowflexleft",

    "axis_lupperarmroll",
    "axis_lshoulderroll",
    "axis_lshoulderpitch",
    "axis_lforearmroll",
    "axis_lelbowflex",
    "axis_lwristroll",
    "axis_lwristpitch",

    "lwristrollleft",
    "lwristrollright",
    "lwristpitchup",
    "lwristpitchdown",
    "lgraspcommand",

    "stop",

    "unknown",
};

}

#endif

