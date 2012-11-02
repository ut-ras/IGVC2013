#ifndef _NAOJOY_CONTROL_
#define _NAOJOY_CONTROL_


#include "BrownJoyControlBase.h"

#include "joy/Joy.h"
#include "NAOcontrol/Walk.h"
#include "NAOcontrol/HeadF.h"
#include "NAOcontrol/Head.h"
#include "NAOcontrol/ArmAngles.h"

using namespace std;
using namespace brown_ros;
using namespace NAOcontrol;

namespace brown_ros
{

class NaoJoyControlNode : public BrownJoyControlBase
{
    //Command codes for robot's actions. There are different commands for different
    //sources - buttons and axes
    //Important note: command code is also an index of the corresponding string
    //in the m_arrConfigCommandStrings array
    enum CommandCodes 
    {
        ccConfig0 = 0,  //Enable configuration 0
        ccConfig1 = 1,
        
        ccWalkForward = 2,		//Button: walk forwards
        ccWalkBackward = 3,		//Button: walk backwards
        ccTurnLeft = 4,			//Button: turn right
        ccTurnRight = 5,		//Button: turn right
        ccAxisWalk = 6,			//Axis: walk forwards/backwards
        ccAxisTurn = 7,			//Axis: turn right/left
        
        ccHeadUp = 8,			//Button: head up
        ccHeadDown = 9,			//Button: head down
        ccHeadLeft = 10,		//Button: head left
        ccHeadRight = 11,		//Button: head right
        ccAxisHeadPitch = 12,		//Axis: control head pitch
        ccAxisHeadYaw = 13,		//Axis: control head yaw
        
        ccRShoulderRollRight = 14,	//Button: roll right shoulder rightwards
        ccRShoulderRollLeft = 15,	//Button: roll right shoulder leftwards
        ccRShoulderPitchUp = 16,	//Button: pitch right shoulder upwards
        ccRShoulderPitchDown = 17,	//Button: pitch right shoulder downwards
        ccRElbowRollRight = 18,		//Button: roll right elbow rightwards
        ccRElbowRollLeft = 19,		//Button: roll right elbow leftwards
        ccRElbowYawRight = 20,		//Button: yaw right elbow rightwards
        ccRElbowYawLeft = 21,		//Button: yaw right elbow leftwards
        ccAxisRShoulderRoll = 22,	//Axis: control right sholder roll 
        ccAxisRShoulderPitch = 23,	//Axis: control right sholder pitch
        ccAxisRElbowRoll = 24,		//Axis: control right elbow roll
        ccAxisRElbowYaw = 25,		//Axis: control right elbow yaw
        ccRWristLeft = 26,		//Button: turn right wrist leftwards
        ccRWristRight = 27,		//Button: turn right wrist rightwards
        ccRGraspCommand = 28,		//Button: open/close right hand

        ccLShoulderRollRight = 29,	//Button: roll left shoulder rightwards
        ccLShoulderRollLeft = 30,	//Button: roll left shoulder leftwards
        ccLShoulderPitchUp = 31,	//Button: pitch left shoulder upwards
        ccLShoulderPitchDown = 32,	//Button: pitch left shoulder downwards
        ccLElbowRollRight = 33,		//Button: roll left elbow rightwards
        ccLElbowRollLeft = 34,		//Button: roll left elbow leftwards
        ccLElbowYawRight = 35,		//Button: yaw left elbow rightwards
        ccLElbowYawLeft = 36,		//Button: yaw left elbow leftwards
        ccAxisLShoulderRoll = 37,	//Axis: control left sholder roll 
        ccAxisLShoulderPitch = 38,	//Axis: control left sholder pitch
        ccAxisLElbowRoll = 39,		//Axis: control left elbow roll
        ccAxisLElbowYaw = 40,		//Axis: control left elbow yaw
        ccLWristLeft = 41,		//Button: turn left wrist leftwards
        ccLWristRight = 42,		//Button: turn left wrist rightwards
        ccLGraspCommand = 43,		//Button: open/close left hand
	
        ccStop = 44,
        ccUnknown = -1
        
    } m_cmdCodeEnum;

    //Total number of control codes that can be sent to the robot from the
    //joystick
    static const int m_iNumControlCodes = 46;

    //Configuration strings that denote robot's actions. These strings will be used to map
    //robot's actions to joystick buttons and axes via ROS configuration parameters.
    //
    //IMPORTANT NOTE:
    // Strings are mapped to commands in the order of their command codes
    // E.g. 0-th string in the ConfigCommandStrings denotes command with command code 0,
    // 1st string - command with code 2 
    // and so on
    static const string m_arrConfigCommandStrings[m_iNumControlCodes]; 


    private:
    ///
	///Publishers for NAO control topics:
	///
    ///Walk - walking control
    ros::Publisher m_pubWalk;
    //HeadP - head movement control
    ros::Publisher m_pubHead;
    //RArmAngles topic - right arm control 
    ros::Publisher m_pubRArmAngles;
    //LArmAngles topic - left arm control 
    ros::Publisher m_pubLArmAngles;
        

	///
	///Values of commands to be sent to the robot
	///
	//Motion command
	int m_iMotionCommand;
	//Head command
	double m_dblHeadXCommand;
	double m_dblHeadYCommand;

	//Right hand commands
	double m_dblRShoulderPitch;
	double m_dblRShoulderRoll;
	double m_dblRElbowRoll;
	double m_dblRElbowYaw;
	double m_dblRWrist;
	double m_dblRGrasp;
	//Current state of the right grasper
	int m_iRGraspState;
	
	//Left hand commands
	double m_dblLShoulderPitch;
	double m_dblLShoulderRoll;
	double m_dblLElbowRoll;
	double m_dblLElbowYaw;
	double m_dblLWrist;
	double m_dblLGrasp;
	//Current state of the left grasper
	int m_iLGraspState;
	
    public:
    NaoJoyControlNode();
    ~NaoJoyControlNode();

    void Initialize();
        
    virtual int GetNumControlCodes();
    virtual string GetStringForControlCode(int iIdx);
    virtual void InitializeCommand();
    virtual void OnCommandReceived(int iCommandID, double dblCommandValue);
    virtual void SendCommandToRobot();
};

//Initializing configuration strings array. Cannot do this inside of class
const string NaoJoyControlNode::m_arrConfigCommandStrings[m_iNumControlCodes] = 
{
    "config0",  //Enable configuration 0
    "config1",  //Enable configuration 1
        
    "walkforward",  
    "walkbackward",
    "turnleft",
    "turnright",
    "axis_walk",
    "axis_turn",
        
    "headup",
    "headdown",
    "headleft",
    "headright",
    "axis_headpitch",
    "axis_headyaw",

    "rshoulderrollright",
    "rshoulderrollleft",
    "rshoulderpitchup",
    "rshoulderpitchdown",
    "relbowrollright",
    "relbowrollleft",
    "relbowyawright",
    "relbowyawleft",
    "axis_rshoulderroll",
    "axis_rshoulderpitch",
    "axis_relbowroll",
    "axis_relbowyaw",
    "rwristleft",
    "rwristright",

    "rgraspcommand",
    "lshoulderrollright",
    "lshoulderrollleft",
    "lshoulderpitchup",
    "lshoulderpitchdown",
    "lelbowrollright",
    "lelbowrollleft",
    "lelbowyawright",
    "lelbowyawleft",
    "axis_lshoulderroll",
    "axis_lshoulderpitch",
    "axis_lelbowroll",
    "axis_lelbowyaw",
    "lwristleft",
    "lwristright",
    "lgraspcommand",

    "stop",

    "unknown",
};
/*    
const string NaoJoyControlNode::m_strBaseConfigTemplateBtn = "/Brown/ALNao/Joy/Config%d/Btn%d";
const string NaoJoyControlNode::m_strBaseConfigTemplateAxis = "/Brown/ALNao/Joy/Config%d/Axis%d";
*/
}

#endif

