#include "NaoJoyControl.h"

namespace brown_ros
{

//Node construstor
NaoJoyControlNode::NaoJoyControlNode() : BrownJoyControlBase()
{
    //Assume that graspers are closed
    m_iRGraspState = -1;
    m_iLGraspState = -1;
}

//Delete the node object
//Clean up all the arrays allocated to command mapping
NaoJoyControlNode::~NaoJoyControlNode()
{
}

int NaoJoyControlNode::GetNumControlCodes()
{
    return m_iNumControlCodes;
}

string NaoJoyControlNode::GetStringForControlCode(int iIdx)
{
    if (iIdx < m_iNumControlCodes)
    {
        return m_arrConfigCommandStrings[iIdx];
    }
    return "";
}

//Initialize the command to "no action"
void NaoJoyControlNode::InitializeCommand()
{
    m_iMotionCommand = 0;

    m_dblHeadXCommand = 0.0;
    m_dblHeadYCommand = 0.0;

    m_dblRShoulderPitch = 0.0;
    m_dblRShoulderRoll = 0.0;
    m_dblRElbowRoll = 0.0;
    m_dblRElbowYaw = 0.0;
    m_dblRWrist = 0.0;
    m_dblRGrasp = 0.0;

    m_dblLShoulderPitch = 0.0;
    m_dblLShoulderRoll = 0.0;
    m_dblLElbowRoll = 0.0;
    m_dblLElbowYaw = 0.0;
    m_dblLWrist = 0.0;
    m_dblLGrasp = 0.0;
}

//Add a specified action to the current command
//commandCode is an actual command to perform, dblCommandValue - value associated 
//with command, e.g. speed of the action - currently not used
//Note: command might contain a number of actions, they will be added
//by one at a time
void NaoJoyControlNode::OnCommandReceived(int iCommandID, double dblCommandValue)
{
    CommandCodes commandCode = static_cast<CommandCodes>(iCommandID);
    switch(commandCode)
    {
	case ccConfig0:
	    m_iActiveConfig = 0;
	    logger.Logl(LVL2, "Setting current configuration to 0");
	    break;
	case ccConfig1:
	    m_iActiveConfig = 1;
	    logger.Logl(LVL2, "Setting current configuration to 1");
	    break;
	case ccWalkForward:
	    m_iMotionCommand |= 1;
	    logger.Logl(LVL2, "Walking forward");
	    break;
	case ccTurnLeft:
	    m_iMotionCommand |= 2;
	    logger.Logl(LVL2, "Turning left");
	    break;
	case ccTurnRight:
	    m_iMotionCommand |= 4;
	    logger.Logl(LVL2, "Turning right");
	    break;
	
	case ccAxisWalk:
	    if (dblCommandValue >= m_dblAxesSensitivity)
	    {
		m_iMotionCommand |= 1;
		logger.Logl(LVL2, "Walking forward");
	    }
	    break;
	case ccAxisTurn:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_iMotionCommand |= 2;
		logger.Logl(LVL2, "Turning left");
	    }else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_iMotionCommand |= 4;
		logger.Logl(LVL2, "Turning right");
	    }
	    break;

	case ccAxisHeadPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblHeadYCommand = -0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblHeadYCommand = 0.1;
	    }
	    break;
	    
	case ccAxisHeadYaw:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblHeadXCommand = 0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblHeadXCommand = -0.1;
	    }
	    break;

	case ccAxisRShoulderRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblRShoulderRoll = 0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblRShoulderRoll = -0.1;
	    }
	    break;
	    
	case ccAxisLShoulderRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblLShoulderRoll = 0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblLShoulderRoll = -0.1;
	    }
	    break;
	    
	case ccAxisRShoulderPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblRShoulderPitch = -0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblRShoulderPitch = 0.1;
	    }
	    break;

	case ccAxisLShoulderPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblLShoulderPitch = -0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblLShoulderPitch = 0.1;
	    }
	    break;

	case ccAxisRElbowRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblRElbowRoll = 0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblRElbowRoll = -0.1;
	    }
	    break;

	case ccAxisLElbowRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblLElbowRoll = -0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblLElbowRoll = 0.1;
	    }
	    break;

	case ccAxisRElbowYaw:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblRElbowYaw = -0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblRElbowYaw = 0.1;
	    }
	    break;

	case ccAxisLElbowYaw:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblLElbowYaw = -0.1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblLElbowYaw = 0.1;
	    }
	    break;

	case ccHeadUp:
	    m_dblHeadYCommand = 0.1;
	    break;
	case ccHeadDown:
	    m_dblHeadYCommand = -0.1;
	    break;
	case ccHeadRight:
	    m_dblHeadXCommand = 0.1;
	    break;
	case ccHeadLeft:
	    m_dblHeadXCommand = -0.1;
	    break;

	case ccRShoulderRollRight:
	    m_dblRShoulderRoll = 0.1;
	    break;
	case ccRShoulderRollLeft:
	    m_dblRShoulderRoll = -0.1;
	    break;
	case ccRShoulderPitchUp:
	    m_dblRShoulderPitch = -0.1;
	    break;
	case ccRShoulderPitchDown:
	    m_dblRShoulderPitch = 0.1;
	    break;
	    
	case ccLShoulderRollRight:
	    m_dblLShoulderRoll = 0.1;
	    break;
	case ccLShoulderRollLeft:
	    m_dblLShoulderRoll = -0.1;
	    break;
	case ccLShoulderPitchUp:
	    m_dblLShoulderPitch = -0.1;
	    break;
	case ccLShoulderPitchDown:
	    m_dblLShoulderPitch = 0.1;
	    break;

	case ccRElbowRollRight:
	    m_dblRElbowRoll = 0.1;
	    break;
	case ccRElbowRollLeft:
	    m_dblRElbowRoll = -0.1;
	    break;
	case ccRElbowYawRight:
	    m_dblRElbowYaw = -0.1;
	    break;
	case ccRElbowYawLeft:
	    m_dblRElbowYaw = 0.1;
	    break;

	case ccLElbowRollRight:
	    m_dblLElbowRoll = -0.1;
	    break;
	case ccLElbowRollLeft:
	    m_dblLElbowRoll = 0.1;
	    break;
	case ccLElbowYawRight:
	    m_dblLElbowYaw = 0.1;
	    break;
	case ccLElbowYawLeft:
	    m_dblLElbowYaw = -0.1;
	    break;

	case ccRWristLeft:
	    m_dblRWrist = -0.1;
	    break;
	case ccRWristRight:
	    m_dblRWrist = 0.1;
	    break;
	case ccLWristLeft:
	    m_dblLWrist = 0.1;
	    break;
	case ccLWristRight:
	    m_dblLWrist = -0.1;
	    break;

	case ccRGraspCommand:
	    m_iRGraspState = -1*m_iRGraspState;
	    m_dblRGrasp = static_cast<double>(m_iRGraspState);
	    break;
	case ccLGraspCommand:
	    m_iLGraspState = -1*m_iLGraspState;
	    m_dblLGrasp = static_cast<double>(m_iLGraspState);
	    break;

	default:
	    logger.Logl(LVL2, "Sending the robot command %d, value %f", commandCode, dblCommandValue);
	    break;
    }
}

//initialize all the command messages and send them to the robot
void NaoJoyControlNode::SendCommandToRobot()
{
    //Publish walk command
    Walk walkMsg;
    walkMsg.walk = m_iMotionCommand;
    m_pubWalk.publish(walkMsg);

    //Publish head command
    HeadF headMsg;
    headMsg.x = m_dblHeadXCommand;
    headMsg.y = m_dblHeadYCommand;
    m_pubHead.publish(headMsg);

    //Publish right arm command
    ArmAngles armMsg;
    armMsg.armAngles[0] = m_dblRShoulderPitch;
    armMsg.armAngles[1] = m_dblRShoulderRoll;
    armMsg.armAngles[2] = m_dblRElbowRoll;
    armMsg.armAngles[3] = m_dblRElbowYaw;
    armMsg.armAngles[4] = m_dblRWrist;
    armMsg.armAngles[5] = m_dblRGrasp;
    m_pubRArmAngles.publish(armMsg);

    //Publish left arm command
    armMsg.armAngles[0] = m_dblLShoulderPitch;
    armMsg.armAngles[1] = m_dblLShoulderRoll;
    armMsg.armAngles[2] = m_dblLElbowRoll;
    armMsg.armAngles[3] = m_dblLElbowYaw;
    armMsg.armAngles[4] = m_dblLWrist;
    armMsg.armAngles[5] = m_dblLGrasp;
    m_pubLArmAngles.publish(armMsg);
}

//Initialize connections to all ROS topics and/or services we are using
void NaoJoyControlNode::Initialize()
{
    BrownJoyControlBase::Initialize();

    //Advertize ourselves as publishers to NAOcontrol topics
    //Motion control
    m_pubWalk = m_node.advertise<Walk>("/motion", 1);
    //Head control
    m_pubHead = m_node.advertise<HeadF>("/headF", 1);
    //Right arm control
    m_pubRArmAngles = m_node.advertise<ArmAngles>("/RArmAngles", 1);
    //Left arm control
    m_pubLArmAngles = m_node.advertise<ArmAngles>("/LArmAngles", 1);
}

}

//Program entry point
int main(int argc, char** argv)
{
    //Initialize the node
    ros::init(argc,argv,"NaoJoyControl");
    NaoJoyControlNode nodeObj;
    
    //Initialize connections to other nodes through topics/services
    nodeObj.Initialize();
    
    //Read parameters, e.g. node parameters and action mappings
    nodeObj.Run();

    return(0);
}

