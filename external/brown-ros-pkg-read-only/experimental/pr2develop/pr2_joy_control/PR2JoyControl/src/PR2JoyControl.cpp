#include "PR2JoyControl.h"

namespace brown_ros
{

//Node construstor
PR2JoyControlNode::PR2JoyControlNode() : BrownJoyControlBase()
{
    //Assume that graspers are closed
    m_iRGraspState = 1;
    m_iLGraspState = 1;

    m_dblHeadAngleInc = 0.075;
    m_dblCurrentHeadYaw = 0.0;
    m_dblCurrentHeadPitch = 0.0;
    m_dblPrevHeadYaw = 0.0;
    m_dblPrevHeadPitch = 0.0;
}

//Delete the node object
//Clean up all the arrays allocated to command mapping
PR2JoyControlNode::~PR2JoyControlNode()
{
    delete m_pPointHeadClient;
}

///
/// Get the number of control codes for the robot commands
///
int PR2JoyControlNode::GetNumControlCodes()
{
    return m_iNumControlCodes;
}

///
/// Get the string that encodes action code number iIdx for the robot
///
string PR2JoyControlNode::GetStringForControlCode(int iIdx)
{
    if (iIdx < m_iNumControlCodes)
    {
        return m_arrConfigCommandStrings[iIdx];
    }
    return "";
}

///
/// Initialize the command to "no action"
///
void PR2JoyControlNode::InitializeCommand()
{
    m_dbl_vx = 0.0;
    m_dbl_vy = 0.0;
    m_dbl_vz = 0.0;

    m_dblHeadXCommand = 0.0;
    m_dblHeadYCommand = 0.0;

    m_dblRUpperArmRoll = 0.0;
    m_dblRShoulderPitch = 0.0;
    m_dblRShoulderRoll = 0.0;
    m_dblRForearmRoll = 0.0;
    m_dblRElbowFlex = 0.0;
    m_dblRWristPitch = 0.0;
    m_dblRWristRoll = 0.0;

    m_dblLUpperArmRoll = 0.0;
    m_dblLShoulderPitch = 0.0;
    m_dblLShoulderRoll = 0.0;
    m_dblLForearmRoll = 0.0;
    m_dblLElbowFlex = 0.0;
    m_dblLWristPitch = 0.0;
    m_dblLWristRoll = 0.0;
    
    m_iHeadPitchState = 0;
    m_iHeadYawState = 0;
}

//Add a specified action to the current command
//commandCode is an actual command to perform, dblCommandValue - value associated 
//with command, e.g. speed of the action - currently not used
//Note: command might contain a number of actions, they will be added
//by one at a time
void PR2JoyControlNode::OnCommandReceived(int iCommandID, double dblCommandValue)
{
    CommandCodes commandCode = static_cast<CommandCodes>(iCommandID);
    switch(commandCode)
    {
	case ccConfig0:
	    m_iActiveConfig = 0;
	    logger.Logl(LVL3, "Setting current configuration to 0");
	    break;
	case ccConfig1:
	    m_iActiveConfig = 1;
	    logger.Logl(LVL3, "Setting current configuration to 1");
	    break;
	case ccMoveBackward:
	    m_dbl_vx = -1*m_dblLinearSpeed;
	    break;
	case ccMoveForward:
	    m_dbl_vx = m_dblLinearSpeed;
	    break;
	case ccTurnLeft:
	    m_dbl_vz = m_dblLinearSpeed;
	    break;
	case ccTurnRight:
	    m_dbl_vz = -1*m_dblLinearSpeed;
	    break;
	
	case ccAxisMove:
	    if (dblCommandValue >= m_dblAxesSensitivity)
	    {
		    m_dbl_vx = m_dblLinearSpeed;
	    }else if (dblCommandValue <= m_dblAxesSensitivityNeg)
	    {
		    m_dbl_vx = -1*m_dblLinearSpeed;
        }
	    break;
	case ccAxisTurn:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		    m_dbl_vz = m_dblAngularSpeed;
	    }else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		    m_dbl_vz = -1*m_dblAngularSpeed;
	    }
	    break;

	case ccAxisHeadPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	        m_iHeadPitchState = -1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
	        m_iHeadPitchState = 1;
	    }

	    break;
	    
	case ccAxisHeadYaw:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	        m_iHeadYawState = 1;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
	        m_iHeadYawState = -1;
	    }
	    break;


	case ccAxisRUpperArmRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
    		m_dblRUpperArmRoll = m_dblUpperArmEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblRUpperArmRoll = -1*m_dblUpperArmEffort;
	    }
	    break;
	case ccAxisRShoulderRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
    		m_dblRShoulderRoll = m_dblShoulderEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblRShoulderRoll = -1*m_dblShoulderEffort;
	    }
	    break;
	case ccAxisRShoulderPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
    		m_dblRShoulderPitch = -1*m_dblShoulderEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblRShoulderPitch = m_dblShoulderEffort;
	    }
	    break;
	case ccAxisRForearmRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblRForearmRoll = m_dblForearmEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblRForearmRoll = -1*m_dblForearmEffort;
	    }
	    break;
	case ccAxisRElbowFlex:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	    	m_dblRElbowFlex = -1*m_dblElbowEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblRElbowFlex = m_dblElbowEffort;
	    }
	    break;
	case ccAxisRWristRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	    	m_dblRWristRoll = -1*m_dblWristEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblRWristRoll = m_dblWristEffort;
	    }
	    break;
	case ccAxisRWristPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	    	m_dblRWristPitch = -1*m_dblWristEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblRWristPitch = m_dblWristEffort;
	    }
	    break;
	



	case ccAxisLUpperArmRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
    		m_dblLUpperArmRoll = m_dblUpperArmEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblLUpperArmRoll = -1*m_dblUpperArmEffort;
	    }
	    break;
	case ccAxisLShoulderRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
    		m_dblLShoulderRoll = m_dblShoulderEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblLShoulderRoll = -1*m_dblShoulderEffort;
	    }
	    break;
	case ccAxisLShoulderPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
    		m_dblLShoulderPitch = -1*m_dblShoulderEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblLShoulderPitch = m_dblShoulderEffort;
	    }
	    break;
	case ccAxisLForearmRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblLForearmRoll = -1*m_dblForearmEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblLForearmRoll = m_dblForearmEffort;
	    }
	    break;
	case ccAxisLElbowFlex:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
		m_dblLElbowFlex = -1*m_dblElbowEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
		m_dblLElbowFlex = m_dblElbowEffort;
	    }
	    break;
	case ccAxisLWristRoll:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	    	m_dblLWristRoll = -1*m_dblWristEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblLWristRoll = m_dblWristEffort;
	    }
	    break;
	case ccAxisLWristPitch:
	    if (dblCommandValue > m_dblAxesSensitivity)
	    {
	    	m_dblLWristPitch = -1*m_dblWristEffort;
	    }
	    else if (dblCommandValue < m_dblAxesSensitivityNeg)
	    {
    		m_dblLWristPitch = m_dblWristEffort;
	    }
	    break;


	case ccHeadUp:
	    m_iHeadPitchState = 1;
	    break;
	case ccHeadDown:
   	    m_iHeadPitchState = -1;
	    break;
	case ccHeadRight:
        m_iHeadYawState = 1;
	    break;
	case ccHeadLeft:
        m_iHeadYawState = -1;
	    break;

	case ccRUpperArmRollRight:
	    m_dblRUpperArmRoll = m_dblUpperArmEffort;
	    break;
	case ccRUpperArmRollLeft:
	    m_dblRUpperArmRoll = -1*m_dblUpperArmEffort;
	    break;
	case ccRShoulderRollRight:
	    m_dblRShoulderRoll = m_dblShoulderEffort;
	    break;
	case ccRShoulderRollLeft:
	    m_dblRShoulderRoll = -1*m_dblShoulderEffort;
	    break;
	case ccRShoulderPitchUp:
	    m_dblRShoulderPitch = -1*m_dblShoulderEffort;
	    break;
	case ccRShoulderPitchDown:
	    m_dblRShoulderPitch = m_dblShoulderEffort;
	    break;
	case ccRForearmRollRight:
	    m_dblRForearmRoll = m_dblForearmEffort;
	    break;
	case ccRForearmRollLeft:
	    m_dblRForearmRoll = -1*m_dblForearmEffort;
	    break;
	case ccRElbowFlexRight:
	    m_dblRElbowFlex = m_dblElbowEffort;
	    break;
	case ccRElbowFlexLeft:
	    m_dblRElbowFlex = -1*m_dblElbowEffort;
	    break;
	case ccRWristPitchUp:
	    m_dblRWristPitch = m_dblWristEffort;
	    break;
	case ccRWristPitchDown:
	    m_dblRWristPitch = -1*m_dblWristEffort;
	    break;
	case ccRWristRollLeft:
	    m_dblRWristRoll = -1*m_dblWristEffort;
	    break;
	case ccRWristRollRight:
	    m_dblRWristRoll = m_dblWristEffort;
	    break;

	    
	case ccLUpperArmRollRight:
	    m_dblLUpperArmRoll = m_dblUpperArmEffort;
	    break;
	case ccLUpperArmRollLeft:
	    m_dblLUpperArmRoll = -1*m_dblUpperArmEffort;
	    break;
	case ccLShoulderRollRight:
	    m_dblLShoulderRoll = m_dblShoulderEffort;
	    break;
	case ccLShoulderRollLeft:
	    m_dblLShoulderRoll = -1*m_dblShoulderEffort;
	    break;
	case ccLShoulderPitchUp:
	    m_dblLShoulderPitch = -1*m_dblShoulderEffort;
	    break;
	case ccLShoulderPitchDown:
	    m_dblLShoulderPitch = m_dblShoulderEffort;
	    break;
	case ccLForearmRollRight:
	    m_dblLForearmRoll = -1*m_dblForearmEffort;
	    break;
	case ccLForearmRollLeft:
	    m_dblLForearmRoll = m_dblForearmEffort;
	    break;
	case ccLElbowFlexRight:
	    m_dblLElbowFlex = m_dblElbowEffort;
	    break;
	case ccLElbowFlexLeft:
	    m_dblLElbowFlex = -1*m_dblElbowEffort;
	    break;
	case ccLWristPitchUp:
	    m_dblLWristPitch = m_dblWristEffort;
	    break;
	case ccLWristPitchDown:
	    m_dblLWristPitch = -1*m_dblWristEffort;
	    break;
	case ccLWristRollLeft:
	    m_dblLWristRoll = m_dblWristEffort;
	    break;
	case ccLWristRollRight:
	    m_dblLWristRoll = -1*m_dblWristEffort;
	    break;

	case ccRGraspCommand:
        logger.Logl(LVL3, "Right grasp commanded");
	    m_iRGraspState = -1*m_iRGraspState;
	    break;
	case ccLGraspCommand:
        logger.Logl(LVL3, "Left grasp commanded");
	    m_iLGraspState = -1*m_iLGraspState;
	    break;

	default:
	    logger.Logl(LVL2, "Unknown command %d, value %f", commandCode, dblCommandValue);
	    break;
    }
}

///
/// initialize all the command messages and send them to the robot
///
void PR2JoyControlNode::SendCommandToRobot()
{

    //Publish movement command
    geometry_msgs::Twist cmd;
    cmd.linear.x = m_dbl_vx;
    cmd.linear.y = m_dbl_vy;
    cmd.angular.z = m_dbl_vz;
    m_pubBase.publish(cmd);
    
/*
    //Publish head command
    PR2JointController::HeadVelocities headMsg;
    headMsg.head_pan_joint_effort = m_dblHeadXCommand;
    headMsg.head_tilt_joint_effort = m_dblHeadYCommand;
    m_pubHead.publish(headMsg);
*/

    //Publish right arm command
    PR2JointController::ArmVelocities armMsg;
    armMsg.upper_arm_roll_joint_effort = m_dblRUpperArmRoll;
    armMsg.shoulder_pan_joint_effort = m_dblRShoulderRoll;
    armMsg.shoulder_lift_joint_effort = m_dblRShoulderPitch;
    armMsg.forearm_roll_joint_effort = m_dblRForearmRoll;
    armMsg.elbow_flex_joint_effort = m_dblRElbowFlex;
    armMsg.wrist_flex_joint_effort = m_dblRWristPitch;
    armMsg.wrist_roll_joint_effort = m_dblRWristRoll;
    m_pubRArmEffort.publish(armMsg);

    armMsg.upper_arm_roll_joint_effort = m_dblLUpperArmRoll;
    armMsg.shoulder_pan_joint_effort = m_dblLShoulderRoll;
    armMsg.shoulder_lift_joint_effort = m_dblLShoulderPitch;
    armMsg.forearm_roll_joint_effort = m_dblLForearmRoll;
    armMsg.elbow_flex_joint_effort = m_dblLElbowFlex;
    armMsg.wrist_flex_joint_effort = m_dblLWristPitch;
    armMsg.wrist_roll_joint_effort = m_dblLWristRoll;
    m_pubLArmEffort.publish(armMsg);
    
    pr2_controllers_msgs::Pr2GripperCommand gripperCmd;
    //Left gripper control
    if (m_iLGraspState == 1)
    {   //Close
        gripperCmd.position = m_dblGripperClosePosition;
        gripperCmd.max_effort = m_dblGripperMaxEffort;
    }else if(m_iLGraspState == -1)
    {   //Open
        gripperCmd.position = m_dblGripperOpenPosition;
        gripperCmd.max_effort = m_dblGripperMaxEffort;
    }else
    {   //Unknown command
        logger.Logl(WARNING, "Unknown left gripper status %d", m_iLGraspState);
    }
    m_pubLGripper.publish(gripperCmd);

    //Right gripper control
    if (m_iRGraspState == 1)
    {   //Close
        gripperCmd.position = m_dblGripperClosePosition;
        gripperCmd.max_effort = m_dblGripperMaxEffort;
    }else if(m_iRGraspState == -1)
    {   //Open
        gripperCmd.position = m_dblGripperOpenPosition;
        gripperCmd.max_effort = m_dblGripperMaxEffort;
    }else
    {   //Unknown command
        logger.Logl(WARNING, "Unknown right gripper status %d", m_iRGraspState);
    }
    m_pubRGripper.publish(gripperCmd);

    //Calculating head pitch/yaw angles
    m_dblCurrentHeadPitch += m_iHeadPitchState*m_dblHeadAngleInc;
	if (m_dblCurrentHeadPitch < cdblMinHeadPitch)
	    m_dblCurrentHeadPitch = cdblMinHeadPitch;
	if (m_dblCurrentHeadPitch > cdblMaxHeadPitch)
	    m_dblCurrentHeadPitch = cdblMaxHeadPitch;
    m_dblCurrentHeadYaw += m_iHeadYawState*m_dblHeadAngleInc;
	if (m_dblCurrentHeadYaw < cdblMinHeadYaw)
	    m_dblCurrentHeadYaw = cdblMinHeadYaw;
	if (m_dblCurrentHeadYaw > cdblMaxHeadYaw)
	    m_dblCurrentHeadYaw = cdblMaxHeadYaw;
    //If these angles changed - send a command to the robot
    if ((m_dblPrevHeadYaw != m_dblCurrentHeadYaw)||
        (m_dblPrevHeadPitch != m_dblCurrentHeadPitch))
    {
        SetHeadCoordinates(m_dblCurrentHeadPitch, m_dblCurrentHeadYaw);
        m_dblPrevHeadPitch = m_dblCurrentHeadPitch;
        m_dblPrevHeadYaw = m_dblCurrentHeadYaw;
    }
}

///
/// Read parameters specific to this node
///
void PR2JoyControlNode::ReadParameters()
{
    m_node.param("gripper_open_position", m_dblGripperOpenPosition, 0.08);
    m_node.param("gripper_close_position", m_dblGripperClosePosition, -100.0);
    m_node.param("gripper_max_effort", m_dblGripperMaxEffort, -1.0);
    m_node.param("linear_speed", m_dblLinearSpeed, 0.3);
    m_node.param("angular_speed", m_dblAngularSpeed, 0.7);
    
    m_node.param("upper_arm_effort", m_dblUpperArmEffort, 0.5);
    m_node.param("shoulder_effort", m_dblShoulderEffort, 2.0);
    m_node.param("elbow_effort", m_dblElbowEffort, 0.6);
    m_node.param("forearm_effort", m_dblForearmEffort, 0.6);
    m_node.param("wrist_effort", m_dblWristEffort, 0.3);
    
}

///
/// Sead robot's head angular coordinates - pitch and yaw
/// Action server does not allow to control pitch and yaw directly, so we have
/// to translate them to cartesian coordinates of the point the robot should be
/// looking at
///
void PR2JoyControlNode::SetHeadCoordinates(double dblPitch, double dblYaw)
{
    double dblX, dblY, dblZ;
    //We assume that the robot will be always looking at the point 5 meters away 
    //from the robot
    double dblR = 5.0;
    //Calculate height of the point (Z coordinate)
    dblZ = tan(dblPitch)*dblR+cdblCameraHeight;
    //Calculate X and Y coordinates of the point
    dblY = dblR * sin(dblYaw);
    dblX = dblR * cos(dblYaw);

    //Send this point to the robot
    pr2_controllers_msgs::PointHeadGoal goal;
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_link";
    point.point.x = dblX;
    point.point.y = dblY;
    point.point.z = dblZ;
    goal.target = point;
    goal.pointing_frame = "high_def_frame";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 1;
    goal.pointing_axis.z = 1;
    goal.min_duration = ros::Duration(0.2);
    goal.max_velocity = 5.0;
    m_pPointHeadClient->sendGoal(goal);
}



//Initialize connections to all ROS topics and/or services we are using
void PR2JoyControlNode::Initialize()
{
    //Read mapping of keys/axes to robot actions
    BrownJoyControlBase::Initialize();

    //Advertize ourselves as publishers to PR2 topics
    //Motion control
    m_pubBase = m_node.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
/*
    //Head control
    m_pubHead = m_node.advertise<PR2JointController::HeadVelocities>("/brown/pr2/head_joint_angles", 1);
*/
    //Right arm control
    m_pubRArmEffort = m_node.advertise<PR2JointController::ArmVelocities>("/brown/pr2/rarm_joint_angles", 1);
    //Left arm control
    m_pubLArmEffort = m_node.advertise<PR2JointController::ArmVelocities>("/brown/pr2/larm_joint_angles", 1);

    //Right gripper control
    m_pubRGripper = m_node.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/r_gripper_controller/command", 1);
    //Left gripper control
    m_pubLGripper = m_node.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", 1);

    //Read parameters specific to this node
    ReadParameters();

    //Connect to the head action client
    m_pPointHeadClient = new PointHeadClient("/head_traj_controller/point_head_action", true);
    while (!m_pPointHeadClient->waitForServer(ros::Duration(5.0)))
    {
        logger.Logl(LVL2, "Waiting to connect to the head action server");
    }
    
    //Set robot's head to look forwards
    SetHeadCoordinates(m_dblCurrentHeadPitch, m_dblCurrentHeadYaw);
}

}

//Program entry point
int main(int argc, char** argv)
{
    //Initialize the node
    ros::init(argc,argv,"PR2JoyControl");
    PR2JoyControlNode nodeObj;
    
    //Initialize connections to other nodes through topics/services
    nodeObj.Initialize();
    
    //Read parameters, e.g. node parameters and action mappings
    nodeObj.Run();

    return(0);
}

