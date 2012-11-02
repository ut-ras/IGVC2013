#include "PR2JointController.h"

#include <pluginlib/class_list_macros.h>

namespace brown_ros
{

//Node construstor
PR2JointController::PR2JointController()
{
}

//Delete the node object
//Clean up all the arrays allocated to command mapping
PR2JointController::~PR2JointController()
{
}

/// Controller initialization in non-realtime
bool PR2JointController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &nh)
{
    m_node = nh;
    
    //Initialize connections to other nodes through topics/services
    if (!InitializeConnections())
    {
        ROS_ERROR("PR2JointController could not connect to topics");
        return false;
    }

    if (!ConnectArmJoints(robot))
    {
        ROS_ERROR("PR2JointController could not initialize joint states");
        return false;
    }

    m_dblRUpperArmRollJointEffort = 0;
    m_dblRShoulderPanJointEffort = 0;
    m_dblRShoulderLiftJointEffort = 0;
    m_dblRForearmRollJointEffort = 0;
    m_dblRElbowFlexJointEffort = 0;
    m_dblRWristFlexJointEffort = 0;
    m_dblRWristRollJointEffort = 0;
    
    m_dblLUpperArmRollJointEffort = 0;
    m_dblLShoulderPanJointEffort = 0;
    m_dblLShoulderLiftJointEffort = 0;
    m_dblLForearmRollJointEffort = 0;
    m_dblLElbowFlexJointEffort = 0;
    m_dblLWristFlexJointEffort = 0;
    m_dblLWristRollJointEffort = 0;
    
    m_dblHeadPanJointEffort = 0;
    m_dblHeadTiltJointEffort = 0;

    return true;
}

/// Controller startup in realtime
void PR2JointController::starting()
{
}

/// Controller update loop in realtime
void PR2JointController::update()
{
	m_pRUpperArmRollJointState->commanded_effort_ = m_dblRUpperArmRollJointEffort;
	m_pRShoulderPanJointState->commanded_effort_ = m_dblRShoulderPanJointEffort;
	m_pRShoulderLiftJointState->commanded_effort_ = m_dblRShoulderLiftJointEffort;
	m_pRForearmRollJointState->commanded_effort_ = m_dblRForearmRollJointEffort;
	m_pRElbowFlexJointState->commanded_effort_ = m_dblRElbowFlexJointEffort;
	m_pRWristFlexJointState->commanded_effort_ = m_dblRWristFlexJointEffort;
	m_pRWristRollJointState->commanded_effort_ = m_dblRWristRollJointEffort;

	m_pLUpperArmRollJointState->commanded_effort_ = m_dblLUpperArmRollJointEffort;
	m_pLShoulderPanJointState->commanded_effort_ = m_dblLShoulderPanJointEffort;
	m_pLShoulderLiftJointState->commanded_effort_ = m_dblLShoulderLiftJointEffort;
	m_pLForearmRollJointState->commanded_effort_ = m_dblLForearmRollJointEffort;
	m_pLElbowFlexJointState->commanded_effort_ = m_dblLElbowFlexJointEffort;
	m_pLWristFlexJointState->commanded_effort_ = m_dblLWristFlexJointEffort;
	m_pLWristRollJointState->commanded_effort_ = m_dblLWristRollJointEffort;
}


/// Controller stopping in realtime
void PR2JointController::stopping()
{
}

void PR2JointController::RightArmVelocitiesCallback(const ArmVelocitiesConstPtr& armsVelMsg)
{

    m_dblRUpperArmRollJointEffort = armsVelMsg->upper_arm_roll_joint_effort;
    m_dblRShoulderPanJointEffort = armsVelMsg->shoulder_pan_joint_effort;
    m_dblRShoulderLiftJointEffort = armsVelMsg->shoulder_lift_joint_effort;
    m_dblRForearmRollJointEffort = armsVelMsg->forearm_roll_joint_effort;
    m_dblRElbowFlexJointEffort = armsVelMsg->elbow_flex_joint_effort;
    m_dblRWristFlexJointEffort = armsVelMsg->wrist_flex_joint_effort;
    m_dblRWristRollJointEffort = armsVelMsg->wrist_roll_joint_effort;
}

void PR2JointController::LeftArmVelocitiesCallback(const ArmVelocitiesConstPtr& armsVelMsg)
{
    m_dblLUpperArmRollJointEffort = armsVelMsg->upper_arm_roll_joint_effort;
    m_dblLShoulderPanJointEffort = armsVelMsg->shoulder_pan_joint_effort;
    m_dblLShoulderLiftJointEffort = armsVelMsg->shoulder_lift_joint_effort;
    m_dblLForearmRollJointEffort = armsVelMsg->forearm_roll_joint_effort;
    m_dblLElbowFlexJointEffort = armsVelMsg->elbow_flex_joint_effort;
    m_dblLWristFlexJointEffort = armsVelMsg->wrist_flex_joint_effort;
    m_dblLWristRollJointEffort = armsVelMsg->wrist_roll_joint_effort;
}

//Initialize connections to all ROS topics and/or services we are using
bool PR2JointController::InitializeConnections()
{
    //Subscribe to joystick messages
    m_subRArmJointVel = m_node.subscribe("/brown/pr2/rarm_joint_angles", 100, &PR2JointController::RightArmVelocitiesCallback, this);
    m_subLArmJointVel = m_node.subscribe("/brown/pr2/larm_joint_angles", 100, &PR2JointController::LeftArmVelocitiesCallback, this);
    return true;
}

bool PR2JointController::ConnectArmJoints(pr2_mechanism_model::RobotState *robot)
{
    ///
    /// Initialize right arm joints
    ///
    m_pRUpperArmRollJointState = robot->getJointState("r_upper_arm_roll_joint");
    if (m_pRUpperArmRollJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_upper_arm_roll_joint'");
        return false;
    }

    m_pRShoulderPanJointState = robot->getJointState("r_shoulder_pan_joint");
    if (m_pRShoulderPanJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_shoulder_pan_joint'");
        return false;
    }
    
    m_pRShoulderLiftJointState = robot->getJointState("r_shoulder_lift_joint");
    if (m_pRShoulderLiftJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_shoulder_lift_joint'");
        return false;
    }

    m_pRForearmRollJointState = robot->getJointState("r_forearm_roll_joint");
    if (m_pRForearmRollJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_forearm_roll_joint'");
        return false;
    }

    m_pRElbowFlexJointState = robot->getJointState("r_elbow_flex_joint");
    if (m_pRElbowFlexJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_elbow_flex_joint'");
        return false;
    }

    m_pRWristFlexJointState = robot->getJointState("r_wrist_flex_joint");
    if (m_pRWristFlexJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_wrist_flex_joint'");
        return false;
    }

    m_pRWristRollJointState = robot->getJointState("r_wrist_roll_joint");
    if (m_pRWristRollJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'r_wrist_roll_joint'");
        return false;
    }

    ///
    /// Initialize left arm joints
    ///
    m_pLUpperArmRollJointState = robot->getJointState("l_upper_arm_roll_joint");
    if (m_pLUpperArmRollJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_upper_arm_roll_joint'");
        return false;
    }

    m_pLShoulderPanJointState = robot->getJointState("l_shoulder_pan_joint");
    if (m_pLShoulderPanJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_shoulder_pan_joint'");
        return false;
    }
    
    m_pLShoulderLiftJointState = robot->getJointState("l_shoulder_lift_joint");
    if (m_pLShoulderLiftJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_shoulder_lift_joint'");
        return false;
    }

    m_pLForearmRollJointState = robot->getJointState("l_forearm_roll_joint");
    if (m_pLForearmRollJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_forearm_roll_joint'");
        return false;
    }

    m_pLElbowFlexJointState = robot->getJointState("l_elbow_flex_joint");
    if (m_pLElbowFlexJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_elbow_flex_joint'");
        return false;
    }

    m_pLWristFlexJointState = robot->getJointState("l_wrist_flex_joint");
    if (m_pLWristFlexJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_wrist_flex_joint'");
        return false;
    }

    m_pLWristRollJointState = robot->getJointState("l_wrist_roll_joint");
    if (m_pLWristRollJointState == NULL)
    {
        ROS_ERROR("PR2JointController could not find joint named 'l_wrist_roll_joint'");
        return false;
    }
    return true;
}

/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(PR2JointControllerPlugin, 
                         brown_ros::PR2JointController,
                         pr2_controller_interface::Controller)

}

