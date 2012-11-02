#ifndef _PR2JOY_CONTROL_
#define _PR2JOY_CONTROL_

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#include <string>

#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

#include <PR2JointController/ArmVelocities.h>

using namespace std;
//using namespace brown_ros;
using namespace PR2JointController;

namespace brown_ros
{

class PR2JointController: public pr2_controller_interface::Controller
{
	private:
	ros::NodeHandle m_node;
	
	ros::Subscriber m_subRArmJointVel;
	ros::Subscriber m_subLArmJointVel;
	ros::Subscriber m_subHeadJointVel;
	
	//Right arm joints
	pr2_mechanism_model::JointState* m_pRUpperArmRollJointState;
	pr2_mechanism_model::JointState* m_pRShoulderPanJointState;
	pr2_mechanism_model::JointState* m_pRShoulderLiftJointState;
	pr2_mechanism_model::JointState* m_pRForearmRollJointState;
	pr2_mechanism_model::JointState* m_pRElbowFlexJointState;
	pr2_mechanism_model::JointState* m_pRWristFlexJointState;
	pr2_mechanism_model::JointState* m_pRWristRollJointState;
    //Left arm joints
	pr2_mechanism_model::JointState* m_pLUpperArmRollJointState;
	pr2_mechanism_model::JointState* m_pLShoulderPanJointState;
	pr2_mechanism_model::JointState* m_pLShoulderLiftJointState;
	pr2_mechanism_model::JointState* m_pLForearmRollJointState;
	pr2_mechanism_model::JointState* m_pLElbowFlexJointState;
	pr2_mechanism_model::JointState* m_pLWristFlexJointState;
	pr2_mechanism_model::JointState* m_pLWristRollJointState;

    double m_dblRUpperArmRollJointEffort;
    double m_dblRShoulderPanJointEffort;
    double m_dblRShoulderLiftJointEffort;
    double m_dblRForearmRollJointEffort;
    double m_dblRElbowFlexJointEffort;
    double m_dblRWristFlexJointEffort;
    double m_dblRWristRollJointEffort;

    double m_dblLUpperArmRollJointEffort;
    double m_dblLShoulderPanJointEffort;
    double m_dblLShoulderLiftJointEffort;
    double m_dblLForearmRollJointEffort;
    double m_dblLElbowFlexJointEffort;
    double m_dblLWristFlexJointEffort;
    double m_dblLWristRollJointEffort;

    double m_dblHeadPanJointEffort;
    double m_dblHeadTiltJointEffort;

    public:
    PR2JointController();
    ~PR2JointController();

	//Override Controller members
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
    void starting();
    void update();
    void stopping();

    bool InitializeConnections();
    bool ConnectArmJoints(pr2_mechanism_model::RobotState *robot);

    void RightArmVelocitiesCallback(const ArmVelocitiesConstPtr& armsVelMsg);
    void LeftArmVelocitiesCallback(const ArmVelocitiesConstPtr& armsVelMsg);
};

}

#endif

