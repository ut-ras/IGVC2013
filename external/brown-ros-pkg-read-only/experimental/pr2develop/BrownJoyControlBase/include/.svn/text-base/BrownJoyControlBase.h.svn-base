#ifndef _BROWNJOY_CONTROL_BASE_
#define _BROWNJOY_CONTROL_BASE_

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#include <string>

#include <ros/ros.h>

#include "joy/Joy.h"

#include "logger.h"

using namespace std;
using namespace brown_ros;
using namespace joy;

namespace brown_ros
{

class BrownJoyControlBase
{
    ///
    ///Property declaration
    ///
    protected:
    //Templates for parameter names
    static const string m_strBaseConfigTemplateBtn;
    static const string m_strBaseConfigTemplateAxis;

    //Node handle
    ros::NodeHandle m_node;

    //Logger instance
    Logger logger;

    //ROS loop rate
    int m_iLoopRate;
        
    //Subscriber for joysctick messages
    ros::Subscriber m_subJoy;

    ///
    /// Joystick configuration
    ///
    //Number of configurations, axes and buttons in the joystick
    int m_iNumConfigurations;
    int m_iNumButtons;
    int m_iNumAxes;

    //Currently active controller configuration
    int m_iActiveConfig;

    //List of dynamic arrays that map joystick buttons to the command. 
    //
    //NOTE: the number of commands is bigger than the number of joystick buttons
    //So we introduce different "configurations" - a kind of modes that are used
    //to switch button-to-command mappings. E.g. configuration 0 can be used to
    //control robot's movement, while configuration 1 controls its arms
    //
    //The list contains mapping array for every separate configuration.
    //Every mapping is an array that contains command codes for each button.
    int** m_parrButtonToCmdMap;

    //List of dynamic arrays that map joystick buttons to the command. 
    //The list contains mapping array for every separate configuration.
    //Every mapping is an array that contains command codes for each axis.
    int** m_parrAxesToCmdMap;
    
    //Sensitivity of the robot's axes, in fact a threshold. A mapped action
    //will be taken if an absolute value of the command coming from the axis 
    //will be higher than the sensitivity parameter, 
    double m_dblAxesSensitivity;
    double m_dblAxesSensitivityNeg;
    
    ///
    ///Method declaration
    ///
    private:
    inline void tolower(string& strToConvert);
    
    void ReadJoystickParameters();
    void AllocateMappingArrays();
    void ReadParameters();

    void JoystickCallback(const JoyConstPtr& joyMsg);

    public:
    BrownJoyControlBase();
    ~BrownJoyControlBase();

    void Initialize();
    void Run();

    ///
    ///Virtual methods that have to be overridden by the user
    ///
    //Return the number of control codes for the robot
    virtual int GetNumControlCodes() = 0;

    //Get the string that defines the control code whose index is iIdx
    virtual string GetStringForControlCode(int iIdx) = 0;

    //Initialize the command to be sent to the robot (set it to "No Action")
    virtual void InitializeCommand() = 0;

    //The routine is called when the node receives the command with
    //command ID = iCommandID and command value = dblCommandValue
    virtual void OnCommandReceived(int iCommandID, double dblCommandValue)=0;

    //Called when all the joystick commands are processed (namely - when
    //OnCommandReceived is called for every command)
    virtual void SendCommandToRobot() = 0;
};

//Convert instance of std::string to lowercase
inline void BrownJoyControlBase::tolower(string& strToConvert)
{
    for (string::iterator it=strToConvert.begin(); it<strToConvert.end(); it++)
    {
        *it = std::tolower(*it);
    }
}

BrownJoyControlBase::BrownJoyControlBase(): m_node("~")
{
    //Set current joystick configuration to 0
    m_iActiveConfig = 0;
    //Set ROS loop rate to 100 Hz
    m_iLoopRate = 100;

    //Set default joystick configuration (XBOX joystick)
    m_iNumConfigurations = 2;
    m_iNumButtons = 10;
    m_iNumAxes = 6;
    m_dblAxesSensitivity = 0.8;
    m_dblAxesSensitivityNeg = -1*m_dblAxesSensitivity;
}

BrownJoyControlBase::~BrownJoyControlBase()
{
    for (int i=0; i<m_iNumConfigurations; i++)
    {
    if (m_parrButtonToCmdMap[i] != NULL)
        free(m_parrButtonToCmdMap[i]);
    if (m_parrAxesToCmdMap[i] != NULL)
        free(m_parrAxesToCmdMap[i]);
    }
    if (m_parrButtonToCmdMap != NULL)
    free(m_parrButtonToCmdMap);
    if (m_parrAxesToCmdMap != NULL)
    free(m_parrAxesToCmdMap);
}

//Read all the node parameters, including ROS node parameters as well
//as joystic-specific parameters (number of buttons/axes and mapping)
void BrownJoyControlBase::ReadParameters()
{
    logger.Logl(LVL2, "Reading node parameters and command mapping. Parameter values will be printed below");
    if (m_node.hasParam("LoopRate"))
    {
        m_node.getParam("LoopRate", m_iLoopRate);
        logger.Logl(LVL2, "LoopRate: %d", m_iLoopRate);
    }

    ReadJoystickParameters();
}

//Read joystick parameters from rosparam. This includes the number of buttons
//and axes for the joystick as well as mapping of robot's actions to buttons
//As a result, mapping arrays will be created and filled
void BrownJoyControlBase::ReadJoystickParameters()
{
    char szParamName[1024];
    string strParamValue;
    
    m_dblAxesSensitivity = 0.8;
    m_dblAxesSensitivityNeg = -1*m_dblAxesSensitivity;
    m_iNumConfigurations = 2;
    m_iNumButtons = 12;
    m_iNumAxes = 6;

    //Read axes sensitivity
    if (m_node.hasParam("AxesSensitivity"))
    {
        m_node.getParam("AxesSensitivity", m_dblAxesSensitivity);
        logger.Logl(LVL1, "AxesSensitivity=%d", m_dblAxesSensitivity);
        m_dblAxesSensitivityNeg = -1*m_dblAxesSensitivity;
    }

    //Read the number of joystic configurations/buttons/axes
    if (m_node.hasParam("NumConfigurations"))
    {
        m_node.getParam("NumConfigurations", m_iNumConfigurations);
        logger.Logl(LVL1, "NumConfigurations=%d", m_iNumConfigurations);
    }
    if (m_node.hasParam("NumButtons"))
    {
        m_node.getParam("NumButtons", m_iNumButtons);
        logger.Logl(LVL1, "NumButtons=%d", m_iNumButtons);
    }
    if (m_node.hasParam("NumAxes"))
    {
        m_node.getParam("NumAxes", m_iNumAxes);
        logger.Logl(LVL1, "NumAxes=%d", m_iNumAxes);
    }

    AllocateMappingArrays();

    logger.Logl(LVL1, "NumControlCodes=%d", GetNumControlCodes());
    for (int i=0; i<GetNumControlCodes(); i++)
    {
        logger.Logl(LVL1, "Control code =%d, string = %s", i, GetStringForControlCode(i).c_str());
    }
    
    //Fill in mapping arrays. Scan all the configurations
    for (int iConf=0; iConf<m_iNumConfigurations; iConf++)
    {
        //Scan all the axes for the given configuration
        for (int iAxis = 0; iAxis < m_iNumAxes; iAxis++)
        {
            sprintf(szParamName, m_strBaseConfigTemplateAxis.c_str(), iConf, iAxis);
            //logger.Logl(LVL1, "%s", szParamName);
            if (m_node.hasParam(szParamName))
            {
                m_node.getParam(szParamName, strParamValue);
                tolower(strParamValue);
                logger.Logl(LVL1, "%s=%s", szParamName, strParamValue.c_str());
                for (int i=0; i<GetNumControlCodes(); i++)
                {
                    if (!strParamValue.compare(GetStringForControlCode(i)))
                    {
                        m_parrAxesToCmdMap[iConf][iAxis] = i;
                        logger.Logl(LVL1, "Setting command code %d", i);
                    }
                }
            }
        }

        //Scan all the button for the given configuration
        for (int iButton = 0; iButton < m_iNumButtons; iButton++)
        {
            sprintf(szParamName, m_strBaseConfigTemplateBtn.c_str(), iConf, iButton);
            if (m_node.hasParam(szParamName))
            {
                m_node.getParam(szParamName, strParamValue);
                tolower(strParamValue);
                logger.Logl(LVL1, "%s=%s", szParamName, strParamValue.c_str());
                for (int i=0; i<GetNumControlCodes(); i++)
                {
                    if (!strParamValue.compare(GetStringForControlCode(i)))
                    {
                        m_parrButtonToCmdMap[iConf][iButton] = i;
                        logger.Logl(LVL1, "Setting command code %d", i);
                    }
                }
            }
        }
    }
}

//Allocates arrays for mapping buttons/axes to commands depending on configuration
void BrownJoyControlBase::AllocateMappingArrays()
{
    //Allocate mapping arrays
    logger.Logl(LVL1, "Allocating configurations array for button command map");

    m_parrButtonToCmdMap = (int**)malloc(m_iNumConfigurations*sizeof(int*));
    m_parrAxesToCmdMap = (int**)malloc(m_iNumConfigurations*sizeof(int*));
    for (int i=0; i<m_iNumConfigurations; i++)
    {    //Allocate mappings for every configuration
        m_parrButtonToCmdMap[i] = (int*)malloc((m_iNumButtons+1)*sizeof(int));
        memset(m_parrButtonToCmdMap[i], -1, (m_iNumButtons+1)*sizeof(int));

        m_parrAxesToCmdMap[i] = (int*)malloc((m_iNumAxes+1)*sizeof(int));
        memset(m_parrAxesToCmdMap[i], -1, (m_iNumAxes+1)*sizeof(int));
    }
    logger.Logl(LVL1, "Done allocating configurations array for button command map");
}

//Initialize the node. 
//First, connections to all ROS topics and/or services we are using
//Second, read all the node parameters
void BrownJoyControlBase::Initialize()
{
    ReadParameters();

    //Subscribe to joystick messages
    m_subJoy = m_node.subscribe("/joy", 100, &BrownJoyControlBase::JoystickCallback, this);

    //Initialize the first command will be sent to the robot
    //Set it to "No action"
    InitializeCommand();
}


//A callback routine that is called each time data is coming from joystick
void BrownJoyControlBase::JoystickCallback(const JoyConstPtr& joyMsg)
{
    unsigned int iButtonCt = m_iNumButtons;
    unsigned int iAxesCt = m_iNumAxes;
    
    //Initialize the command to "No action" - default state
    InitializeCommand();
    
    //Check if the real number of joystick buttons (one we've got from the message)
    //is equal to the number of buttons the user specified in the config
    if (joyMsg->buttons.size() != static_cast<unsigned int>(m_iNumButtons))
    {
            logger.Logl(WARNING, "Configured number of buttons %d is different than the number of buttons %u in the message",
                m_iNumButtons, joyMsg->buttons.size());
        if (joyMsg->buttons.size() < static_cast<unsigned int>(m_iNumButtons))
        {
            iButtonCt = joyMsg->buttons.size();
        }
    }
    
    //Check if the real number of joystick axes (one we've got from the message)
    //is equal to the number of buttons the user specified in the config
    if (joyMsg->axes.size() != static_cast<unsigned int>(m_iNumAxes))
    {
            logger.Logl(WARNING, "Configured number of axes %d is different than the number of buttons in the message",
                m_iNumAxes, joyMsg->axes.size());
        if (joyMsg->axes.size() < static_cast<unsigned int>(m_iNumAxes))
        {
            iAxesCt = joyMsg->axes.size();
        }
    }
    
    //Scan all the buttons pressed and add corresponding actions to the command
    for (unsigned int i=0; i<iButtonCt; i++)
    {
        int iBtnValue = joyMsg->buttons[i];
        if ((iBtnValue != 0)&&(m_parrButtonToCmdMap[m_iActiveConfig][i] != -1))
        {
            //If the command is defined for this button, then call the routine that will
            //pass this command to the robot. Since the command is going from the button,
            //the value of the command will be set to 1.0
            OnCommandReceived(m_parrButtonToCmdMap[m_iActiveConfig][i], 1.0);
        }
    }

    //Scan state of all the axes and add corresponding actions to the command
    for (unsigned int i=0; i<iAxesCt; i++)
    {
        double dblAxeValue = joyMsg->axes[i];
        if ((fabs(dblAxeValue) > m_dblAxesSensitivity)&&(m_parrAxesToCmdMap[m_iActiveConfig][i] != -1))
        {
            //If the command is defined for this axis, then call the routine that will
            //pass this command to the robot. Command value will be equal to the axis value
            OnCommandReceived(m_parrAxesToCmdMap[m_iActiveConfig][i], dblAxeValue);
        }
    }
}    

//Run the node in the loop
void BrownJoyControlBase::Run()
{
    ros::Rate loop_rate(m_iLoopRate);
    logger.Logl(LVL1, "Entering main loop. Loop rate %d", m_iLoopRate);
    while(ros::ok())
    {
        SendCommandToRobot();
        loop_rate.sleep();
        ros::spinOnce();
    }
    logger.Logl(LVL1, "Exiting main loop");
    
}

//Templates of parameter names. Every parameter name represent a button or axis name
const string BrownJoyControlBase::m_strBaseConfigTemplateBtn = "Config%d/Btn%d";
const string BrownJoyControlBase::m_strBaseConfigTemplateAxis = "Config%d/Axis%d";

}

#endif

