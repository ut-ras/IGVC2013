// Mostly based on jaus_challenge_2010.cpp in jaus++

// License associated with the file and jaus++
//
///
///  Redistribution and use in source and binary forms, with or without
///  modification, are permitted provided that the following conditions are met:
///      * Redistributions of source code must retain the above copyright
///        notice, this list of conditions and the following disclaimer.
///      * Redistributions in binary form must reproduce the above copyright
///        notice, this list of conditions and the following disclaimer in the
///        documentation and/or other materials provided with the distribution.
///      * Neither the name of the ACTIVE LAB, IST, UCF, nor the
///        names of its contributors may be used to endorse or promote products
///        derived from this software without specific prior written permission.
/// 
///  THIS SOFTWARE IS PROVIDED BY THE ACTIVE LAB''AS IS'' AND ANY
///  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
///  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
///  DISCLAIMED. IN NO EVENT SHALL UCF BE LIABLE FOR ANY
///  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
///  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
///  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
///  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
///  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
///  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///

#include "jaus/mobility/sensors/globalposesensor.h"
#include "jaus/mobility/sensors/localposesensor.h"
#include "jaus/mobility/sensors/velocitystatesensor.h"
#include "jaus/mobility/drivers/localwaypointlistdriver.h"
#include "jaus/core/component.h"

#include "ros/ros.h"
#include "jaus/Pose.h"
#include "jaus/Velocity.h"

#include <cstdio>

// Fill in subsystem ID at competition, node/component will remain 1
#define ADDRESS JAUS::Address(152, 1, 1)

#define JUDGE_ADDRESS JAUS::Address(42, 1, 1)
#define JUDGE_IP "192.168.1.42"
#define JUDGE_PORT 3794

JAUS::Component component;
JAUS::GlobalPoseSensor *globalPose;
JAUS::LocalPoseSensor *localPose;
JAUS::VelocityStateSensor *velocity;
JAUS::LocalWaypointListDriver *waypointDriver;

void jaus_print() {
    printf("\n======================================================\n");
    component.AccessControlService()->PrintStatus();    printf("\n");
    component.ManagementService()->PrintStatus();       printf("\n");
    globalPose->PrintStatus();                          printf("\n");
    localPose->PrintStatus();                           printf("\n");
    velocity->PrintStatus();                            printf("\n");
    waypointDriver->PrintStatus();
}


void jaus_init() {

    // Setting timeout for control to 0 (disables timeout of control).
    // This is done because the JVT and OCP do not query for the timeout period
    // and may not send messages to re-acquire/maintain control within the
    // normal 2 second timeout window.
    component.AccessControlService()->SetTimeoutPeriod(0);

    // Add the services we want/need for our component. By
    // default, the component object already has the Core Service set
    // (e.g. Discovery, Events, Access Control, Management).
    // The Discovery service automatically handles finding other JAUS
    // components, so you do not need to generate any Query Messages
    // (e.g. Query Identification) yourself, because Discovery does it
    // for you.

    // In this test program, we are making a simulated robot, which
    // requires the following mobility services.

    globalPose = new JAUS::GlobalPoseSensor();
    globalPose->SetSensorUpdateRate(25);
    component.AddService(globalPose);

    localPose = new JAUS::LocalPoseSensor();
    localPose->SetSensorUpdateRate(25);
    component.AddService(localPose);

    velocity = new JAUS::VelocityStateSensor();
    velocity->SetSensorUpdateRate(25);
    component.AddService(velocity);

    // Need a List Manager Service for Local Waypoint List Driver.
    component.AddService(new JAUS::ListManager());
    waypointDriver = new JAUS::LocalWaypointListDriver();
    component.AddService(waypointDriver);

    // Set Vehicle Identification Information.  Available options for the
    // parameters are JAUS::Subsystem::OCU and JAUS::Subsystem::Vehicle.  Since
    // we are not an OCU, we use Vehicle.  Finally, the string represents the
    // type of robot you have (e.g. Segway RMP, XUV).  This is different than the
    // name of your vehicle, but you can use that if you want.  Your subsystem
    // number is your unique identifier.
    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle, "Wheelchair");
   
    // Initialize component with a given ID.  Remember, you must
    // add all your services before you initialize the component, because you 
    // cannot add them after initialization.  All services will be deleted by
    // the component on program exit.
    if(!component.Initialize(ADDRESS)) {
        ROS_FATAL("Failed to Initialize JAUS++");
        ros::shutdown();
    }

    // Set state to Standby since we have initialized OK.
    component.ManagementService()->SetStatus(JAUS::Management::Status::Standby);

    // Now that we are initialized, lets create a fixed connection to the
    // JVT for testing.  Since the JVT and the OCP of the JAUS 
    // Interoperability Challenge do not support multicast for Discovery, you
    // must make a direct connection to them.
    JAUS::Transport* transportService = component.TransportService();

    JAUS::Connection::Info connectionInfo;
    connectionInfo.mDestIP = JUDGE_IP;
    connectionInfo.mDestPortNumber = JUDGE_PORT;
    connectionInfo.mTransportType = JAUS::Connection::Transport::JUDP;

    transportService->AddConnection(JUDGE_ADDRESS, connectionInfo);
}

void localPoseCB(const jaus::Pose::ConstPtr &msg) {
    JAUS::LocalPose lPose;

    lPose.SetX(msg->xpos);
    lPose.SetY(msg->ypos);
    lPose.SetZ(msg->zpos);
    //lPose.SetPositionRMS(0.0); //TODO
    lPose.SetRoll(msg->roll); //int radians
    lPose.SetPitch(msg->pitch);
    lPose.SetYaw(msg->yaw);
    //lPose.SetAttitudeRMS(0.0); //TODO
    lPose.SetTimeStamp(JAUS::Time(true));
    // Save the data to the service.
    localPose->SetLocalPose(lPose);
}

void globalPoseCB(const jaus::Pose::ConstPtr &msg) {
    JAUS::GlobalPose gPose;

    gPose.SetLatitude(msg->xpos);
    gPose.SetLongitude(msg->ypos);
    gPose.SetAltitude(msg->zpos);
    //gPose.SetPositionRMS(0.0); //TODO
    gPose.SetRoll(msg->roll); //int radians
    gPose.SetPitch(msg->pitch);
    gPose.SetYaw(msg->yaw);
    //gPose.SetAttitudeRMS(0.0); //TODO
    gPose.SetTimeStamp(JAUS::Time(true));
    // Save the data to the service.
    globalPose->SetGlobalPose(gPose);

    //Uncomment this if you want the local to update from the global
    //localPose->SetLocalPose(gPose);
}

void velocityCB(const jaus::Velocity::ConstPtr &msg) {
    // Set an initial velocity state.
    JAUS::VelocityState vState;

    vState.SetVelocityX(msg->xrate);
    vState.SetVelocityY(msg->yrate);
    vState.SetVelocityZ(msg->zrate);
    vState.SetRollRate(msg->rollrate);
    vState.SetPitchRate(msg->pitchrate);
    vState.SetYawRate(msg->yawrate);
    //vState.SetVelocityRMS(0.0);//TODO
    vState.SetTimeStamp(JAUS::Time(true));
    // Save the data to the service.
    velocity->SetVelocityState(vState);
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "jaus");
    ros::NodeHandle node;

    ros::Rate loop_rate(1);

    jaus_init();

    // listen to the topics for the appropriate updates
    ros::Subscriber lpsub = node.subscribe("jaus/local_pose", 10, localPoseCB);
    ros::Subscriber gpsub = node.subscribe("jaus/global_pose", 10, globalPoseCB);
    ros::Subscriber vsub =  node.subscribe("jaus/velocity", 10, velocityCB);

    while(ros::ok() && component.ManagementService()->GetStatus() != 
                       JAUS::Management::Status::Shutdown           ) {
        jaus_print();

        ros::spinOnce();
        loop_rate.sleep();
    }


    // Don't delete services, they are deleted by the Component class.
    // Shutdown any components associated with our subsystem.
    component.Shutdown();

    return 0;
}
