#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

double* getIK(int argc, double *argv){
  double* toReturn;
  toReturn = new double[7];

  char** args;
  sprintf(args[0], "%d", argc);
  for (int i=0; i<argc; i++)
    sprintf(args[i+1], "%f", argv[i]);
  ros::init (argc, args, "get_ik");
  ros::NodeHandle rh;

  ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik");

  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>("pr2_right_arm_kinematics/get_ik");

  printf("Converting from: posx %f posy %f posz %f orientx %f orienty %f orientz %f orientw %f\n", argv[1], argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  // define the service messages
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

  gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = argv[1];
  gpik_req.ik_request.pose_stamped.pose.position.y = argv[2];
  gpik_req.ik_request.pose_stamped.pose.position.z = argv[3];

  gpik_req.ik_request.pose_stamped.pose.orientation.x = argv[4];
  gpik_req.ik_request.pose_stamped.pose.orientation.y = argv[5];
  gpik_req.ik_request.pose_stamped.pose.orientation.z = argv[6];
  gpik_req.ik_request.pose_stamped.pose.orientation.w = argv[7];

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++){
        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
	toReturn[i] = gpik_res.solution.joint_state.position[i];
      }
    else
      ROS_ERROR("Inverse kinematics failed");
      return NULL;
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");
    return NULL;
  ros::shutdown();

  return toReturn;
}

