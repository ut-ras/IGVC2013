#include "ardrone_pf.h"

void ardrone_particle_filter::perform_motion_update(ardrone_state& drone_state)
{
  //For each particle in vec_pf perform a motion update
  //Compute the prior probability
  vector<ardrone_particle>::iterator it=vec_pf->begin();
  
  double yaw_delta;
  yaw_delta = getQuaternionDiffShortestRad(drone_state.q_yaw, drone_state.q_yaw_prev);
  ///cout << "yaw_delta: " << (yaw_delta/PI*180.0) << "\n";

  //standard deviation for theta_z(yaw) odometry reading  
  double thz_sd = fabs(yaw_delta*odom_rpy_error[2]);
  
  //standard deviation for lin_vel_x odometry reading
  double vx_sd = fabs(drone_state.b_lin_vel[0]*odom_linvel_error[0]);

  //standard deviation for lin_vel_y odometry reading
  double vy_sd = fabs(drone_state.b_lin_vel[1]*odom_linvel_error[1]);

  double gaussian_error_thz=0.0;
  double delta_thz_sample=0.0;
  double thz_sample=0.0;
  double cthz=0.0,sthz=0.0;
  double gaussian_error_vx=0.0;
  double vx_sample=0.0;
  double w_vx_sample=0.0;
  double gaussian_error_vy=0.0;
  double vy_sample=0.0;
  double w_vy_sample=0.0;
  double delta_x_sample=0.0;
  double delta_y_sample=0.0;
  double x_sample=0.0;
  double y_sample=0.0;

  bool first_iter=true;

  /*
  btVector3 avg_pos(0,0,0);
  btVector3 avg_rpy(0,0,0);
  */

  for(;it<vec_pf->end();it++)
    {
      //compute thetaz sample
      gaussian_error_thz = pf_ran_gaussian(thz_sd);

      delta_thz_sample = (yaw_delta - gaussian_error_thz);

      thz_sample = ((*it).drone_rpy[2] + delta_thz_sample);

      //convert thz_sample to [0,2*PI]
      if(thz_sample<0)
	{
	  thz_sample = (thz_sample+(2*PI));
	}
      else if(thz_sample>(2*PI))
	{
	  thz_sample = (thz_sample-(2*PI));
	}

      if(first_iter)
	{
	  first_iter=false;
	  yaw_min=thz_sample;
	  yaw_max=thz_sample;
	}
      else
	{
	  if(thz_sample<yaw_min) yaw_min=thz_sample;
	  if(thz_sample>yaw_max) yaw_max=thz_sample;
	}

      cthz = cos(thz_sample);
      sthz = sin(thz_sample);

      //compute lin_vel_x sample
      gaussian_error_vx = pf_ran_gaussian(vx_sd);
      vx_sample = (drone_state.b_lin_vel[0] - gaussian_error_vx);

      //compute lin_vel_y sample
      gaussian_error_vy = pf_ran_gaussian(vy_sd);
      vy_sample = (drone_state.b_lin_vel[1] - gaussian_error_vy);

      //compute linear velocities in the world frame (simple yaw rotation)
      w_vx_sample = (cthz*vx_sample - sthz*vy_sample);
      w_vy_sample = (sthz*vx_sample + cthz*vy_sample);

      delta_x_sample = (drone_state.dt*((w_vx_sample + (*it).w_linvel[0])/2.0));
      delta_y_sample = (drone_state.dt*((w_vy_sample + (*it).w_linvel[1])/2.0));      
      
      x_sample = ((*it).w_pos[0] + delta_x_sample);
      y_sample = ((*it).w_pos[1] + delta_y_sample);

      (*it).w_linvel[0] = w_vx_sample;
      (*it).w_linvel[1] = w_vy_sample;
      (*it).w_pos[0] = x_sample;
      (*it).w_pos[1] = y_sample;
      (*it).drone_rpy[2] = thz_sample;
      (*it).seqcnt = drone_state.seqcnt;

      /*
      avg_pos[0]+=x_sample;
      avg_pos[1]+=y_sample;
      avg_rpy[2]+=thz_sample;
      */
    }

  /*
  avg_pos[0]=(avg_pos[0]/np);
  avg_pos[1]=(avg_pos[1]/np);
  avg_rpy[2]=(avg_rpy[2]/np);

  cout << "\nardrone_pf: \n";
  cout << "avg_pos[0]: " << avg_pos[0] << "\n";
  cout << "avg_pos[1]: " << avg_pos[1] << "\n";
  //convert orientation to degrees
  double avg_rpy_deg = (avg_rpy[2]/PI*180);
  cout << "avg_rpy[2]: " << avg_rpy[2] << "\n";
  cout << "avg_rpy_deg[2]: " << avg_rpy_deg << "\n";
  */
}

double ardrone_particle_filter::custom_compute_gaussian_density(double rv, double sd)
{
  double pdfcoeff = (1.0/(sqrt(2.0*PI*sd*sd)));
  double expval = exp((-0.5*rv*rv)/(sd*sd));
  double probval = (pdfcoeff*expval);
  return probval;
}

void ardrone_particle_filter::perform_measurement_update(const ar_recog::Tags::ConstPtr& tagsmsg, vector<tagPose> &tagposes, double avg_posx, double avg_posy, const ros::Publisher &artag_visualization)
{

  ////cout << "Inside measurement update function.\n";
  
  //we perform a measurement update using the biggest tag in the field of view
  if(tagsmsg->tags.size()>0)
    {
      int biggestTag=0;
      double biggestDiameter=0.0;
      for(unsigned int i=0; i<tagsmsg->tags.size();i++)
	{
	  if(tagsmsg->tags[i].diameter>biggestDiameter)
	    {
	      biggestDiameter=tagsmsg->tags[i].diameter;
	      biggestTag=i;
	    }
	}
      
      if(biggestDiameter>0)
	{
	  //bottom camera width and height
	  unsigned int imwidth = 176;
	  unsigned int imheight = 144;

	  //pose of identified tag with respect to map
	  tagPose *tempTagPose=NULL;

	  //compute meters/pixel coefficient
	  double tc[8]={0.0};
	  for(int i=0; i<8; i++)
	    tc[i]=tagsmsg->tags[biggestTag].cwCorners[i];
	  double edgeL=0.0;
	  edgeL = sqrt(pow((tc[2]-tc[0]),2)+pow((tc[3]-tc[1]),2));
	  double mcoeff = (0.16/edgeL);

	  //get tag orientation offset from drone frame (its in radians)
	  double tthz = tagsmsg->tags[biggestTag].zRot;

	  vector<tagPose>::iterator tagposeit=tagposes.begin();
	  double dmin=0.0;
	  bool firstmatch=true;
	  for(;tagposeit<tagposes.end();tagposeit++)
	    if((*tagposeit).id==tagsmsg->tags[biggestTag].id)
	      {
		if(firstmatch)
		  {
		    firstmatch=false;
		    tempTagPose = &(*tagposeit);
		    dmin = sqrt(pow(((tempTagPose->x)-avg_posx),2)+pow(((tempTagPose->y)-avg_posy),2));
		  }
		else
		  {
		    double tagdist = sqrt(pow((((*tagposeit).x)-avg_posx),2)+pow((((*tagposeit).y)-avg_posy),2));
		    if(tagdist<dmin)
		      {
			tempTagPose = &(*tagposeit);
		      }
		  }
	      }

	  if(tempTagPose!=NULL)
	    {
	      /*
	      cout << "tthz: " << (tthz/PI*180.0) << "\n";
	      cout << "trot: " << (tempTagPose->trot/PI*180.0) << "\n";
	      */

	      //publish tag pose to rviz
	      btVector3 temprotaxis(0,0,1);
	      visualization_msgs::Marker tagmarker;
	      tagmarker.header.frame_id = "/map";
	      tagmarker.header.stamp = ros::Time();
	      tagmarker.ns = "tag_marker";
	      tagmarker.id = 0;
	      tagmarker.type = visualization_msgs::Marker::ARROW;
	      tagmarker.action = visualization_msgs::Marker::ADD;
	      tagmarker.pose.position.x = tempTagPose->x;
	      tagmarker.pose.position.y = tempTagPose->y;
	      tagmarker.pose.position.z = 1;
	      btQuaternion qrottemp(temprotaxis,tempTagPose->trot);
	      tagmarker.pose.orientation.x = qrottemp.x();
	      tagmarker.pose.orientation.y = qrottemp.y();
	      tagmarker.pose.orientation.z = qrottemp.z();
	      tagmarker.pose.orientation.w = qrottemp.w();
	      tagmarker.scale.x = 2.0;
	      tagmarker.scale.y = 2.0;
	      tagmarker.scale.z = 2.0;
	      tagmarker.color.a = 0.5;
	      tagmarker.color.r = 1.0;
	      tagmarker.color.g = 1.0;
	      tagmarker.color.b = 0.0;

	      artag_visualization.publish(tagmarker);

	      double dronetrueyaw = (tthz+tempTagPose->trot);

	      //change dronetrueyaw to {[0,+PI] U [0,-PI]}
	      if(dronetrueyaw>PI)
		{
		  dronetrueyaw = (dronetrueyaw-2*PI);
		}
	      else if(dronetrueyaw<-PI)
		{
		  dronetrueyaw = (2*PI+dronetrueyaw);
		}

	      //compute drone true pose
	      double xcam = ((tagsmsg->tags[biggestTag].x-(imwidth/2.0))*mcoeff);
	      double ycam = ((tagsmsg->tags[biggestTag].y-(imheight/2.0))*mcoeff);
	      double xoff = ycam;
	      double yoff = xcam;
	      double ctthz = cos(dronetrueyaw);
	      double stthz = sin(dronetrueyaw);
	      /*
		double drone_xoff = ((xoff*ctthz)-(yoff*stthz));
		double drone_yoff = ((xoff*stthz)+(yoff*ctthz));
	      */
	      double drone_xoff = ((xoff*ctthz)-(yoff*stthz));
	      double drone_yoff = ((xoff*stthz)+(yoff*ctthz));
	      
	      double dronetruex = (tempTagPose->x + drone_xoff);
	      double dronetruey = (tempTagPose->y + drone_yoff);
	      
	      cout << "dronetruex: " << dronetruex << "\n";
	      cout << "dronetruey: " << dronetruey << "\n";

	      /*	      
	      double dronetrueyaw_deg = (dronetrueyaw/PI*180);
	      cout << "dronetrueyaw(before): " << dronetrueyaw << "\n";
	      cout << "dronetrueyaw_deg(before): " << dronetrueyaw_deg << "\n\n";
	      */

	      //convert dronetrueyaw from [-PI,PI] to [0,2*PI]
	      if(dronetrueyaw<0)
		{
		  dronetrueyaw = (dronetrueyaw+(2*PI));
		}
	      
	      double dronetrueyaw_deg = (dronetrueyaw/PI*180);
	      cout << "dronetrueyaw: " << dronetrueyaw << "\n";
	      cout << "dronetrueyaw_deg: " << dronetrueyaw_deg << "\n\n";
	      
	      //iterate through all the particles and recompute weight
	      vector<ardrone_particle>::iterator it=vec_pf->begin();
	      double total_wt=0.0;
	      for(;it<vec_pf->end();it++)
		{
		  //double custom_compute_gaussian_density(double rv, double sd);
		  double rv1 = ((*it).w_pos[0]-dronetruex);
		  double w1 = custom_compute_gaussian_density(rv1,measurement_model_pos_error[0]);
		  double rv2 = ((*it).w_pos[1]-dronetruey);
		  double w2 = custom_compute_gaussian_density(rv2,measurement_model_pos_error[1]);
		  //Need to account for the case when drone_rpy[2] and dronetrueyaw have -
		  //different signs
		  double yaw_error=((*it).drone_rpy[2]-dronetrueyaw);
		  
		  if(((*it).drone_rpy[2]*dronetrueyaw)<0)
		    {
		      if(yaw_error<0)
			yaw_error=(yaw_error+(2*PI));
		      else
			yaw_error=(yaw_error-(2*PI));
		    }
		  
		  double rv3=yaw_error;
		  double w3 = custom_compute_gaussian_density(rv3,measurement_model_rpy_error[2]);
		  (*it).wt = w1*w2*w3;
		  total_wt+=(*it).wt;
		}
	      
	      //Normalize weights
	      it=vec_pf->begin();
	      for(;it<vec_pf->end();it++)
		{
		  (*it).wt = ((*it).wt/total_wt);
		}
	      
	      //perform low variance resampling
	      double rnum = (drand48()/(np*1.0));
	      it=vec_pf->begin();
	      double cmlwt = (*it).wt;
	      double wtlimit=0.0;
	      int icnt=1;
	      vector<ardrone_particle> *vec_pf_temp = new vector<ardrone_particle>();
	      vec_pf_temp->reserve(np);
	      for(unsigned int m=1;m<=np;m++)
		{
		  wtlimit = rnum + (m-1)/(np*1.0);
		  while(wtlimit>cmlwt)
		    {
		      icnt=icnt+1;
		      it++;
		      cmlwt=cmlwt+(*it).wt;
		    }
		  vec_pf_temp->push_back((*it));
		}
	      
	      vec_pf->erase(vec_pf->begin(),vec_pf->end());
	      delete vec_pf;
	      vec_pf = vec_pf_temp;
	    }
	}
    }
}






