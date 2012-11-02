#include "bumper_localizer.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "bumper_localizer");
	 ros::NodeHandle n1, n2, n3, n4, n5;
	 nhw = new NodeHandleWrapper();	 
     vel_pub = n1.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
     pos_sub = n2.subscribe("/position", 1, positionCallback);
     bump_sub = n4.subscribe("/sensorPacket", 1, bumperCallback);
     lmap_sub = n5.subscribe("/line_map", 1, lineMapCallback);

	 //don't drive unless you are told so
     nhw->local_node_handle->param("step_navigator/ok_to_drive", ok_to_drive, 1);

	 ros::spin();
}

void lineMapCallback(const map_loader::LineMapConstPtr& msg)
{
	 map4 = (*msg).lines; 	
}

void positionCallback(const position_tracker::PositionConstPtr& msg)
{
    cur_pos = *msg;
} 

void bumperCallback(const irobot_create_2_1::SensorPacketConstPtr& msg)
{
    cur_sensors = *msg;

    nhw->local_node_handle->getParam("step_navigator/ok_to_drive", ok_to_drive);

    //we have recovered from bumping         
    if(!(cur_sensors.bumpLeft || cur_sensors.bumpRight))
    {		
        if(!ok_to_drive)
        {
            ros::NodeHandle nn;
            ros::ServiceClient client = nn.serviceClient<position_tracker::SetPosition>("set_position");
            position_tracker::SetPosition srv;            
            srv.request.x = latestBumpPos.x;
            srv.request.y = latestBumpPos.y;
            srv.request.theta = latestBumpPos.theta; //cur_pos.theta + 0.5; //pos.theta + 0.15; //0.15: concluded from experiments

            if(!client.call(srv))
            {
                ROS_ERROR("Failed to call service setPosition");
            }

            //ok_to_drive = 1;
		    nhw->local_node_handle->setParam("step_navigator/ok_to_drive", 1);

            prevBump = 0;

			//only useful when node running alone
		   /*geometry_msgs::Twist twist;
		   twist.angular.z = 0.0;
		   twist.linear.x = 0.0;
		   vel_pub.publish(twist);*/

        }
        
        return;
    }


   //ok_to_drive = 0;
   nhw->local_node_handle->setParam("step_navigator/ok_to_drive", 0);
   geometry_msgs::Twist twist;
   twist.angular.z = 0.0;
   twist.linear.x = 0.0;

   //we've hit something hard or head-on.  Back away slowly.
   if(cur_sensors.bumpLeft && cur_sensors.bumpRight)
   {
      twist.linear.x = -0.1; //-0.1;
      prevBump = 1;
   }
   else if(cur_sensors.bumpLeft)
   {   
      twist.angular.z = -0.1; //-0.1;
      if(!prevBump)
      {
            latestBumpPos = getClosestLeftLineProjection();
            //pospub.publish(pos);
            cout << "BUMPED LEFT:" << endl;
            cout << "latestBumpPos.x = " << latestBumpPos.x << endl;
            cout << "latestBumpPos.y = " << latestBumpPos.y << endl;
            cout << "latestBumpPos.theta = " << latestBumpPos.theta << endl;
      }
      prevBump = 1;
   }
   else if(cur_sensors.bumpRight)
   {
      twist.angular.z = 0.1;
      if(!prevBump)
      {
            latestBumpPos = getClosestRightLineProjection();
            //pospub.publish(pos);
            cout << "BUMPED RIGHT:" << endl;
            cout << "latestBumpPos.x = " << latestBumpPos.x << endl;
            cout << "latestBumpPos.y = " << latestBumpPos.y << endl;
            cout << "latestBumpPos.theta = " << latestBumpPos.theta << endl;

      }
      prevBump = 1;
   }

   cout << "Publish twist (" << twist.linear.x << ", " << twist.angular.z << ") from bumper_localizer" << endl;
   vel_pub.publish(twist);
}

position_tracker::Position getClosestLeftLineProjection()
{

   //WHAT IF THERE IS NO LINE TO THE LEFT OF THE ROBOT???

   double min_dist = 100000000;
   position_tracker::Position *tmp_pos = new position_tracker::Position();
   tmp_pos->x = -1;
   tmp_pos->y = -1;
   tmp_pos->theta = -1;

   vector<map_loader::Line >::iterator i;   
   for(i = map4.begin(); i != map4.end(); ++i) //for each edge in the map
   {
      map_loader::Line e = *i;
      double D[2][2] = {{(e.Bx - e.Ax), (e.By - e.Ay)}, {(cur_pos.x - e.Ax), (cur_pos.y - e.Ay)}};

      /*cout << "D[0][0] = " << D[0][0] << endl;
      cout << "D[0][1] = " << D[0][1] << endl;
      cout << "D[1][0] = " << D[1][0] << endl;
      cout << "D[1][1] = " << D[1][1] << endl;*/

      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e.Ax = " << e.Ax << endl;
      cout << "e.Ay = " << e.Ay << endl;
      cout << "e.Bx = " << e.Bx << endl;
      cout << "e.By = " << e.By << endl;
      cout << "e.theta = " << e.theta << endl;
      cout << "det(D) = " << det(D) << endl;


      //keep only the lines to the left of the current robot pos
      if((det(D) < 0 && fabs(det(D)) > 0.0001 && (cur_pos.theta > e.theta - PI/2 && cur_pos.theta < e.theta + PI/2)) || (det(D) > 0  && fabs(det(D)) > 0.0001 && (cur_pos.theta < e.theta - PI/2 || cur_pos.theta > e.theta + PI/2)))
      {
          //find the projection of the current point to the last identified left line
          double A[2][2] = {{(e.Bx - e.Ax), (e.By - e.Ay)}, {(e.Ay - e.By), (e.Bx - e.Ax)}};
          double X[2] = {0, 0};
          double B[2] = {cur_pos.x*(e.Bx - e.Ax) + cur_pos.y*(e.By - e.Ay), e.Ay*(e.Bx - e.Ax) - e.Ax*(e.By - e.Ay)};
          solve(A, B, X);

            
          //discard line segment if the projection falls outside its boundaries
          if(e.theta < 0.1 ) //horizontal line
          {
              if(X[0] > e.Bx || X[0] < e.Ax)
                 continue;
          }
		  else{ //vertical line
              if(X[1] > e.By || X[1] < e.Ay)
                 continue; 

          }
          

      cout << "line to the LEFT of the robot" << endl;
/*      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e.Ax = " << e.Ax << endl;
      cout << "e.Ay = " << e.Ay << endl;
      cout << "e.Bx = " << e.Bx << endl;
      cout << "e.By = " << e.By << endl;
      cout << "e.theta = " << e.theta << endl;*/

          //calculate the distance between the projection and the current robot location
          double sdist = sqrt(pow(cur_pos.x - X[0], 2) + pow(cur_pos.y - X[1], 2));

          //keep the projection with the min distance from the current pos
          if(sdist < min_dist)
          {
              min_dist = sdist;
              tmp_pos->x = X[0] + (0.05/sdist)*(cur_pos.x - X[0]); //X[0]; //15 cm away from the wall may seem hard, but it handles the doors in the building
              tmp_pos->y = X[1] + (0.05/sdist)*(cur_pos.y - X[1]); //X[1];


			  double angle1 = e.theta;
			  double angle2 = e.theta + PI;
              if(angle1 > PI)
                   angle1 = -2*PI + angle1;
              else if(angle1 < -PI)
                   angle1 = 2*PI + angle1;

              if(angle2 > PI)
                   angle2 = -2*PI + angle2;
              else if(angle2 < -PI)
                   angle2 = 2*PI + angle2;

              double diff1 = angle1 - cur_pos.theta;
              double diff2 = angle2 - cur_pos.theta;

              if(fabs(diff1) < fabs(diff2))
                   tmp_pos->theta = angle1; 
              else
                   tmp_pos->theta = angle2; 

      cout << "angle1 = " << angle1 << endl;
      cout << "angle2 = " << angle2 << endl;
      cout << "tmp_pos->theta = " << tmp_pos->theta << endl;


      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e.Ax = " << e.Ax << endl;
      cout << "e.Ay = " << e.Ay << endl;
      cout << "e.Bx = " << e.Bx << endl;
      cout << "e.By = " << e.By << endl;
      cout << "e.theta = " << e.theta << endl;
      cout << "det(D) = " << det(D) << endl;

          }
      }
   }
  
   return *tmp_pos;

}

position_tracker::Position getClosestRightLineProjection()
{

   //WHAT IF THERE IS NO LINE TO THE RIGHT OF THE ROBOT???

   double min_dist = 100000000;
   position_tracker::Position *tmp_pos = new position_tracker::Position();
   tmp_pos->x = -1;
   tmp_pos->y = -1;
   tmp_pos->theta = -1;

   vector<map_loader::Line>::iterator i;   
   for(i = map4.begin(); i != map4.end(); ++i) //for each edge in the map
   {
      map_loader::Line e = *i;
      double D[2][2] = {{(e.Bx - e.Ax), (e.By - e.Ay)}, {(cur_pos.x - e.Ax), (cur_pos.y - e.Ay)}};

/*      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e.Ax = " << e.Ax << endl;
      cout << "e.Ay = " << e.Ay << endl;
      cout << "e.Bx = " << e.Bx << endl;
      cout << "e.By = " << e.By << endl;
      cout << "e.theta = " << e.theta << endl;
      cout << "det(D) = " << det(D) << endl;*/

      //keep only the lines to the right of the current robot pos
      if((det(D) > 0  && fabs(det(D)) > 0.0001 && (cur_pos.theta > e.theta - PI/2 && cur_pos.theta < e.theta + PI/2)) || (det(D) < 0  && fabs(det(D)) > 0.0001 && (cur_pos.theta < e.theta - PI/2 || cur_pos.theta > e.theta + PI/2)))
      {
          //find the projection of the current point to the last identified left line
          double A[2][2] = {{(e.Bx - e.Ax), (e.By - e.Ay)}, {(e.Ay - e.By), (e.Bx - e.Ax)}};
          double X[2] = {0, 0};
          double B[2] = {cur_pos.x*(e.Bx - e.Ax) + cur_pos.y*(e.By - e.Ay), e.Ay*(e.Bx - e.Ax) - e.Ax*(e.By - e.Ay)};
          solve(A, B, X);

          //discard line segment if the projection falls outside its boundaries
          if(e.theta < 0.1 ) //horizontal line
          {
              if(X[0] > e.Bx || X[0] < e.Ax)
                 continue;
          }
		  else{ //vertical line
              if(X[1] > e.By || X[1] < e.Ay)
                 continue; 
          }

          cout << "line to the RIGHT of the robot" << endl;

          //calculate the distance between the projection and the current robot location
          double sdist = pow(cur_pos.x - X[0], 2) + pow(cur_pos.y - X[1], 2);

          //keep the projection with the min distance from the current pos
          if(sdist < min_dist)
          {
              min_dist = sdist;
              tmp_pos->x = X[0] + (0.05/sdist)*(cur_pos.x - X[0]); //X[0];
              tmp_pos->y = X[1] + (0.05/sdist)*(cur_pos.y - X[1]); //X[1];


			  double angle1 = e.theta;
			  double angle2 = e.theta + PI;
              if(angle1 > PI)
                   angle1 = -2*PI + angle1;
              else if(angle1 < -PI)
                   angle1 = 2*PI + angle1;

              if(angle2 > PI)
                   angle2 = -2*PI + angle2;
              else if(angle2 < -PI)
                   angle2 = 2*PI + angle2;

              double diff1 = angle1 - cur_pos.theta;
              double diff2 = angle2 - cur_pos.theta;

              if(fabs(diff1) < fabs(diff2))
                   tmp_pos->theta = angle1; 
              else
                   tmp_pos->theta = angle2; 

      cout << "angle1 = " << angle1 << endl;
      cout << "angle2 = " << angle2 << endl;
      cout << "tmp_pos->theta = " << tmp_pos->theta << endl;

      cout << "cur_pos.x = " << cur_pos.x << endl;
      cout << "cur_pos.y = " << cur_pos.y << endl;
      cout << "cur_pos.theta = " << cur_pos.theta << endl;
      cout << "e.Ax = " << e.Ax << endl;
      cout << "e.Ay = " << e.Ay << endl;
      cout << "e.Bx = " << e.Bx << endl;
      cout << "e.By = " << e.By << endl;
      cout << "e.theta = " << e.theta << endl;
      cout << "det(D) = " << det(D) << endl;

          }
      }
   }

   return *tmp_pos;

}

// det = ad - bc
double det(double A[2][2])
{
   return(A[0][0]*A[1][1] - A[0][1]*A[1][0]);
}

// D = 1/det(A)
//
// .............. | d -b | . | D*d -D*b |
// inv = D * |-c a | = |-D*c D*a |
void inv(double A[2][2], double IA[2][2])
{
   double D = 1/det(A);
   IA[0][0] = +D*A[1][1];
   IA[0][1] = -D*A[0][1];
   IA[1][0] = -D*A[1][0];
   IA[1][1] = +D*A[0][0];
}

// |a b| |x| . |e|
// |c d| |y| = |f|
//
// |x| . |a b|-1 |e|
// |y| = |c d| .. |f|
void solve(double A[2][2], double C[2], double S[2])
{
 double IA[2][2];
 inv(A, IA);
 S[0] = IA[0][0]*C[0] + IA[0][1]*C[1];
 S[1] = IA[1][0]*C[0] + IA[1][1]*C[1];
}

void substring(const char* text, int start, int stop, char *new_string)
{
    sprintf(new_string, "%.*s", stop - start, &text[start]);
}

string char_to_string(char *input_p)
{
    string str(input_p);
    return str;
}

void error(char *msg)
{
    perror(msg);
    exit(1);
}

