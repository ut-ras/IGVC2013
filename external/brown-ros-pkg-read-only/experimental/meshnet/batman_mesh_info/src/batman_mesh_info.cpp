#include "batman_mesh_info.h"

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "batman_mesh_info");
     ros::NodeHandle n1, n2;
     NN_pub = n1.advertise<batman_mesh_info::WifiNNs>("/wifiNNs", 1); 
    
     while(1)
	   {
		    get_NN_and_qual();
	   }
     
}

void get_NN_and_qual()
{
	 batman_mesh_info::WifiNNs NN;
	 FILE *ptr;
	 char buf[BUFSIZ];
	 char *command = "sudo batmand -b -c -d 1";
   vector<string> dstsList;
	 list<string> neigbList;     
	 vector<int> signalList;

	 if((ptr = popen(command, "r")) != NULL)
	 {
	    //read and ignore the first line
	    fgets(buf, BUFSIZ, ptr);

		  while(fgets(buf, BUFSIZ, ptr) != NULL)
		  {
			   string str, ip_str, qual_str;
	       string::size_type pos1, pos2;
			   int signal_strength = -1;

   	     str = char_to_string(buf);
         //get the IP of the destination node
         pos1 = 0;
         pos2 = str.find_first_of('(');
	       ip_str = str.substr(pos1, pos2 - pos1 - 5); //-5: found in practice (should probably be "tab" size or sth like that
			   dstsList.push_back(ip_str); 

			   //get the link quality with the destination node
			   pos1 = str.find_first_of('(');
			   pos2 = str.find_first_of(')');
			   qual_str = str.substr(pos1+1, pos2 - pos1);
			   istringstream ss(qual_str);
			   ss >> signal_strength; 
			   signalList.push_back(signal_strength);
			
         //get the IP of current destination's next hop node
         pos1 = str.find_first_of(')');
         pos2 = str.find_first_of('[');
	       ip_str = str.substr(pos1 + 6, pos2 - pos1 - 7);
			   neigbList.push_back(ip_str);

         //get the list of unique neighbors
			   neigbList.unique();
			   NN.count = neigbList.size();

			   NN.neighbors.clear();

			   //put the neighbors and their corresponding signal strengths in the wifiNN list
			   list<std::string>::iterator it4;
     		 for(it4 = neigbList.begin(); it4 != neigbList.end(); ++it4)
			   {
				    vector<string>::iterator pit;
				    int index; 

            pit = find(dstsList.begin(), dstsList.end(), (*it4));
				    index = &(*pit) - &(*dstsList.begin());
	          batman_mesh_info::WifiNN* obj = new batman_mesh_info::WifiNN();
				    obj->ip = (*it4);
				    obj->quality = signalList[index];
		        NN.neighbors.push_back(*obj);
			   }
			
		  } 
	 }
	 pclose(ptr);

	 //publish the list of neighbors and the corresponding link quality
	 NN_pub.publish(NN);
}

string char_to_string(char *input_p)
{
    string str(input_p);
    return str;
}

