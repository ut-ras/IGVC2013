#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
using namespace std;

//global params
int bytesPerMsg, maxMsgsPerSec;
double minMicroSecs, sand;
char* junk;
boost::asio::ip::tcp::socket* s;
ifstream urnd;
std::string topic;
boost::posix_time::ptime sent;

void send(std::string data) 
{
  std::ostringstream msg;
  msg.put((char)0x00);
  msg << "{\"receiver\":\"/" << topic << "\",\"msg\":{\"data\":\"" << data << "\"},\"type\":\"std_msgs/String\"}";
  msg.put((char)0xFF);

  //time measurement should include initial transmission
  sent = boost::posix_time::microsec_clock::local_time();
  boost::asio::write(*s, boost::asio::buffer(msg.str(), msg.str().length()));
}

void processMsg(const std_msgs::String::ConstPtr& msg)
{
  //local_time is *not* monatomic and microseconds do not exist on all
  //platforms. The following may be entirely sketchy.
  boost::posix_time::time_duration elapsed = boost::posix_time::microsec_clock::local_time() - sent;

  if (msg->data.compare("echo.")) {
    if (msg->data.compare(junk) == 0) {
      cout << "proper";
    } else {
      cout << "improper";
    }
    cout << " echo heard after (uSecs): " << elapsed.total_microseconds() << endl;
  }

  urnd.read((char*)junk, sizeof(char)*bytesPerMsg);

  std::ostringstream out;
  for (int i = 0; i < bytesPerMsg; i++) {
    junk[i] = (char)((abs((int)junk[i])%26)+65);
    out.put(junk[i]);
  }

  elapsed = boost::posix_time::microsec_clock::local_time() - sent;
  sand += minMicroSecs - elapsed.total_microseconds();

  //sleep innaccurate on EC2 and other platforms, so we never sleep less than
  //1/30 of a second
  if (sand >= 33333) {
    struct timespec toSleep = {0,sand*1000};
    nanosleep(&toSleep, (struct timespec *)NULL);
    sand = 0;
  }

  send(out.str());
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "camberwell");
  ros::NodeHandle nh;

  nh.param("camberwell/bytesPerMsg", bytesPerMsg, 131072);
  cout << "bytesPerMsg: " << bytesPerMsg << endl;
  nh.param("camberwell/maxMsgsPerSec", maxMsgsPerSec, 100000);
  cout << "maxMsgsPerSec: " << maxMsgsPerSec << endl;
  minMicroSecs = (double)1000000/(double) maxMsgsPerSec;
  sand = 0;
  junk = (char*) malloc((bytesPerMsg+1)*sizeof(char));
  junk[bytesPerMsg] = (char) NULL;

  std::string host;
  int port;
  nh.param<std::string>("camberwell/host", host, "localhost");
  cout << "host: " << host << endl;
  nh.param("camberwell/port", port, 9090);
  cout << "port: " << port << endl;
  std::ostringstream sport;
  sport << port;

  nh.param<std::string>("camberwell/topic", topic, "camberwell");
  cout << "topic: " << topic << endl;

  boost::asio::io_service ios;
  boost::asio::ip::tcp::resolver resolver(ios);
  boost::asio::ip::tcp::resolver::query q(boost::asio::ip::tcp::v4(), host, sport.str());
  boost::asio::ip::tcp::resolver::iterator itr = resolver.resolve(q);
  boost::asio::ip::tcp::socket socket(ios);
  s = &socket; //acceptable as socket doesn't leave scope until process dies

  urnd.open("/dev/urandom", ios::in | ios::binary);
  s->connect(*itr);
  boost::asio::write(*s, boost::asio::buffer("raw\r\n\r\n", 7));
  send(topic); //establishes topic type
  //allow for processing
  cout << "Waiting five seconds for initial transmission..." << endl;
  ros::Duration(5).sleep();

  ros::Subscriber sub = nh.subscribe(topic, 1000, processMsg);

  //subscriber thread creation doesn't block
  cout << "Waiting five seconds for subscriber thread..." << endl;
  ros::Duration(5).sleep();
  cout << "##################" << endl;
  send("echo.");

  ros::spin();

  urnd.close();
  s->close();
  free(junk);

  return 0;
}
