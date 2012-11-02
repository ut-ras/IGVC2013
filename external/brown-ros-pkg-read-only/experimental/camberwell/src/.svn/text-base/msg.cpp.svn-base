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
ros::Publisher pub;
char chr;

void send(std::string data) 
{
  std_msgs::String payload;
  payload.data = data;
  //time measurement should include initial transmission
  sent = boost::posix_time::microsec_clock::local_time();
  pub.publish(payload);
}

void processSock(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (chr == (char)0xFF) {
    boost::posix_time::time_duration elapsed = boost::posix_time::microsec_clock::local_time() - sent;
    cout << "unverified echo heard in " << elapsed.total_microseconds() << "(uSecs)" << endl;

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

    if(!error) 
      send(out.str());
  }

  if (!error) {
    boost::asio::async_read(*s, boost::asio::buffer(&chr,1), processSock);
	} else
    cout << error << endl;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "camberwell");
  ros::NodeHandle nh;

  nh.param("camberwell/bytesPerMsg", bytesPerMsg, 131072);
  bytesPerMsg = 1;
  cout << "bytesPerMsg (user setting ignored, due to buffer issues): " << bytesPerMsg << endl;
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

  pub = nh.advertise<std_msgs::String>(topic, 1000);

  boost::asio::io_service ios;
  boost::asio::ip::tcp::resolver resolver(ios);
  boost::asio::ip::tcp::resolver::query q(boost::asio::ip::tcp::v4(), host, sport.str());
  boost::asio::ip::tcp::resolver::iterator itr = resolver.resolve(q);
  boost::asio::ip::tcp::socket socket(ios);
  s = &socket; //acceptable as socket doesn't leave scope until process dies

  urnd.open("/dev/urandom", ios::in | ios::binary);
  s->connect(*itr);
  boost::asio::write(*s, boost::asio::buffer("raw\r\n\r\n", 7));

  cout << "Waiting five seconds for publisher..." << endl;
  ros::Duration(5).sleep();
  send(topic); //establishes topic type

  cout << "Waiting for initial transmission..." << endl;
  ros::Duration(5).sleep();
  std::ostringstream topics;
  topics.put((char)0x00);
  topics << "{\"receiver\":\"/rosbridge/subscribe\",\"msg\":[\"/" << topic << "\",-1]}";
  topics.put((char)0xFF);
  boost::asio::write(*s, boost::asio::buffer(topics.str(), topics.str().length()));

  cout << "Waiting for subscriber..." << endl;
  ros::Duration(5).sleep();
  boost::asio::async_read(*s, boost::asio::buffer(&chr,1), processSock);
  cout << "##################" << endl;

	while(ros::ok()) {
		ios.poll_one();
	}

  urnd.close();
  s->close();
  free(junk);

  return 0;
}
