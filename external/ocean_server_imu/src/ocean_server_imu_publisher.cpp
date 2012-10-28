/*
 * ocean_server_imu_publisher.cpp
 *
 *      Author: Joshua James, Nicu Stiurca
 */
 
// standard library includes
#include <string>
#include <sstream>

// boost library includes
#include <boost/asio/serial_port.hpp>
#include <boost/iostreams/stream.hpp>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// project includes
#include "ocean_server_imu/RawData.h"
#include "ocean_server_imu/ocean_server_5000.hpp"

using namespace std;
//using namespace boost::asio;
//using namespace boost::iostreams;

#define LOG(var) ROS_DEBUG_STREAM(#var << ": " << var)


/// copied from
/// http://groups.google.com/group/boost-list/browse_thread/thread/7c271c4269bd0f93
class serial_device
{
public:
  typedef char char_type;
  typedef boost::iostreams::bidirectional_device_tag category;
  serial_device(boost::asio::serial_port& port) :
    serial_port_(port)
  {
  }
  
  std::streamsize read(char* buf, std::streamsize n)
  {
    return serial_port_.read_some(boost::asio::buffer(buf, n));
  }
  std::streamsize write(const char* buf, std::streamsize n)
  {
    return serial_port_.write_some(boost::asio::buffer(buf, n));
  }
private:
  boost::asio::serial_port& serial_port_;
};

typedef boost::iostreams::stream<serial_device> serial_stream;

bool fillMsg(ocean_server_imu::RawData& msg, serial_stream& stream)
{
  bool success = true;

  string nmea_sentence;
  stream >> nmea_sentence;
  msg.header.stamp = ros::Time::now();

  LOG(nmea_sentence);

  success = isValid(nmea_sentence);

  if(success) 
  {
    success = parseOSData(nmea_sentence, msg);
  }

  if(success) 
  {
    ROS_DEBUG_STREAM("\033[0;36;40myaw: " << msg.yaw << "\033[0;37;40m"); // "\033[0;36;40m" and "\033[0;37;40m" are for color changes
  }

  return success;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "ocean_server_imu_publisher");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<ocean_server_imu::RawData> ("imu_data", 1);

  boost::asio::io_service io;
  boost::asio::serial_port port(io, "/dev/imu");
  port.set_option(boost::asio::serial_port_base::baud_rate(115200));
  serial_device device(port);
  serial_stream stream(device);

  ocean_server_imu::RawData msg;
  msg.header.frame_id = "oceanserver_link";

  while (ros::ok()) 
  {
    if (fillMsg(msg, stream)) 
    {
      imu_pub.publish(msg);
    }

    ros::spinOnce();
  }

  return 0;
}
