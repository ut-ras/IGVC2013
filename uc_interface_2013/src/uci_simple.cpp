/*
 * uci_simple.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: ras
 */

// standard library includes
#include <string>
#include <sstream>
#include <iostream>

// boost library includes
#include <boost/asio/serial_port.hpp>
#include <boost/iostreams/stream.hpp>

// ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <uc_interface_2013/Telemetry.h>
// #include <ocean_server_imu/ocean_server_5000.hpp>

	bool ready_to_send;
using namespace std;

#define LOG(var) ROS_DEBUG_STREAM(#var << ": " << var)

/// copied from
/// http://groups.google.com/group/boost-list/browse_thread/thread/7c271c4269bd0f93

class serial_device {
public:
	typedef char char_type;
	typedef boost::iostreams::bidirectional_device_tag category;
	serial_device(boost::asio::serial_port& port) :
		serial_port_(port) {
		boost::asio::serial_port_base::baud_rate baud_option(115200*2);             
        serial_port_.set_option(baud_option);
	}

	std::streamsize read(char* buf, std::streamsize n) {
		boost::system::error_code ec;
		int len = serial_port_.read_some(boost::asio::buffer(buf, n), ec);
		return len;
	}
	std::streamsize write(const char* buf, std::streamsize n) {
		return serial_port_.write_some(boost::asio::buffer(buf, n));
	}
private:
	boost::asio::serial_port& serial_port_;
};

typedef boost::iostreams::stream<serial_device> serial_stream;

class uci_simple {
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	boost::asio::io_service io_;
	boost::asio::serial_port serial_port_;
	serial_device serial_device_;
	serial_stream serial_stream_;

public:
	uci_simple();
	void callback(const geometry_msgs::TwistConstPtr &vel_cmd);
	bool readAndPublishTelemetry();

private:
	static bool parseTelemetryData(const string &nmea_sentence,
			uc_interface_2013::Telemetry &msg);
};

uci_simple::uci_simple() :
	nh_(), sub_(nh_.subscribe<geometry_msgs::Twist> ("vel_cmd", 1,
			&uci_simple::callback, this)), pub_(nh_.advertise<
			uc_interface_2013::Telemetry> ("telemetry", 100)),

	io_(), serial_port_(io_, "/dev/ttyACM0"), serial_device_(serial_port_),
			serial_stream_(serial_device_) {
	ready_to_send = true;
}

void uci_simple::callback(const geometry_msgs::TwistConstPtr &vel_cmd) {
	ROS_INFO_STREAM(*vel_cmd);
	
	//int lin = static_cast<int> (vel_cmd->linear.x);
	//int ang = static_cast<int> (vel_cmd->angular.z);
	
	int x = static_cast<int> (vel_cmd->linear.x + 127);
	int y = static_cast<int> (vel_cmd->angular.z + 127);

	//>SVXA:[voltage]\r
	ROS_INFO_STREAM(">SVXA:" << x << endl);
	ROS_INFO_STREAM(">SVYA:" << y << endl);
	serial_stream_ << ">SVXA:" << x << endl;
	serial_stream_ << ">SVYA:" << y << endl;
}

bool uci_simple::readAndPublishTelemetry() {
	uc_interface_2013::Telemetry msg;

	bool success = true;

	string nmea_sentence;
	serial_stream_ >> nmea_sentence;
	msg.header.stamp = ros::Time::now();

	LOG(nmea_sentence);

	//success = isValid(nmea_sentence);

	if (success) {
		success = parseTelemetryData(nmea_sentence, msg);
	}

	if (success) {
		ROS_DEBUG_STREAM("\033[0;36;40mactual: " << msg.actual << "\033[0;37;40m"); // "\033[0;36;40m" and "\033[0;37;40m" are for color changes
	}

	return success;
}

bool uci_simple::parseTelemetryData(const string &nmea_sentence,
		uc_interface_2013::Telemetry &msg) {

	bool success = true;

	char field;
	stringstream string_stream(nmea_sentence);

	string_stream >> field;
	if(field == 6)
	{
		ROS_INFO("GOT_ACK!");
		ready_to_send = true;
	}
	else
	{
		ROS_INFO("Got %c :(", field);
	}

	//Sample Telemetry data: "$O74P1234T4312*6A"
	/*while (string_stream.peek() != '*') {
		string_stream >> field;
		switch (field) {
		case 'O':
			string_stream >> msg.command;
			break;
		case 'P':
			string_stream >> msg.actual;
			break;
		case 'T':
			string_stream >> msg.header.seq;
			break;
		default:
			ROS_WARN_STREAM_ONCE("Unknown field delimiter: " << field << " in senctence " << nmea_sentence);
			//      success = false;
			break;
		}
	}*/

	return success;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "uci_simple");
	uci_simple uci;
	while (ros::ok()) {
		//if(!ready_to_send)
		//{
		//	ROS_INFO("Waiting for ack...");
		//	uci.readAndPublishTelemetry();
		//}
		ros::spinOnce();
	}
	return 0;
}
