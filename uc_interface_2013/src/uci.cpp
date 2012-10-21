/*
 * ocean_server_imu_publisher.cpp
 *
 *      Author: Joshua James, Nicu Stiurca, Robby Nevels
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

using namespace std;

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
    boost::system::error_code ec;
    int len = serial_port_.read_some(boost::asio::buffer(buf, n), ec);
    return len;
  }
  std::streamsize write(const char* buf, std::streamsize n)
  {
    return serial_port_.write_some(boost::asio::buffer(buf, n));
  }
private:
  boost::asio::serial_port& serial_port_;
};

typedef boost::iostreams::stream<serial_device> serial_stream;

boost::asio::io_service io;
boost::asio::serial_port port(io, "/dev/ucontroller");
serial_device device(port);
serial_stream stream(device);

void getLine(char* buffer, int len) {
	stream.getline(buffer, len, '\r');
}

void sendLine(string str) {
	char *charArr = (char*)str.c_str();
	device.write(charArr, str.length());
}

void sendData(const string opcode, float data) {
	int checksum = '{'^':'^'['^']'^'}'^':'^'['^']';
	int i = 0;
	for(i = 0; i < 4; i++) checksum ^= opcode[i];
	std::ostringstream ds;
	ds << data;
	string dataStr = ds.str();
	i = 0;
	char ch = dataStr[i];
	while(ch) {
		checksum ^= ch;
		i++;
		ch = dataStr[i];
	}

	char ch0 = checksum%16;
	if (ch0 > 10) ch0 += 65-10;
	else ch0 += 48;
	char ch1 = checksum/16;
	if (ch1 > 10) ch1 += 65-10;
	else ch1 += 48;

	std::ostringstream s;
	s << "{" << opcode << ":[" << data << "]}:[" << ch1 << ch0 << "]\n";
	string sendStr = s.str();
	ROS_INFO_STREAM(sendStr);
	sendLine(sendStr);
}

// Send:	{SVLM:[voltage]}:[Checksum]\n
void setVoltageOfLeftMotor(float voltage) {
	sendData("SVLM", voltage);
}

// Send:	{SVRM:[voltage]}:[Checksum]\n
void setVoltageOfRightMotor(float voltage) {
	sendData("SVRM", voltage);
}

// Send: 	{SVBM:[leftVoltage]_SVBM:[rightVoltage]}:[Checksum]\n
void setVoltageOfDriveMotor(float voltage) {
	sendData("SVBM", voltage);
}

// Send: 	{SASS:[angle]}:[Checksum]\n
void setAngleOfSteeringServo(float angle) {
	sendData("SASS", angle);
}

// Send: 	{SALS:[angle]}:[Checksum]\n
void setAngleOfLidarServo(float angle){
	sendData("SALS", angle);
}

// Send: 	None (uC sends every 100ms automatically)
// Get:		{ENCL:[Left encoder counts]_ENCR:[Right encoder counts]_VELL:[Left velocity]_VELR:[Right velocity]_STEA:[steering angle]_HOKA:[hokuyo angle]_TIME:[Timestamp]}:[Checksum]\n
void getEncoderCounts( void (*Callback) (int lEncoderTicks, int rEncoderTicks, int timeStamp) ) {

}

int getXor(char str[]) {
	int result = str[0], i = 1;
	while(str[i]) {
		result ^= str[i];
		i++;
	}
	return result;
}

enum MessageIndexes{ENCL_I, ENCR_I, VELL_I, VELR_I, STEA_I, HOKA_I, TIME_I, CKSM_I};

int parse(char str[], int messageValues[]) {
		// k: index of char in string for a particular value
		//	ex) in ...ENCL:[123]... if i points to the character '2', then k = 1
		//		if i points to a non-number character, then k = -1
		int i = 0, k = -1, valueIndex = 0, maxCharPerNum = 10;
		char numStr[10];

		// determine checksum for everything but the numbers.
		// when iterating thru the message, collect the checksum on all non-checksum numbers
		// getXor("{ENCL:[]_ENCR:[]_VELL:[]_VELR:[]_STEA:[]_HOKA:[]_TIME:[]}:[]") = 29
		int cksm = 29;

		while(str[i]) {
			if (str[i]=='[') {
				k = 0;
			} else if (str[i] == ']') {
				// get an integer from the ascii numStr array
				int value = 0;
				if (valueIndex != CKSM_I) {
					int powerOfTen = 1;
					for(int x = k-1; x >= 0; x--) {
						value += (numStr[x]-48)*powerOfTen;
						powerOfTen *= 10;
					}
				} else {
					int powerOf16 = 1;
					for(int x = k-1; x >= 0; x--) {
						if (numStr[x]-48>10)
							value += (numStr[x]-65+10)*powerOf16;
						else
							value += (numStr[x]-48)*powerOf16;
						powerOf16 *= 16;
					}
				}

				messageValues[valueIndex] = value;
				valueIndex++;
				k = -1;

				// clear the ascii num array
				for(int j = 0; j < maxCharPerNum; j++)
					numStr[j] = 0;
			} else if (k > -1) {
				numStr[k] = str[i];
				k++;
				if (valueIndex != CKSM_I)
					cksm ^= str[i];
			}
			i++;
		}

		if (valueIndex != 8) return false;
		return cksm == messageValues[CKSM_I];
}

int main(int argc, char**  argv)
{
	ros::init(argc, argv, "uszynski_the_ucontroller");
	port.set_option(boost::asio::serial_port_base::baud_rate(115200));

	while(1) {
		// read
		char line[1024];
		getLine(line, 1024);
		//ROS_INFO_STREAM(line);

		// parse
		int messageValues[8];
		int success = parse(line, messageValues);

		// publish
		if (success) {
			std::ostringstream s;
			s << "Encl: " << messageValues[ENCL_I] << "\n";
			s << "Encr: " << messageValues[ENCR_I] << "\n";
			s << "Vell: " << messageValues[VELL_I] << "\n";
			s << "Velr: " << messageValues[VELR_I] << "\n";
			s << "Stea: " << messageValues[STEA_I] << "\n";
			s << "Hoka: " << messageValues[HOKA_I] << "\n";
			s << "Time: " << messageValues[TIME_I];
			ROS_INFO_STREAM(s);
		} else {
			ROS_INFO_STREAM("Unsucceful read:");
			/*
			int i = 0;
			while(line[i]) {
				printf("%d ",line[i]);
				i++;
			}
			printf("\n");
			*/
			printf("[%s]\n",line);
		}

		// pause...?
	}
}
