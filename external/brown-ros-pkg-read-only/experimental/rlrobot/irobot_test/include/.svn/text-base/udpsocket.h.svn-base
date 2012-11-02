/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#ifndef UDPSOCKET_H
#define UDPSOCKET_H UDPSOCKET_H

#include "./global.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

namespace rlab {

class Datagram;

class UDPSocket
{
public:
	UDPSocket(bool blocking = false);
	UDPSocket(unsigned int port, bool blocking = false);
	~UDPSocket();

	bool prepareUDPServerSocket();
	bool prepareUDPClientSocket();

	bool dataAvailable(unsigned int msec = 0);
	Datagram* receiveDatagram(unsigned int timeoutmsec = 1000);

	bool sendDatagram(const Datagram& dgram);
	bool sendMessageToBound(const std::string& msg);

  unsigned int getPort() const;

  static std::string getHostIp(const std::string& host);
  static struct sockaddr_in makeSocketAddr(const std::string& host, unsigned int port);
  static struct sockaddr_in makeBroadcastAddr(unsigned int port);
  static int makeDatagramSocket();

private:
  static void reportError(const std::string& msg);

private:
  unsigned int serverPort_;
  int socketFD_;
  bool blocking_;
};

} // namespace rlab

#endif // UDPSOCKET_H

