/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#include "../include/udpsocket.h"
#include "../include/datagram.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <iostream>
#include <string.h>

namespace rlab {

using namespace std;

//-----------------------------------------------------------------------------------------------------------
UDPSocket::UDPSocket(bool blocking)
  : serverPort_(0),
    socketFD_(-1),
    blocking_(blocking)
{
}


//-----------------------------------------------------------------------------------------------------------
UDPSocket::UDPSocket(unsigned int port, bool blocking)
  : serverPort_(port),
    socketFD_(-1),
    blocking_(blocking)
{
}


//-----------------------------------------------------------------------------------------------------------
UDPSocket::~UDPSocket()
{
   if (socketFD_ > -1) {
    close(socketFD_);
   }
}


//-----------------------------------------------------------------------------------------------------------
unsigned int UDPSocket::getPort() const
{
  return serverPort_;
}


//-----------------------------------------------------------------------------------------------------------
string UDPSocket::getHostIp(const string& host)
{
	struct hostent* he = gethostbyname(host.c_str());
	string result(host);
	if (he && he->h_addr_list) {
		result = inet_ntoa(*(struct in_addr*)*(he->h_addr_list));
  }
	return result;
}


//-----------------------------------------------------------------------------------------------------------
struct sockaddr_in UDPSocket::makeSocketAddr(const string& host, unsigned int port)
{
	struct sockaddr_in sa;
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);
	if (!inet_aton(getHostIp(host).c_str(), &sa.sin_addr)) {
		sa.sin_addr.s_addr = htonl(INADDR_ANY);
  }
	return sa;
}


//-----------------------------------------------------------------------------------------------------------
struct sockaddr_in UDPSocket::makeBroadcastAddr(unsigned int port)
{
  struct sockaddr_in sa;
  sa.sin_family = AF_INET;
  sa.sin_port = htons(port);
  sa.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  return sa;
}


//-----------------------------------------------------------------------------------------------------------
int	UDPSocket::makeDatagramSocket()
{
	return socket(PF_INET, SOCK_DGRAM, 0);
}


//-----------------------------------------------------------------------------------------------------------
bool UDPSocket::prepareUDPServerSocket()
{
	socketFD_ = makeDatagramSocket();
	if (socketFD_ < 0) {
		return false;
	}

  if (!blocking_) {
    int flags = fcntl(socketFD_, F_GETFL, 0);
    if (flags < 0) {
      return false;
    }
    flags |= O_NONBLOCK;
    if (fcntl(socketFD_, F_SETFL, flags) < 0) {
      return false;
    }
  }

	int opt = 1;
	if (setsockopt(socketFD_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
		return false;
	}

	struct sockaddr_in sa = makeSocketAddr("0.0.0.0", serverPort_);
	if (bind(socketFD_, (struct sockaddr*)&sa, sizeof(sa)) < 0) {
    return false;
  }

	return true;
}


//-----------------------------------------------------------------------------------------------------------
bool UDPSocket::prepareUDPClientSocket()
{
	socketFD_ = makeDatagramSocket();
	if (socketFD_ < 0) {
    return false;
  }

	return true;
}


//-----------------------------------------------------------------------------------------------------------
bool UDPSocket::dataAvailable(unsigned int msec)
{
	fd_set rset;
	FD_ZERO(&rset);
	FD_SET(socketFD_, &rset);

	struct timeval to;
	to.tv_sec = msec / 1000;
	to.tv_usec = (msec % 1000) * 1000;

	int res = select(socketFD_ + 1, &rset, 0, 0, &to);
	if (res == 1) {
    return true;
  }

	/*if (res < 0) {
		reportError("select failed");
	}
  else {
		reportError("select timed out");
	}*/

	return false;
}

//-----------------------------------------------------------------------------------------------------------
Datagram* UDPSocket::receiveDatagram(unsigned int msec)
{
	char buf[Datagram::MAX_LEN];
	struct sockaddr_in peer;

	memset(&peer, 0, sizeof(peer));
	socklen_t addrlen = sizeof(peer);

	if (dataAvailable(msec)) {
		int len = recvfrom(socketFD_, buf, sizeof(buf), 0, (struct sockaddr*)&peer, &addrlen);
		if (len >= 0) {
      return new Datagram(string(buf, len), peer);
    }
	}

	return NULL;
}


//-----------------------------------------------------------------------------------------------------------
bool UDPSocket::sendDatagram(const Datagram& dgram)
{
	return sendto(socketFD_, dgram.getMessage().data(), dgram.getMessage().length(), 0,
    (struct sockaddr*)&dgram.getPeer(), sizeof(dgram.getPeer())) == static_cast<ssize_t>(dgram.getMessage().length());
}


//-----------------------------------------------------------------------------------------------------------
bool UDPSocket::sendMessageToBound(const string& message){
	return send(socketFD_, message.data(), message.length(), 0);
}


//-----------------------------------------------------------------------------------------------------------
void UDPSocket::reportError(const string& msg)
{
  cerr << msg << ":" << strerror(errno) << endl;
}

} // namespace rlab

