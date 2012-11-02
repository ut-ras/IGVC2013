/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#include "../include/datagram.h"
#include "../include/udpsocket.h"
#include <sstream>

namespace rlab {

using namespace std;

//-----------------------------------------------------------------------------------------------------------
Datagram::Datagram(const string& msg, unsigned int peerPort, const string& peerHost)
  : msg_(msg),
    peer_(UDPSocket::makeSocketAddr(peerHost, peerPort))
{
}


//-----------------------------------------------------------------------------------------------------------
Datagram::Datagram(const string& msg, struct sockaddr_in peer)
  : msg_(msg),
    peer_(peer)
{
}


//-----------------------------------------------------------------------------------------------------------
void Datagram::setMessage(const string& msg)
{
  msg_ = msg;
}


//-----------------------------------------------------------------------------------------------------------
const string& Datagram::getMessage() const
{
  return msg_;
}


//-----------------------------------------------------------------------------------------------------------
const struct sockaddr_in& Datagram::getPeer() const
{
  return peer_;
}


//-----------------------------------------------------------------------------------------------------------
string Datagram::getPeerInfo() const
{
	ostringstream result;
	result << inet_ntoa(peer_.sin_addr);
	result << ":" << ntohs(peer_.sin_port);
	return result.str();
}

} // namespace rlab

