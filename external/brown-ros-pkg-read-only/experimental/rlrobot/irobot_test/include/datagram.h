/**
 * Brown University, Computer Science Department
 *
 * Author: Jonas Schwertfeger <js at cs.brown.edu>
 * Date:   10/2/2007
 *
 */

#ifndef DATAGRAM_H
#define DATAGRAM_H DATAGRAM_H

#include "global.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

namespace rlab {

class Datagram {
public:
	static const long MAX_LEN = 65507L; //= 2^16 - IP-Headersize(20) - UDP-Headersize(8) - 1

public:
  Datagram(const std::string& msg, unsigned int peerPort, const std::string& peerHost);
  Datagram(const std::string& msg, struct sockaddr_in peer);

  void setMessage(const std::string& msg);
  const std::string& getMessage() const;

  const struct sockaddr_in& getPeer() const;

  std::string getPeerInfo() const;

private:
	std::string	msg_;
	struct sockaddr_in peer_;
};

} // namespace rlab

#endif // DATAGRAM_H

