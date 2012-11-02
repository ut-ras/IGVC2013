#include <topDownLocObservation>

using namespace std;
using namespace rlab;


topDownLocObservation::topDownLocObservation()
{
  udpSocket(8856, false);
    if (!udpSocket.prepareUDPServerSocket()) {
    cerr << "Failed to create UDP socket on port " << udpSocket.getPort() << "." << endl;
    return 1;
  }
  cout << "Listening on port " << udpSocket.getPort() << "..." << endl;
}
