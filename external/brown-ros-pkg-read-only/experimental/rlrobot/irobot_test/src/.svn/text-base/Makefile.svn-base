# This file is a regular Makefile that does not use CMake. You
# can use this file and make to build your project.

CLIENTSOURCES = client.cpp udpsocket.cpp packet.cpp objectposepacket.cpp objectpose.cpp localizableobject.cpp datagram.cpp

CLIENTOBJECTS=$(CLIENTSOURCES:.cpp=.o)

CLIENT_EXE=client

#CFLAGS=`pkg-config --cflags playerc++`-Wall -g -c #-ansi
#LDFLAGS=`pkg-config --libs playerc++`

all: $(CLIENT_EXE)

client: $(CLIENTOBJECTS)
	g++ $(CLIENTOBJECTS) $(LDFLAGS) -o $@

clean:
	rm -f *.o $(CLIENT_EXE)

client.o: udpsocket.h datagram.h objectposepacket.h localizableobject.h
udpsocket.o: udpsocket.h
datagram.o: datagram.h
objectposepacket.o: objectposepacket.h
localizableobject.o: localizableobject.h
packet.o: packet.h
