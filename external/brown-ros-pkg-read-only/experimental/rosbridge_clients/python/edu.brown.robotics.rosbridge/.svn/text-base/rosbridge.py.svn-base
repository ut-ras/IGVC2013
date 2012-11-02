import socket, asynchat, json, asyncore
import traceback

class Rosbridge():
    
    callbacks = {}
    
    def __init__(self, host, port):
        self.connection = RosbridgeTCPConnection(host, port)
        self.connection.registerCallback(self.onMessageReceived)

    """ Returns the underlying socket used for this connection.  Call this method
    if you need to, for example, select over the socket """
    def socket(self):
        return self.connection.socket
        
    """ Publishes the provided object on the given topic, with the given type.  
    obj should be a dict mimicking JSON style object hierarchies ."""
    def publish(self, topic, typeStr, obj):
        self.callService(topic, obj, typeStr)
        
    """ Subscribes to the given topic.  Provides a callback to be called when 
    messages are received on that topic.  Optionally specify a message rate
    (a minimum amount of time that the rosbridge server must wait between sending messages)
    Callback will only be called when the check() method of this object is called """
    def subscribe(self, topic, message_type, callback, rate=-1):
        args = [ topic, rate, message_type ]       
        self.addCallback(topic, callback)
        self.callService("/rosbridge/subscribe", args) 
    
    """ Calls the service with the given name. msg should be a dict mimicking
    JSON style object hierarchies. """
    def callService(self, serviceName, msg, typeStr=None):
        call = { "receiver": serviceName, "msg": msg }
        if (typeStr): 
            call["type"] = typeStr
        self.send(call)
    
    """ Private method, don't use """
    def send(self, obj):
        try:
            self.connection.sendString(json.dumps(obj))
            self.check()
        except:
            traceback.print_exc()
            raise
        
    """ Private method, don't use """
    def addCallback(self, topic, callback):
        if (self.callbacks.has_key(topic)):
            self.callbacks[topic].append(callback)
        else:
            self.callbacks[topic] = [ callback ]
        
    """ Private method, don't use """
    def onMessageReceived(self, message):
        try:
            # Load the string into a JSON object
            obj = json.loads(message)
            # Extract the receiver and msg fields
            receiver = obj["receiver"]
            msg = obj["msg"]
            # Call any registered callbacks
            if self.callbacks.has_key(receiver):
                for callback in self.callbacks[receiver]:
                    try:
                        callback(msg)
                    except:
                        print "exception on callback", callback, "from", receiver
                        traceback.print_exc()
                        raise
        except:
            print "exception in onMessageReceived"
            print "message", message
            traceback.print_exc()
            raise
    
    """ Checks the socket for incoming messages, then returns.  Blocks for the amount of time
    specified by timeout, or until data is received. If there are incoming messages, 
    the callback for messages on that topic is called """            
    def check(self, timeout=0.01):
        # Check each socket once before returning
        asyncore.loop(count=1, timeout=timeout)

class RosbridgeTCPConnection(asynchat.async_chat):
    
    connected = False
    preconnectionBuffer = "";
    
    callbacks = []
    message = ""
    
    def __init__(self, host, port):
        self.socket = socket.socket()
        asynchat.async_chat.__init__(self, sock=self.socket)
        
        # Set the terminator to the \xff bit
        self.set_terminator('\xff')
        
        # Create the socket connection
        self.connect((host, port))


    def handle_connect(self):
        self.send('raw\r\n\r\n')
        if len(self.preconnectionBuffer) > 0:
            self.push(self.preconnectionBuffer)
            self.preconnectionBuffer = ""
        self.connected=True
        
    def sendString(self, message):
        data = "\x00"+message+"\xff"
        if (self.connected):
            self.push(data)
        else:
            self.preconnectionBuffer += data
        
    def registerCallback(self, callback):
        self.callbacks.append(callback)
        
    def collect_incoming_data(self, data):
        self.message += data
        
    def found_terminator(self):
        # Copy out the completed message and clear the buffer
        completemessage = self.message
        self.message = ""
        
        # Get rid of the leading x00
        if completemessage[0]=='\x00':
            completemessage = completemessage[1:]
                
        # Call the handlers
        for callback in self.callbacks:
            callback(completemessage)