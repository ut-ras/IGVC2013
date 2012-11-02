from rosbridge import Rosbridge

if __name__=="__main__":
    bridge = Rosbridge("localhost", 9090)
    
    msg = { "data": "Hello from Python!" }
    bridge.publish("/jon", "std_msgs/String", msg)
    print "Published: %s" % (msg, )
    
    def callback(msg):
        print "Received msg: %s" % (msg, )
    
    bridge.subscribe("/jon", "std_msgs/String", callback)
    
    while True:
        bridge.check()