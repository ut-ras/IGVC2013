/*
  @date : 05.01.2011
  @ Jihoon Lee
 */
var ros = null;
var rosSubTopic = null;

function move(x,z) {
  ros.publish('/cmd_vel', 'geometry_msgs/Twist', '{"linear":{"x":' + x + ',"y":0,"z":0}, "angular":{"x":0,"y":0,"z":' + z + '}}');
}

function publish(topic,topic_type,msg)
{
  ros.publish(topic,topic_type,msg);
}

function callService(service,service_msg,callback)
{
  log('in callService');
  ros.callService(service,service_msg,callback);
}

function connect(addr) {
  log('Trying to connect...');
  ros = new ROS(addr);
 
  ros.setOnClose(function (e) {
    log('connection closed');

    if(rosSubTopic != null)
      rosSubTopic.clear();

    rosSubTopic = null;
  });
  
  ros.setOnError(function (e) {
    log('connection error');
  });
  
  ros.setOnOpen(function (e) {
    log('connected to ROS');
    log('initializing ROSProxy...');
    try {
      ros.callService('/rosjs/topics','[]',nop);
    }catch (error) {
      log('problem initializing ROSProxy');
      return;
    }
    rosSubTopic = new Array();
    log('initialized');
  });
}

function checkTopic(str) {
  for( x in rosSubTopic) {
    if(rosSubTopic[x] == str) {
      return true;
    }
  }

  return false;
}


function subscribe(topicname,func)
{
  if(rosSubTopic == null)
  {
    log('Connection is not established yet.');
  }
  
  if(checkTopic(topicname))
  {
    log('This topic [' + topicname +'] is already being subscribed.');
    return;
  }
  rosSubTopic.push(topicname);
  log('registering handler for ' + topicname);
                                                                            
  try {
    ros.addHandler(topicname,func);
  }catch (error) {
    log('Problem registering handler');
    return;
  }
  
  log('registered');
  log('subscibing to ' + topicname);
  
  subStr = '["' + topicname + '",100]';
  try {
    ros.callService('/rosjs/subscribe',subStr,nop); 
  }catch(error) {
    log('Problem subscribing!');
  }
  log('subscribed');
}         

