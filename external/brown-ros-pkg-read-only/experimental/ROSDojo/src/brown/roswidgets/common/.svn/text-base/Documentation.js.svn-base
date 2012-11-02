dojo.provide("roswidgets.common.Documentation");

dojo.require("rosdojo.rosdojo");

dojo.declare("roswidgets.common.Documentation", null, {
    /*
     * This widget provides methods for getting documentation and links
     * for ROS and ROS stuffs.  At the moment it is barebones, and only
     * provides URLs to external ROS documentation pages
     */
    
    wikiURL: function(packageName) {
        return "http://ros.org/wiki/"+packageName;
    },
    
    packageURL: function(packageName) {
        return "http://www.ros.org/browse/details.php?name="+packageName;
    },
    
    messageAPIURL: function(packageName, messageName) {
        return "http://www.ros.org/doc/api/"+packageName+"/html/msg/"+messageName+".html";
    },
    
    serviceAPIURL: function(packageName, serviceName) {
        return "http://www.ros.org/doc/api/"+packageName+"/html/srv/"+serviceName+".html";
    },
    
    packageName: function(messageType) {
        var split = messageType.indexOf("/");
        if (split>0) {
            return messageType.slice(0, split);
        } else {
            return null;
        }        
    },
    
    messageName: function(messageType) {
        var split = messageType.indexOf("/");
        if (split>0) {
            return messageType.slice(split+1, messageType.length);
        } else {
            return null;
        }        
    },
    
    serviceName: function(serviceType) {
        var split = serviceType.indexOf("/");
        if (split>0) {
            return serviceType.slice(split+1, serviceType.length);
        } else {
            return null;
        }             
    }
    
});

dojo.global.ros.docs = new roswidgets.common.Documentation();