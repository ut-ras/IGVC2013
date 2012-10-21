; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude Home-request.msg.html

(cl:defclass <Home-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Home-request (<Home-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Home-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Home-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<Home-request> is deprecated: use amtec-srv:Home-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Home-request>) ostream)
  "Serializes a message object of type '<Home-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Home-request>) istream)
  "Deserializes a message object of type '<Home-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Home-request>)))
  "Returns string type for a service object of type '<Home-request>"
  "amtec/HomeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Home-request)))
  "Returns string type for a service object of type 'Home-request"
  "amtec/HomeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Home-request>)))
  "Returns md5sum for a message object of type '<Home-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Home-request)))
  "Returns md5sum for a message object of type 'Home-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Home-request>)))
  "Returns full string definition for message of type '<Home-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Home-request)))
  "Returns full string definition for message of type 'Home-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Home-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Home-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Home-request
))
;//! \htmlinclude Home-response.msg.html

(cl:defclass <Home-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Home-response (<Home-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Home-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Home-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<Home-response> is deprecated: use amtec-srv:Home-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Home-response>) ostream)
  "Serializes a message object of type '<Home-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Home-response>) istream)
  "Deserializes a message object of type '<Home-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Home-response>)))
  "Returns string type for a service object of type '<Home-response>"
  "amtec/HomeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Home-response)))
  "Returns string type for a service object of type 'Home-response"
  "amtec/HomeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Home-response>)))
  "Returns md5sum for a message object of type '<Home-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Home-response)))
  "Returns md5sum for a message object of type 'Home-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Home-response>)))
  "Returns full string definition for message of type '<Home-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Home-response)))
  "Returns full string definition for message of type 'Home-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Home-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Home-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Home-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Home)))
  'Home-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Home)))
  'Home-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Home)))
  "Returns string type for a service object of type '<Home>"
  "amtec/Home")