; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude Halt-request.msg.html

(cl:defclass <Halt-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Halt-request (<Halt-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Halt-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Halt-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<Halt-request> is deprecated: use amtec-srv:Halt-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Halt-request>) ostream)
  "Serializes a message object of type '<Halt-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Halt-request>) istream)
  "Deserializes a message object of type '<Halt-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Halt-request>)))
  "Returns string type for a service object of type '<Halt-request>"
  "amtec/HaltRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Halt-request)))
  "Returns string type for a service object of type 'Halt-request"
  "amtec/HaltRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Halt-request>)))
  "Returns md5sum for a message object of type '<Halt-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Halt-request)))
  "Returns md5sum for a message object of type 'Halt-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Halt-request>)))
  "Returns full string definition for message of type '<Halt-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Halt-request)))
  "Returns full string definition for message of type 'Halt-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Halt-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Halt-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Halt-request
))
;//! \htmlinclude Halt-response.msg.html

(cl:defclass <Halt-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Halt-response (<Halt-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Halt-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Halt-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<Halt-response> is deprecated: use amtec-srv:Halt-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Halt-response>) ostream)
  "Serializes a message object of type '<Halt-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Halt-response>) istream)
  "Deserializes a message object of type '<Halt-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Halt-response>)))
  "Returns string type for a service object of type '<Halt-response>"
  "amtec/HaltResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Halt-response)))
  "Returns string type for a service object of type 'Halt-response"
  "amtec/HaltResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Halt-response>)))
  "Returns md5sum for a message object of type '<Halt-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Halt-response)))
  "Returns md5sum for a message object of type 'Halt-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Halt-response>)))
  "Returns full string definition for message of type '<Halt-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Halt-response)))
  "Returns full string definition for message of type 'Halt-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Halt-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Halt-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Halt-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Halt)))
  'Halt-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Halt)))
  'Halt-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Halt)))
  "Returns string type for a service object of type '<Halt>"
  "amtec/Halt")