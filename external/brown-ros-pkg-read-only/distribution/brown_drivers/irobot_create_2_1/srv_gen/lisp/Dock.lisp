; Auto-generated. Do not edit!


(cl:in-package irobot_create_2_1-srv)


;//! \htmlinclude Dock-request.msg.html

(cl:defclass <Dock-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Dock-request (<Dock-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dock-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dock-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name irobot_create_2_1-srv:<Dock-request> is deprecated: use irobot_create_2_1-srv:Dock-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dock-request>) ostream)
  "Serializes a message object of type '<Dock-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dock-request>) istream)
  "Deserializes a message object of type '<Dock-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dock-request>)))
  "Returns string type for a service object of type '<Dock-request>"
  "irobot_create_2_1/DockRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dock-request)))
  "Returns string type for a service object of type 'Dock-request"
  "irobot_create_2_1/DockRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dock-request>)))
  "Returns md5sum for a message object of type '<Dock-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dock-request)))
  "Returns md5sum for a message object of type 'Dock-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dock-request>)))
  "Returns full string definition for message of type '<Dock-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dock-request)))
  "Returns full string definition for message of type 'Dock-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dock-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dock-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Dock-request
))
;//! \htmlinclude Dock-response.msg.html

(cl:defclass <Dock-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Dock-response (<Dock-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dock-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dock-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name irobot_create_2_1-srv:<Dock-response> is deprecated: use irobot_create_2_1-srv:Dock-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Dock-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader irobot_create_2_1-srv:success-val is deprecated.  Use irobot_create_2_1-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dock-response>) ostream)
  "Serializes a message object of type '<Dock-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dock-response>) istream)
  "Deserializes a message object of type '<Dock-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dock-response>)))
  "Returns string type for a service object of type '<Dock-response>"
  "irobot_create_2_1/DockResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dock-response)))
  "Returns string type for a service object of type 'Dock-response"
  "irobot_create_2_1/DockResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dock-response>)))
  "Returns md5sum for a message object of type '<Dock-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dock-response)))
  "Returns md5sum for a message object of type 'Dock-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dock-response>)))
  "Returns full string definition for message of type '<Dock-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dock-response)))
  "Returns full string definition for message of type 'Dock-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dock-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dock-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Dock-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Dock)))
  'Dock-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Dock)))
  'Dock-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dock)))
  "Returns string type for a service object of type '<Dock>"
  "irobot_create_2_1/Dock")