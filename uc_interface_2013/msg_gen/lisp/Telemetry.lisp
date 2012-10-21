; Auto-generated. Do not edit!


(cl:in-package uc_interface_2013-msg)


;//! \htmlinclude Telemetry.msg.html

(cl:defclass <Telemetry> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (command
    :reader command
    :initarg :command
    :type cl:integer
    :initform 0)
   (actual
    :reader actual
    :initarg :actual
    :type cl:integer
    :initform 0))
)

(cl:defclass Telemetry (<Telemetry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Telemetry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Telemetry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name uc_interface_2013-msg:<Telemetry> is deprecated: use uc_interface_2013-msg:Telemetry instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uc_interface_2013-msg:header-val is deprecated.  Use uc_interface_2013-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uc_interface_2013-msg:command-val is deprecated.  Use uc_interface_2013-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'actual-val :lambda-list '(m))
(cl:defmethod actual-val ((m <Telemetry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader uc_interface_2013-msg:actual-val is deprecated.  Use uc_interface_2013-msg:actual instead.")
  (actual m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Telemetry>) ostream)
  "Serializes a message object of type '<Telemetry>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'actual)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Telemetry>) istream)
  "Deserializes a message object of type '<Telemetry>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'actual) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Telemetry>)))
  "Returns string type for a message object of type '<Telemetry>"
  "uc_interface_2013/Telemetry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Telemetry)))
  "Returns string type for a message object of type 'Telemetry"
  "uc_interface_2013/Telemetry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Telemetry>)))
  "Returns md5sum for a message object of type '<Telemetry>"
  "099f23789666af6c5996c42d9aa81d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Telemetry)))
  "Returns md5sum for a message object of type 'Telemetry"
  "099f23789666af6c5996c42d9aa81d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Telemetry>)))
  "Returns full string definition for message of type '<Telemetry>"
  (cl:format cl:nil "Header header~%~%int32 command~%int32 actual~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Telemetry)))
  "Returns full string definition for message of type 'Telemetry"
  (cl:format cl:nil "Header header~%~%int32 command~%int32 actual~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Telemetry>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Telemetry>))
  "Converts a ROS message object to a list"
  (cl:list 'Telemetry
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':actual (actual msg))
))
