; Auto-generated. Do not edit!


(cl:in-package gumstix_memread_bma180-msg)


;//! \htmlinclude bma180meas.msg.html

(cl:defclass <bma180meas> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vals
    :reader vals
    :initarg :vals
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass bma180meas (<bma180meas>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bma180meas>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bma180meas)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gumstix_memread_bma180-msg:<bma180meas> is deprecated: use gumstix_memread_bma180-msg:bma180meas instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <bma180meas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gumstix_memread_bma180-msg:header-val is deprecated.  Use gumstix_memread_bma180-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vals-val :lambda-list '(m))
(cl:defmethod vals-val ((m <bma180meas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gumstix_memread_bma180-msg:vals-val is deprecated.  Use gumstix_memread_bma180-msg:vals instead.")
  (vals m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bma180meas>) ostream)
  "Serializes a message object of type '<bma180meas>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vals))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'vals))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bma180meas>) istream)
  "Deserializes a message object of type '<bma180meas>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vals) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vals)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bma180meas>)))
  "Returns string type for a message object of type '<bma180meas>"
  "gumstix_memread_bma180/bma180meas")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bma180meas)))
  "Returns string type for a message object of type 'bma180meas"
  "gumstix_memread_bma180/bma180meas")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bma180meas>)))
  "Returns md5sum for a message object of type '<bma180meas>"
  "77f3cd2c4156a9950a3ec747c76ba5b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bma180meas)))
  "Returns md5sum for a message object of type 'bma180meas"
  "77f3cd2c4156a9950a3ec747c76ba5b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bma180meas>)))
  "Returns full string definition for message of type '<bma180meas>"
  (cl:format cl:nil "Header header~%int16[] vals~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bma180meas)))
  "Returns full string definition for message of type 'bma180meas"
  (cl:format cl:nil "Header header~%int16[] vals~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bma180meas>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vals) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bma180meas>))
  "Converts a ROS message object to a list"
  (cl:list 'bma180meas
    (cl:cons ':header (header msg))
    (cl:cons ':vals (vals msg))
))
