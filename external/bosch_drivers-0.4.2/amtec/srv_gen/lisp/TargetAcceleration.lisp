; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude TargetAcceleration-request.msg.html

(cl:defclass <TargetAcceleration-request> (roslisp-msg-protocol:ros-message)
  ((acceleration_pan
    :reader acceleration_pan
    :initarg :acceleration_pan
    :type cl:float
    :initform 0.0)
   (acceleration_tilt
    :reader acceleration_tilt
    :initarg :acceleration_tilt
    :type cl:float
    :initform 0.0))
)

(cl:defclass TargetAcceleration-request (<TargetAcceleration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetAcceleration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetAcceleration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<TargetAcceleration-request> is deprecated: use amtec-srv:TargetAcceleration-request instead.")))

(cl:ensure-generic-function 'acceleration_pan-val :lambda-list '(m))
(cl:defmethod acceleration_pan-val ((m <TargetAcceleration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:acceleration_pan-val is deprecated.  Use amtec-srv:acceleration_pan instead.")
  (acceleration_pan m))

(cl:ensure-generic-function 'acceleration_tilt-val :lambda-list '(m))
(cl:defmethod acceleration_tilt-val ((m <TargetAcceleration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:acceleration_tilt-val is deprecated.  Use amtec-srv:acceleration_tilt instead.")
  (acceleration_tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetAcceleration-request>) ostream)
  "Serializes a message object of type '<TargetAcceleration-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acceleration_pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acceleration_tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetAcceleration-request>) istream)
  "Deserializes a message object of type '<TargetAcceleration-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration_pan) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration_tilt) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetAcceleration-request>)))
  "Returns string type for a service object of type '<TargetAcceleration-request>"
  "amtec/TargetAccelerationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetAcceleration-request)))
  "Returns string type for a service object of type 'TargetAcceleration-request"
  "amtec/TargetAccelerationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetAcceleration-request>)))
  "Returns md5sum for a message object of type '<TargetAcceleration-request>"
  "d342e69bfc77177d150eaee5d0f71f06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetAcceleration-request)))
  "Returns md5sum for a message object of type 'TargetAcceleration-request"
  "d342e69bfc77177d150eaee5d0f71f06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetAcceleration-request>)))
  "Returns full string definition for message of type '<TargetAcceleration-request>"
  (cl:format cl:nil "float64 acceleration_pan~%float64 acceleration_tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetAcceleration-request)))
  "Returns full string definition for message of type 'TargetAcceleration-request"
  (cl:format cl:nil "float64 acceleration_pan~%float64 acceleration_tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetAcceleration-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetAcceleration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetAcceleration-request
    (cl:cons ':acceleration_pan (acceleration_pan msg))
    (cl:cons ':acceleration_tilt (acceleration_tilt msg))
))
;//! \htmlinclude TargetAcceleration-response.msg.html

(cl:defclass <TargetAcceleration-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TargetAcceleration-response (<TargetAcceleration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetAcceleration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetAcceleration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<TargetAcceleration-response> is deprecated: use amtec-srv:TargetAcceleration-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetAcceleration-response>) ostream)
  "Serializes a message object of type '<TargetAcceleration-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetAcceleration-response>) istream)
  "Deserializes a message object of type '<TargetAcceleration-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetAcceleration-response>)))
  "Returns string type for a service object of type '<TargetAcceleration-response>"
  "amtec/TargetAccelerationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetAcceleration-response)))
  "Returns string type for a service object of type 'TargetAcceleration-response"
  "amtec/TargetAccelerationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetAcceleration-response>)))
  "Returns md5sum for a message object of type '<TargetAcceleration-response>"
  "d342e69bfc77177d150eaee5d0f71f06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetAcceleration-response)))
  "Returns md5sum for a message object of type 'TargetAcceleration-response"
  "d342e69bfc77177d150eaee5d0f71f06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetAcceleration-response>)))
  "Returns full string definition for message of type '<TargetAcceleration-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetAcceleration-response)))
  "Returns full string definition for message of type 'TargetAcceleration-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetAcceleration-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetAcceleration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetAcceleration-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TargetAcceleration)))
  'TargetAcceleration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TargetAcceleration)))
  'TargetAcceleration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetAcceleration)))
  "Returns string type for a service object of type '<TargetAcceleration>"
  "amtec/TargetAcceleration")