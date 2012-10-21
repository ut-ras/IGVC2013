; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude TargetVelocity-request.msg.html

(cl:defclass <TargetVelocity-request> (roslisp-msg-protocol:ros-message)
  ((velocity_pan
    :reader velocity_pan
    :initarg :velocity_pan
    :type cl:float
    :initform 0.0)
   (velocity_tilt
    :reader velocity_tilt
    :initarg :velocity_tilt
    :type cl:float
    :initform 0.0))
)

(cl:defclass TargetVelocity-request (<TargetVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<TargetVelocity-request> is deprecated: use amtec-srv:TargetVelocity-request instead.")))

(cl:ensure-generic-function 'velocity_pan-val :lambda-list '(m))
(cl:defmethod velocity_pan-val ((m <TargetVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:velocity_pan-val is deprecated.  Use amtec-srv:velocity_pan instead.")
  (velocity_pan m))

(cl:ensure-generic-function 'velocity_tilt-val :lambda-list '(m))
(cl:defmethod velocity_tilt-val ((m <TargetVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:velocity_tilt-val is deprecated.  Use amtec-srv:velocity_tilt instead.")
  (velocity_tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetVelocity-request>) ostream)
  "Serializes a message object of type '<TargetVelocity-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity_pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity_tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetVelocity-request>) istream)
  "Deserializes a message object of type '<TargetVelocity-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_pan) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_tilt) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetVelocity-request>)))
  "Returns string type for a service object of type '<TargetVelocity-request>"
  "amtec/TargetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetVelocity-request)))
  "Returns string type for a service object of type 'TargetVelocity-request"
  "amtec/TargetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetVelocity-request>)))
  "Returns md5sum for a message object of type '<TargetVelocity-request>"
  "2adb301f21428b03215ce423fc357684")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetVelocity-request)))
  "Returns md5sum for a message object of type 'TargetVelocity-request"
  "2adb301f21428b03215ce423fc357684")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetVelocity-request>)))
  "Returns full string definition for message of type '<TargetVelocity-request>"
  (cl:format cl:nil "float64 velocity_pan~%float64 velocity_tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetVelocity-request)))
  "Returns full string definition for message of type 'TargetVelocity-request"
  (cl:format cl:nil "float64 velocity_pan~%float64 velocity_tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetVelocity-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetVelocity-request
    (cl:cons ':velocity_pan (velocity_pan msg))
    (cl:cons ':velocity_tilt (velocity_tilt msg))
))
;//! \htmlinclude TargetVelocity-response.msg.html

(cl:defclass <TargetVelocity-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TargetVelocity-response (<TargetVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<TargetVelocity-response> is deprecated: use amtec-srv:TargetVelocity-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetVelocity-response>) ostream)
  "Serializes a message object of type '<TargetVelocity-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetVelocity-response>) istream)
  "Deserializes a message object of type '<TargetVelocity-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetVelocity-response>)))
  "Returns string type for a service object of type '<TargetVelocity-response>"
  "amtec/TargetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetVelocity-response)))
  "Returns string type for a service object of type 'TargetVelocity-response"
  "amtec/TargetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetVelocity-response>)))
  "Returns md5sum for a message object of type '<TargetVelocity-response>"
  "2adb301f21428b03215ce423fc357684")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetVelocity-response)))
  "Returns md5sum for a message object of type 'TargetVelocity-response"
  "2adb301f21428b03215ce423fc357684")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetVelocity-response>)))
  "Returns full string definition for message of type '<TargetVelocity-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetVelocity-response)))
  "Returns full string definition for message of type 'TargetVelocity-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetVelocity-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetVelocity-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TargetVelocity)))
  'TargetVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TargetVelocity)))
  'TargetVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetVelocity)))
  "Returns string type for a service object of type '<TargetVelocity>"
  "amtec/TargetVelocity")