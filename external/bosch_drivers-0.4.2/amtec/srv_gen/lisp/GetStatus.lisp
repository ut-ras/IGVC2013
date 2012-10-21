; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude GetStatus-request.msg.html

(cl:defclass <GetStatus-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetStatus-request (<GetStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<GetStatus-request> is deprecated: use amtec-srv:GetStatus-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStatus-request>) ostream)
  "Serializes a message object of type '<GetStatus-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStatus-request>) istream)
  "Deserializes a message object of type '<GetStatus-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStatus-request>)))
  "Returns string type for a service object of type '<GetStatus-request>"
  "amtec/GetStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStatus-request)))
  "Returns string type for a service object of type 'GetStatus-request"
  "amtec/GetStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStatus-request>)))
  "Returns md5sum for a message object of type '<GetStatus-request>"
  "1faf4b4c83e1e50040afc67afdd423b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStatus-request)))
  "Returns md5sum for a message object of type 'GetStatus-request"
  "1faf4b4c83e1e50040afc67afdd423b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStatus-request>)))
  "Returns full string definition for message of type '<GetStatus-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStatus-request)))
  "Returns full string definition for message of type 'GetStatus-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStatus-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStatus-request
))
;//! \htmlinclude GetStatus-response.msg.html

(cl:defclass <GetStatus-response> (roslisp-msg-protocol:ros-message)
  ((position_pan
    :reader position_pan
    :initarg :position_pan
    :type cl:float
    :initform 0.0)
   (position_tilt
    :reader position_tilt
    :initarg :position_tilt
    :type cl:float
    :initform 0.0)
   (velocity_pan
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

(cl:defclass GetStatus-response (<GetStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<GetStatus-response> is deprecated: use amtec-srv:GetStatus-response instead.")))

(cl:ensure-generic-function 'position_pan-val :lambda-list '(m))
(cl:defmethod position_pan-val ((m <GetStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:position_pan-val is deprecated.  Use amtec-srv:position_pan instead.")
  (position_pan m))

(cl:ensure-generic-function 'position_tilt-val :lambda-list '(m))
(cl:defmethod position_tilt-val ((m <GetStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:position_tilt-val is deprecated.  Use amtec-srv:position_tilt instead.")
  (position_tilt m))

(cl:ensure-generic-function 'velocity_pan-val :lambda-list '(m))
(cl:defmethod velocity_pan-val ((m <GetStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:velocity_pan-val is deprecated.  Use amtec-srv:velocity_pan instead.")
  (velocity_pan m))

(cl:ensure-generic-function 'velocity_tilt-val :lambda-list '(m))
(cl:defmethod velocity_tilt-val ((m <GetStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:velocity_tilt-val is deprecated.  Use amtec-srv:velocity_tilt instead.")
  (velocity_tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetStatus-response>) ostream)
  "Serializes a message object of type '<GetStatus-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position_pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position_tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetStatus-response>) istream)
  "Deserializes a message object of type '<GetStatus-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position_pan) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position_tilt) (roslisp-utils:decode-double-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetStatus-response>)))
  "Returns string type for a service object of type '<GetStatus-response>"
  "amtec/GetStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStatus-response)))
  "Returns string type for a service object of type 'GetStatus-response"
  "amtec/GetStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetStatus-response>)))
  "Returns md5sum for a message object of type '<GetStatus-response>"
  "1faf4b4c83e1e50040afc67afdd423b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetStatus-response)))
  "Returns md5sum for a message object of type 'GetStatus-response"
  "1faf4b4c83e1e50040afc67afdd423b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetStatus-response>)))
  "Returns full string definition for message of type '<GetStatus-response>"
  (cl:format cl:nil "float64 position_pan~%float64 position_tilt~%float64 velocity_pan~%float64 velocity_tilt~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetStatus-response)))
  "Returns full string definition for message of type 'GetStatus-response"
  (cl:format cl:nil "float64 position_pan~%float64 position_tilt~%float64 velocity_pan~%float64 velocity_tilt~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetStatus-response>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetStatus-response
    (cl:cons ':position_pan (position_pan msg))
    (cl:cons ':position_tilt (position_tilt msg))
    (cl:cons ':velocity_pan (velocity_pan msg))
    (cl:cons ':velocity_tilt (velocity_tilt msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetStatus)))
  'GetStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetStatus)))
  'GetStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetStatus)))
  "Returns string type for a service object of type '<GetStatus>"
  "amtec/GetStatus")