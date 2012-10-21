; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude SetPosition-request.msg.html

(cl:defclass <SetPosition-request> (roslisp-msg-protocol:ros-message)
  ((position_pan
    :reader position_pan
    :initarg :position_pan
    :type cl:float
    :initform 0.0)
   (position_tilt
    :reader position_tilt
    :initarg :position_tilt
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPosition-request (<SetPosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<SetPosition-request> is deprecated: use amtec-srv:SetPosition-request instead.")))

(cl:ensure-generic-function 'position_pan-val :lambda-list '(m))
(cl:defmethod position_pan-val ((m <SetPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:position_pan-val is deprecated.  Use amtec-srv:position_pan instead.")
  (position_pan m))

(cl:ensure-generic-function 'position_tilt-val :lambda-list '(m))
(cl:defmethod position_tilt-val ((m <SetPosition-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:position_tilt-val is deprecated.  Use amtec-srv:position_tilt instead.")
  (position_tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPosition-request>) ostream)
  "Serializes a message object of type '<SetPosition-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPosition-request>) istream)
  "Deserializes a message object of type '<SetPosition-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPosition-request>)))
  "Returns string type for a service object of type '<SetPosition-request>"
  "amtec/SetPositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPosition-request)))
  "Returns string type for a service object of type 'SetPosition-request"
  "amtec/SetPositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPosition-request>)))
  "Returns md5sum for a message object of type '<SetPosition-request>"
  "cefac8c7f126ab41b3e46de351ef0cea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPosition-request)))
  "Returns md5sum for a message object of type 'SetPosition-request"
  "cefac8c7f126ab41b3e46de351ef0cea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPosition-request>)))
  "Returns full string definition for message of type '<SetPosition-request>"
  (cl:format cl:nil "float64 position_pan~%float64 position_tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPosition-request)))
  "Returns full string definition for message of type 'SetPosition-request"
  (cl:format cl:nil "float64 position_pan~%float64 position_tilt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPosition-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPosition-request
    (cl:cons ':position_pan (position_pan msg))
    (cl:cons ':position_tilt (position_tilt msg))
))
;//! \htmlinclude SetPosition-response.msg.html

(cl:defclass <SetPosition-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetPosition-response (<SetPosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<SetPosition-response> is deprecated: use amtec-srv:SetPosition-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPosition-response>) ostream)
  "Serializes a message object of type '<SetPosition-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPosition-response>) istream)
  "Deserializes a message object of type '<SetPosition-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPosition-response>)))
  "Returns string type for a service object of type '<SetPosition-response>"
  "amtec/SetPositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPosition-response)))
  "Returns string type for a service object of type 'SetPosition-response"
  "amtec/SetPositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPosition-response>)))
  "Returns md5sum for a message object of type '<SetPosition-response>"
  "cefac8c7f126ab41b3e46de351ef0cea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPosition-response)))
  "Returns md5sum for a message object of type 'SetPosition-response"
  "cefac8c7f126ab41b3e46de351ef0cea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPosition-response>)))
  "Returns full string definition for message of type '<SetPosition-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPosition-response)))
  "Returns full string definition for message of type 'SetPosition-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPosition-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPosition-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPosition)))
  'SetPosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPosition)))
  'SetPosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPosition)))
  "Returns string type for a service object of type '<SetPosition>"
  "amtec/SetPosition")