; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude SweepTilt-request.msg.html

(cl:defclass <SweepTilt-request> (roslisp-msg-protocol:ros-message)
  ((sweep_amplitude
    :reader sweep_amplitude
    :initarg :sweep_amplitude
    :type cl:float
    :initform 0.0)
   (sweep_period
    :reader sweep_period
    :initarg :sweep_period
    :type cl:float
    :initform 0.0))
)

(cl:defclass SweepTilt-request (<SweepTilt-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SweepTilt-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SweepTilt-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<SweepTilt-request> is deprecated: use amtec-srv:SweepTilt-request instead.")))

(cl:ensure-generic-function 'sweep_amplitude-val :lambda-list '(m))
(cl:defmethod sweep_amplitude-val ((m <SweepTilt-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:sweep_amplitude-val is deprecated.  Use amtec-srv:sweep_amplitude instead.")
  (sweep_amplitude m))

(cl:ensure-generic-function 'sweep_period-val :lambda-list '(m))
(cl:defmethod sweep_period-val ((m <SweepTilt-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:sweep_period-val is deprecated.  Use amtec-srv:sweep_period instead.")
  (sweep_period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SweepTilt-request>) ostream)
  "Serializes a message object of type '<SweepTilt-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sweep_amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sweep_period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SweepTilt-request>) istream)
  "Deserializes a message object of type '<SweepTilt-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sweep_amplitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sweep_period) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SweepTilt-request>)))
  "Returns string type for a service object of type '<SweepTilt-request>"
  "amtec/SweepTiltRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SweepTilt-request)))
  "Returns string type for a service object of type 'SweepTilt-request"
  "amtec/SweepTiltRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SweepTilt-request>)))
  "Returns md5sum for a message object of type '<SweepTilt-request>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SweepTilt-request)))
  "Returns md5sum for a message object of type 'SweepTilt-request"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SweepTilt-request>)))
  "Returns full string definition for message of type '<SweepTilt-request>"
  (cl:format cl:nil "float64 sweep_amplitude~%float64 sweep_period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SweepTilt-request)))
  "Returns full string definition for message of type 'SweepTilt-request"
  (cl:format cl:nil "float64 sweep_amplitude~%float64 sweep_period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SweepTilt-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SweepTilt-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SweepTilt-request
    (cl:cons ':sweep_amplitude (sweep_amplitude msg))
    (cl:cons ':sweep_period (sweep_period msg))
))
;//! \htmlinclude SweepTilt-response.msg.html

(cl:defclass <SweepTilt-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SweepTilt-response (<SweepTilt-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SweepTilt-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SweepTilt-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<SweepTilt-response> is deprecated: use amtec-srv:SweepTilt-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SweepTilt-response>) ostream)
  "Serializes a message object of type '<SweepTilt-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SweepTilt-response>) istream)
  "Deserializes a message object of type '<SweepTilt-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SweepTilt-response>)))
  "Returns string type for a service object of type '<SweepTilt-response>"
  "amtec/SweepTiltResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SweepTilt-response)))
  "Returns string type for a service object of type 'SweepTilt-response"
  "amtec/SweepTiltResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SweepTilt-response>)))
  "Returns md5sum for a message object of type '<SweepTilt-response>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SweepTilt-response)))
  "Returns md5sum for a message object of type 'SweepTilt-response"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SweepTilt-response>)))
  "Returns full string definition for message of type '<SweepTilt-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SweepTilt-response)))
  "Returns full string definition for message of type 'SweepTilt-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SweepTilt-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SweepTilt-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SweepTilt-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SweepTilt)))
  'SweepTilt-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SweepTilt)))
  'SweepTilt-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SweepTilt)))
  "Returns string type for a service object of type '<SweepTilt>"
  "amtec/SweepTilt")