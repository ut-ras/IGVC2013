; Auto-generated. Do not edit!


(cl:in-package amtec-srv)


;//! \htmlinclude SweepPan-request.msg.html

(cl:defclass <SweepPan-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SweepPan-request (<SweepPan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SweepPan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SweepPan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<SweepPan-request> is deprecated: use amtec-srv:SweepPan-request instead.")))

(cl:ensure-generic-function 'sweep_amplitude-val :lambda-list '(m))
(cl:defmethod sweep_amplitude-val ((m <SweepPan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:sweep_amplitude-val is deprecated.  Use amtec-srv:sweep_amplitude instead.")
  (sweep_amplitude m))

(cl:ensure-generic-function 'sweep_period-val :lambda-list '(m))
(cl:defmethod sweep_period-val ((m <SweepPan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader amtec-srv:sweep_period-val is deprecated.  Use amtec-srv:sweep_period instead.")
  (sweep_period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SweepPan-request>) ostream)
  "Serializes a message object of type '<SweepPan-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SweepPan-request>) istream)
  "Deserializes a message object of type '<SweepPan-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SweepPan-request>)))
  "Returns string type for a service object of type '<SweepPan-request>"
  "amtec/SweepPanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SweepPan-request)))
  "Returns string type for a service object of type 'SweepPan-request"
  "amtec/SweepPanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SweepPan-request>)))
  "Returns md5sum for a message object of type '<SweepPan-request>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SweepPan-request)))
  "Returns md5sum for a message object of type 'SweepPan-request"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SweepPan-request>)))
  "Returns full string definition for message of type '<SweepPan-request>"
  (cl:format cl:nil "float64 sweep_amplitude~%float64 sweep_period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SweepPan-request)))
  "Returns full string definition for message of type 'SweepPan-request"
  (cl:format cl:nil "float64 sweep_amplitude~%float64 sweep_period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SweepPan-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SweepPan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SweepPan-request
    (cl:cons ':sweep_amplitude (sweep_amplitude msg))
    (cl:cons ':sweep_period (sweep_period msg))
))
;//! \htmlinclude SweepPan-response.msg.html

(cl:defclass <SweepPan-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SweepPan-response (<SweepPan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SweepPan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SweepPan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name amtec-srv:<SweepPan-response> is deprecated: use amtec-srv:SweepPan-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SweepPan-response>) ostream)
  "Serializes a message object of type '<SweepPan-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SweepPan-response>) istream)
  "Deserializes a message object of type '<SweepPan-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SweepPan-response>)))
  "Returns string type for a service object of type '<SweepPan-response>"
  "amtec/SweepPanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SweepPan-response)))
  "Returns string type for a service object of type 'SweepPan-response"
  "amtec/SweepPanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SweepPan-response>)))
  "Returns md5sum for a message object of type '<SweepPan-response>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SweepPan-response)))
  "Returns md5sum for a message object of type 'SweepPan-response"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SweepPan-response>)))
  "Returns full string definition for message of type '<SweepPan-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SweepPan-response)))
  "Returns full string definition for message of type 'SweepPan-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SweepPan-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SweepPan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SweepPan-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SweepPan)))
  'SweepPan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SweepPan)))
  'SweepPan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SweepPan)))
  "Returns string type for a service object of type '<SweepPan>"
  "amtec/SweepPan")