; Auto-generated. Do not edit!


(cl:in-package PSoC_Listener-msg)


;//! \htmlinclude EncoderVelocity.msg.html

(cl:defclass <EncoderVelocity> (roslisp-msg-protocol:ros-message)
  ((v
    :reader v
    :initarg :v
    :type cl:fixnum
    :initform 0)
   (w
    :reader w
    :initarg :w
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EncoderVelocity (<EncoderVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncoderVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncoderVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name PSoC_Listener-msg:<EncoderVelocity> is deprecated: use PSoC_Listener-msg:EncoderVelocity instead.")))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <EncoderVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader PSoC_Listener-msg:v-val is deprecated.  Use PSoC_Listener-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <EncoderVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader PSoC_Listener-msg:w-val is deprecated.  Use PSoC_Listener-msg:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncoderVelocity>) ostream)
  "Serializes a message object of type '<EncoderVelocity>"
  (cl:let* ((signed (cl:slot-value msg 'v)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'w)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncoderVelocity>) istream)
  "Deserializes a message object of type '<EncoderVelocity>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'v) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'w) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncoderVelocity>)))
  "Returns string type for a message object of type '<EncoderVelocity>"
  "PSoC_Listener/EncoderVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncoderVelocity)))
  "Returns string type for a message object of type 'EncoderVelocity"
  "PSoC_Listener/EncoderVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncoderVelocity>)))
  "Returns md5sum for a message object of type '<EncoderVelocity>"
  "ecb7d5299516eb7b426c83bc2bf91b44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncoderVelocity)))
  "Returns md5sum for a message object of type 'EncoderVelocity"
  "ecb7d5299516eb7b426c83bc2bf91b44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncoderVelocity>)))
  "Returns full string definition for message of type '<EncoderVelocity>"
  (cl:format cl:nil "int16 v~%int16 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncoderVelocity)))
  "Returns full string definition for message of type 'EncoderVelocity"
  (cl:format cl:nil "int16 v~%int16 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncoderVelocity>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncoderVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'EncoderVelocity
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
