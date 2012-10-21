; Auto-generated. Do not edit!


(cl:in-package photo-srv)


;//! \htmlinclude SetConfig-request.msg.html

(cl:defclass <SetConfig-request> (roslisp-msg-protocol:ros-message)
  ((param
    :reader param
    :initarg :param
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass SetConfig-request (<SetConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name photo-srv:<SetConfig-request> is deprecated: use photo-srv:SetConfig-request instead.")))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <SetConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader photo-srv:param-val is deprecated.  Use photo-srv:param instead.")
  (param m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader photo-srv:value-val is deprecated.  Use photo-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetConfig-request>) ostream)
  "Serializes a message object of type '<SetConfig-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'param))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetConfig-request>) istream)
  "Deserializes a message object of type '<SetConfig-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'param) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetConfig-request>)))
  "Returns string type for a service object of type '<SetConfig-request>"
  "photo/SetConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetConfig-request)))
  "Returns string type for a service object of type 'SetConfig-request"
  "photo/SetConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetConfig-request>)))
  "Returns md5sum for a message object of type '<SetConfig-request>"
  "6f5acdb9088557703e392df3355dcb2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetConfig-request)))
  "Returns md5sum for a message object of type 'SetConfig-request"
  "6f5acdb9088557703e392df3355dcb2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetConfig-request>)))
  "Returns full string definition for message of type '<SetConfig-request>"
  (cl:format cl:nil "string param~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetConfig-request)))
  "Returns full string definition for message of type 'SetConfig-request"
  (cl:format cl:nil "string param~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetConfig-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'param))
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetConfig-request
    (cl:cons ':param (param msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetConfig-response.msg.html

(cl:defclass <SetConfig-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetConfig-response (<SetConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name photo-srv:<SetConfig-response> is deprecated: use photo-srv:SetConfig-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetConfig-response>) ostream)
  "Serializes a message object of type '<SetConfig-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetConfig-response>) istream)
  "Deserializes a message object of type '<SetConfig-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetConfig-response>)))
  "Returns string type for a service object of type '<SetConfig-response>"
  "photo/SetConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetConfig-response)))
  "Returns string type for a service object of type 'SetConfig-response"
  "photo/SetConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetConfig-response>)))
  "Returns md5sum for a message object of type '<SetConfig-response>"
  "6f5acdb9088557703e392df3355dcb2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetConfig-response)))
  "Returns md5sum for a message object of type 'SetConfig-response"
  "6f5acdb9088557703e392df3355dcb2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetConfig-response>)))
  "Returns full string definition for message of type '<SetConfig-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetConfig-response)))
  "Returns full string definition for message of type 'SetConfig-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetConfig-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetConfig-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetConfig)))
  'SetConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetConfig)))
  'SetConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetConfig)))
  "Returns string type for a service object of type '<SetConfig>"
  "photo/SetConfig")