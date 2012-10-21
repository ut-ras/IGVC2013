; Auto-generated. Do not edit!


(cl:in-package photo-srv)


;//! \htmlinclude GetConfig-request.msg.html

(cl:defclass <GetConfig-request> (roslisp-msg-protocol:ros-message)
  ((param
    :reader param
    :initarg :param
    :type cl:string
    :initform ""))
)

(cl:defclass GetConfig-request (<GetConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name photo-srv:<GetConfig-request> is deprecated: use photo-srv:GetConfig-request instead.")))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <GetConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader photo-srv:param-val is deprecated.  Use photo-srv:param instead.")
  (param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConfig-request>) ostream)
  "Serializes a message object of type '<GetConfig-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'param))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConfig-request>) istream)
  "Deserializes a message object of type '<GetConfig-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'param) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'param) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConfig-request>)))
  "Returns string type for a service object of type '<GetConfig-request>"
  "photo/GetConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfig-request)))
  "Returns string type for a service object of type 'GetConfig-request"
  "photo/GetConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConfig-request>)))
  "Returns md5sum for a message object of type '<GetConfig-request>"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConfig-request)))
  "Returns md5sum for a message object of type 'GetConfig-request"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConfig-request>)))
  "Returns full string definition for message of type '<GetConfig-request>"
  (cl:format cl:nil "string param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConfig-request)))
  "Returns full string definition for message of type 'GetConfig-request"
  (cl:format cl:nil "string param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConfig-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'param))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConfig-request
    (cl:cons ':param (param msg))
))
;//! \htmlinclude GetConfig-response.msg.html

(cl:defclass <GetConfig-response> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass GetConfig-response (<GetConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name photo-srv:<GetConfig-response> is deprecated: use photo-srv:GetConfig-response instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <GetConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader photo-srv:value-val is deprecated.  Use photo-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConfig-response>) ostream)
  "Serializes a message object of type '<GetConfig-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConfig-response>) istream)
  "Deserializes a message object of type '<GetConfig-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConfig-response>)))
  "Returns string type for a service object of type '<GetConfig-response>"
  "photo/GetConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfig-response)))
  "Returns string type for a service object of type 'GetConfig-response"
  "photo/GetConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConfig-response>)))
  "Returns md5sum for a message object of type '<GetConfig-response>"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConfig-response)))
  "Returns md5sum for a message object of type 'GetConfig-response"
  "2017f3298983627814c079c3b10ca05d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConfig-response>)))
  "Returns full string definition for message of type '<GetConfig-response>"
  (cl:format cl:nil "string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConfig-response)))
  "Returns full string definition for message of type 'GetConfig-response"
  (cl:format cl:nil "string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConfig-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConfig-response
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetConfig)))
  'GetConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetConfig)))
  'GetConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfig)))
  "Returns string type for a service object of type '<GetConfig>"
  "photo/GetConfig")