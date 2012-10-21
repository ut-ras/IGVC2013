; Auto-generated. Do not edit!


(cl:in-package adc_sub20-msg)


;//! \htmlinclude sub20_ADC_err.msg.html

(cl:defclass <sub20_ADC_err> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (strErrMessage
    :reader strErrMessage
    :initarg :strErrMessage
    :type cl:string
    :initform ""))
)

(cl:defclass sub20_ADC_err (<sub20_ADC_err>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sub20_ADC_err>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sub20_ADC_err)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name adc_sub20-msg:<sub20_ADC_err> is deprecated: use adc_sub20-msg:sub20_ADC_err instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <sub20_ADC_err>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_sub20-msg:header-val is deprecated.  Use adc_sub20-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'strErrMessage-val :lambda-list '(m))
(cl:defmethod strErrMessage-val ((m <sub20_ADC_err>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_sub20-msg:strErrMessage-val is deprecated.  Use adc_sub20-msg:strErrMessage instead.")
  (strErrMessage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sub20_ADC_err>) ostream)
  "Serializes a message object of type '<sub20_ADC_err>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'strErrMessage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'strErrMessage))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sub20_ADC_err>) istream)
  "Deserializes a message object of type '<sub20_ADC_err>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'strErrMessage) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'strErrMessage) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sub20_ADC_err>)))
  "Returns string type for a message object of type '<sub20_ADC_err>"
  "adc_sub20/sub20_ADC_err")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sub20_ADC_err)))
  "Returns string type for a message object of type 'sub20_ADC_err"
  "adc_sub20/sub20_ADC_err")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sub20_ADC_err>)))
  "Returns md5sum for a message object of type '<sub20_ADC_err>"
  "e22867bb71c25a46586924f4cb488c56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sub20_ADC_err)))
  "Returns md5sum for a message object of type 'sub20_ADC_err"
  "e22867bb71c25a46586924f4cb488c56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sub20_ADC_err>)))
  "Returns full string definition for message of type '<sub20_ADC_err>"
  (cl:format cl:nil "Header header~%string strErrMessage~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sub20_ADC_err)))
  "Returns full string definition for message of type 'sub20_ADC_err"
  (cl:format cl:nil "Header header~%string strErrMessage~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sub20_ADC_err>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'strErrMessage))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sub20_ADC_err>))
  "Converts a ROS message object to a list"
  (cl:list 'sub20_ADC_err
    (cl:cons ':header (header msg))
    (cl:cons ':strErrMessage (strErrMessage msg))
))
