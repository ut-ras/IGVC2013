; Auto-generated. Do not edit!


(cl:in-package adc_sub20-msg)


;//! \htmlinclude sub20_ADC_meas.msg.html

(cl:defclass <sub20_ADC_meas> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (strIdSubDev
    :reader strIdSubDev
    :initarg :strIdSubDev
    :type cl:string
    :initform "")
   (uiRaw
    :reader uiRaw
    :initarg :uiRaw
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (fVolts
    :reader fVolts
    :initarg :fVolts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass sub20_ADC_meas (<sub20_ADC_meas>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sub20_ADC_meas>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sub20_ADC_meas)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name adc_sub20-msg:<sub20_ADC_meas> is deprecated: use adc_sub20-msg:sub20_ADC_meas instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <sub20_ADC_meas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_sub20-msg:header-val is deprecated.  Use adc_sub20-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'strIdSubDev-val :lambda-list '(m))
(cl:defmethod strIdSubDev-val ((m <sub20_ADC_meas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_sub20-msg:strIdSubDev-val is deprecated.  Use adc_sub20-msg:strIdSubDev instead.")
  (strIdSubDev m))

(cl:ensure-generic-function 'uiRaw-val :lambda-list '(m))
(cl:defmethod uiRaw-val ((m <sub20_ADC_meas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_sub20-msg:uiRaw-val is deprecated.  Use adc_sub20-msg:uiRaw instead.")
  (uiRaw m))

(cl:ensure-generic-function 'fVolts-val :lambda-list '(m))
(cl:defmethod fVolts-val ((m <sub20_ADC_meas>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader adc_sub20-msg:fVolts-val is deprecated.  Use adc_sub20-msg:fVolts instead.")
  (fVolts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sub20_ADC_meas>) ostream)
  "Serializes a message object of type '<sub20_ADC_meas>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'strIdSubDev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'strIdSubDev))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'uiRaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'uiRaw))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fVolts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'fVolts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sub20_ADC_meas>) istream)
  "Deserializes a message object of type '<sub20_ADC_meas>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'strIdSubDev) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'strIdSubDev) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'uiRaw) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'uiRaw)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'fVolts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'fVolts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sub20_ADC_meas>)))
  "Returns string type for a message object of type '<sub20_ADC_meas>"
  "adc_sub20/sub20_ADC_meas")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sub20_ADC_meas)))
  "Returns string type for a message object of type 'sub20_ADC_meas"
  "adc_sub20/sub20_ADC_meas")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sub20_ADC_meas>)))
  "Returns md5sum for a message object of type '<sub20_ADC_meas>"
  "e5a875ce6bb2d936bd7c1ae33b9699ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sub20_ADC_meas)))
  "Returns md5sum for a message object of type 'sub20_ADC_meas"
  "e5a875ce6bb2d936bd7c1ae33b9699ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sub20_ADC_meas>)))
  "Returns full string definition for message of type '<sub20_ADC_meas>"
  (cl:format cl:nil "Header header~%string strIdSubDev~%uint16[] uiRaw~%float64[] fVolts~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sub20_ADC_meas)))
  "Returns full string definition for message of type 'sub20_ADC_meas"
  (cl:format cl:nil "Header header~%string strIdSubDev~%uint16[] uiRaw~%float64[] fVolts~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sub20_ADC_meas>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'strIdSubDev))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uiRaw) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fVolts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sub20_ADC_meas>))
  "Converts a ROS message object to a list"
  (cl:list 'sub20_ADC_meas
    (cl:cons ':header (header msg))
    (cl:cons ':strIdSubDev (strIdSubDev msg))
    (cl:cons ':uiRaw (uiRaw msg))
    (cl:cons ':fVolts (fVolts msg))
))
