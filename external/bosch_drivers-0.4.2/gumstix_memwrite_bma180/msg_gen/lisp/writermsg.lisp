; Auto-generated. Do not edit!


(cl:in-package gumstix_memwrite_bma180-msg)


;//! \htmlinclude writermsg.msg.html

(cl:defclass <writermsg> (roslisp-msg-protocol:ros-message)
  ((written
    :reader written
    :initarg :written
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass writermsg (<writermsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <writermsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'writermsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gumstix_memwrite_bma180-msg:<writermsg> is deprecated: use gumstix_memwrite_bma180-msg:writermsg instead.")))

(cl:ensure-generic-function 'written-val :lambda-list '(m))
(cl:defmethod written-val ((m <writermsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gumstix_memwrite_bma180-msg:written-val is deprecated.  Use gumstix_memwrite_bma180-msg:written instead.")
  (written m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <writermsg>) ostream)
  "Serializes a message object of type '<writermsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'written) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <writermsg>) istream)
  "Deserializes a message object of type '<writermsg>"
    (cl:setf (cl:slot-value msg 'written) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<writermsg>)))
  "Returns string type for a message object of type '<writermsg>"
  "gumstix_memwrite_bma180/writermsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'writermsg)))
  "Returns string type for a message object of type 'writermsg"
  "gumstix_memwrite_bma180/writermsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<writermsg>)))
  "Returns md5sum for a message object of type '<writermsg>"
  "b709114f9799cee160e24092334f8514")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'writermsg)))
  "Returns md5sum for a message object of type 'writermsg"
  "b709114f9799cee160e24092334f8514")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<writermsg>)))
  "Returns full string definition for message of type '<writermsg>"
  (cl:format cl:nil "bool written~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'writermsg)))
  "Returns full string definition for message of type 'writermsg"
  (cl:format cl:nil "bool written~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <writermsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <writermsg>))
  "Converts a ROS message object to a list"
  (cl:list 'writermsg
    (cl:cons ':written (written msg))
))
