; Auto-generated. Do not edit!


(in-package microcontroller_interface-msg)


;//! \htmlinclude Telemetry.msg.html

(defclass <Telemetry> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (command
    :reader command-val
    :initarg :command
    :type integer
    :initform 0)
   (actual
    :reader actual-val
    :initarg :actual
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Telemetry>) ostream)
  "Serializes a message object of type '<Telemetry>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'command)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'command)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'actual)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'actual)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'actual)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'actual)) ostream)
)
(defmethod deserialize ((msg <Telemetry>) istream)
  "Deserializes a message object of type '<Telemetry>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'command)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'actual)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'actual)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'actual)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'actual)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Telemetry>)))
  "Returns string type for a message object of type '<Telemetry>"
  "microcontroller_interface/Telemetry")
(defmethod md5sum ((type (eql '<Telemetry>)))
  "Returns md5sum for a message object of type '<Telemetry>"
  "099f23789666af6c5996c42d9aa81d25")
(defmethod message-definition ((type (eql '<Telemetry>)))
  "Returns full string definition for message of type '<Telemetry>"
  (format nil "Header header~%~%int32 command~%int32 actual~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Telemetry>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
))
(defmethod ros-message-to-list ((msg <Telemetry>))
  "Converts a ROS message object to a list"
  (list '<Telemetry>
    (cons ':header (header-val msg))
    (cons ':command (command-val msg))
    (cons ':actual (actual-val msg))
))
