; Auto-generated. Do not edit!


(in-package irobot_test-msg)


;//! \htmlinclude LocalizableObject.msg.html

(defclass <LocalizableObject> (ros-message)
  ((objecttype
    :reader objecttype-val
    :initarg :objecttype
    :type integer
    :initform 0)
   (objectid
    :reader objectid-val
    :initarg :objectid
    :type integer
    :initform 0)
   (posx
    :reader posx-val
    :initarg :posx
    :type float
    :initform 0.0)
   (posy
    :reader posy-val
    :initarg :posy
    :type float
    :initform 0.0)
   (yaw
    :reader yaw-val
    :initarg :yaw
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <LocalizableObject>) ostream)
  "Serializes a message object of type '<LocalizableObject>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'objecttype)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'objecttype)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'objecttype)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'objecttype)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'objectid)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'objectid)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'objectid)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'objectid)) ostream)
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'posx))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'posy))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'yaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <LocalizableObject>) istream)
  "Deserializes a message object of type '<LocalizableObject>"
  (setf (ldb (byte 8 0) (slot-value msg 'objecttype)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'objecttype)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'objecttype)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'objecttype)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'objectid)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'objectid)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'objectid)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'objectid)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'posx) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'posy) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<LocalizableObject>)))
  "Returns string type for a message object of type '<LocalizableObject>"
  "irobot_test/LocalizableObject")
(defmethod md5sum ((type (eql '<LocalizableObject>)))
  "Returns md5sum for a message object of type '<LocalizableObject>"
  "7d1f0aa2894b6168a0ed8aeafb7bd204")
(defmethod message-definition ((type (eql '<LocalizableObject>)))
  "Returns full string definition for message of type '<LocalizableObject>"
  (format nil "uint32 objecttype~%uint32 objectid~%float64 posx~%float64 posy~%float64 yaw~%~%~%"))
(defmethod serialization-length ((msg <LocalizableObject>))
  (+ 0
     4
     4
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <LocalizableObject>))
  "Converts a ROS message object to a list"
  (list '<LocalizableObject>
    (cons ':objecttype (objecttype-val msg))
    (cons ':objectid (objectid-val msg))
    (cons ':posx (posx-val msg))
    (cons ':posy (posy-val msg))
    (cons ':yaw (yaw-val msg))
))
