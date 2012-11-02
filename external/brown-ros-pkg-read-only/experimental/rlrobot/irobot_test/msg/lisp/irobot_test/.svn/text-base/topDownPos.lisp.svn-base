; Auto-generated. Do not edit!


(in-package irobot_test-msg)


;//! \htmlinclude topDownPos.msg.html

(defclass <topDownPos> (ros-message)
  ((posx
    :accessor posx-val
    :initarg :posx
    :initform 0.0)
   (posy
    :accessor posy-val
    :initarg :posy
    :initform 0.0)
   (yaw
    :accessor yaw-val
    :initarg :yaw
    :initform 0.0))
)
(defmethod serialize ((msg <topDownPos>) ostream)
  "Serializes a message object of type '<topDownPos>"
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'posx))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'posy))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <topDownPos>) istream)
  "Deserializes a message object of type '<topDownPos>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'posx) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'posy) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<topDownPos>)))
  "Returns string type for a message object of type '<topDownPos>"
  "irobot_test/topDownPos")
(defmethod md5sum ((type (eql '<topDownPos>)))
  "Returns md5sum for a message object of type '<topDownPos>"
  #x7f8be2666b5b18a1db5ae6e18b2a0607)
(defmethod message-definition ((type (eql '<topDownPos>)))
  "Returns full string definition for message of type '<topDownPos>"
  (format nil "float32 posx~%float32 posy~%float32 yaw~%~%~%"))
(defmethod serialization-length ((msg <topDownPos>))
  (+ 0
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <topDownPos>))
  "Converts a ROS message object to a list"
  (list '<topDownPos>
    (cons ':posx (ros-message-to-list (posx-val msg)))
    (cons ':posy (ros-message-to-list (posy-val msg)))
    (cons ':yaw (ros-message-to-list (yaw-val msg)))
))
