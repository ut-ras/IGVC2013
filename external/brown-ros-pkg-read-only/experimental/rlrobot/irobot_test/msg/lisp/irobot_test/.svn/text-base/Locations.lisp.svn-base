; Auto-generated. Do not edit!


(in-package irobot_test-msg)


;//! \htmlinclude Locations.msg.html

(defclass <Locations> (ros-message)
  ((objectlist
    :reader objectlist-val
    :initarg :objectlist
    :type (vector <LocalizableObject>)
   :initform (make-array 0 :element-type '<LocalizableObject> :initial-element (make-instance '<LocalizableObject>))))
)
(defmethod serialize ((msg <Locations>) ostream)
  "Serializes a message object of type '<Locations>"
  (let ((__ros_arr_len (length (slot-value msg 'objectlist))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'objectlist))
)
(defmethod deserialize ((msg <Locations>) istream)
  "Deserializes a message object of type '<Locations>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'objectlist) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'objectlist)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance '<LocalizableObject>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Locations>)))
  "Returns string type for a message object of type '<Locations>"
  "irobot_test/Locations")
(defmethod md5sum ((type (eql '<Locations>)))
  "Returns md5sum for a message object of type '<Locations>"
  "d54a280f7da3551346c2e82e4298dc42")
(defmethod message-definition ((type (eql '<Locations>)))
  "Returns full string definition for message of type '<Locations>"
  (format nil "LocalizableObject[] objectlist~%~%================================================================================~%MSG: irobot_test/LocalizableObject~%uint32 objecttype~%uint32 objectid~%float64 posx~%float64 posy~%float64 yaw~%~%~%"))
(defmethod serialization-length ((msg <Locations>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'objectlist) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <Locations>))
  "Converts a ROS message object to a list"
  (list '<Locations>
    (cons ':objectlist (objectlist-val msg))
))
