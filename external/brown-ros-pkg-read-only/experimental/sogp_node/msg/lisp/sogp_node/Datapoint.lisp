; Auto-generated. Do not edit!


(in-package sogp_node-msg)


;//! \htmlinclude Datapoint.msg.html

(defclass <Datapoint> (ros-message)
  ((data
    :accessor data-val
    :initarg :data
    :initform (make-array 0 :initial-element 0.0)))
)
(defmethod serialize ((msg <Datapoint>) ostream)
  "Serializes a message object of type '<Datapoint>"
  (let ((__ros_arr_len (length (slot-value msg 'data))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))
    (slot-value msg 'data))
)
(defmethod deserialize ((msg <Datapoint>) istream)
  "Deserializes a message object of type '<Datapoint>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'data) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'data)))
      (dotimes (i __ros_arr_len)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Datapoint>)))
  "Returns string type for a message object of type '<Datapoint>"
  "sogp_node/Datapoint")
(defmethod md5sum ((type (eql '<Datapoint>)))
  "Returns md5sum for a message object of type '<Datapoint>"
  #x420cd38b6b071cd49f2970c3e2cee511)
(defmethod message-definition ((type (eql '<Datapoint>)))
  "Returns full string definition for message of type '<Datapoint>"
  (format nil "float32[] data~%~%~%"))
(defmethod serialization-length ((msg <Datapoint>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'data) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <Datapoint>))
  "Converts a ROS message object to a list"
  (list '<Datapoint>
    (cons ':data (ros-message-to-list (data-val msg)))
))
