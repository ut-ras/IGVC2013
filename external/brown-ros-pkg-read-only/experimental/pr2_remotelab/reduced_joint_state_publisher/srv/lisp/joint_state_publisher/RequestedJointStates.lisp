; Auto-generated. Do not edit!


(in-package joint_state_publisher-srv)


;//! \htmlinclude RequestedJointStates-request.msg.html

(defclass <RequestedJointStates-request> (ros-message)
  ((names
    :reader names-val
    :initarg :names
    :type (vector string)
   :initform (make-array 0 :element-type 'string :initial-element "")))
)
(defmethod serialize ((msg <RequestedJointStates-request>) ostream)
  "Serializes a message object of type '<RequestedJointStates-request>"
  (let ((__ros_arr_len (length (slot-value msg 'names))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((__ros_str_len (length ele)))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) ele))
    (slot-value msg 'names))
)
(defmethod deserialize ((msg <RequestedJointStates-request>) istream)
  "Deserializes a message object of type '<RequestedJointStates-request>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'names) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'names)))
      (dotimes (i __ros_arr_len)
(let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (aref vals i) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (aref vals i) __ros_str_idx) (code-char (read-byte istream))))))))
  msg
)
(defmethod ros-datatype ((msg (eql '<RequestedJointStates-request>)))
  "Returns string type for a service object of type '<RequestedJointStates-request>"
  "joint_state_publisher/RequestedJointStatesRequest")
(defmethod md5sum ((type (eql '<RequestedJointStates-request>)))
  "Returns md5sum for a message object of type '<RequestedJointStates-request>"
  "42807e19c10a36f2522bfa472fc172f4")
(defmethod message-definition ((type (eql '<RequestedJointStates-request>)))
  "Returns full string definition for message of type '<RequestedJointStates-request>"
  (format nil "string[] names~%~%~%"))
(defmethod serialization-length ((msg <RequestedJointStates-request>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'names) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4 (length ele))))
))
(defmethod ros-message-to-list ((msg <RequestedJointStates-request>))
  "Converts a ROS message object to a list"
  (list '<RequestedJointStates-request>
    (cons ':names (names-val msg))
))
;//! \htmlinclude RequestedJointStates-response.msg.html

(defclass <RequestedJointStates-response> (ros-message)
  ((found
    :reader found-val
    :initarg :found
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0)))
)
(defmethod serialize ((msg <RequestedJointStates-response>) ostream)
  "Serializes a message object of type '<RequestedJointStates-response>"
  (let ((__ros_arr_len (length (slot-value msg 'found))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'found))
)
(defmethod deserialize ((msg <RequestedJointStates-response>) istream)
  "Deserializes a message object of type '<RequestedJointStates-response>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'found) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'found)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<RequestedJointStates-response>)))
  "Returns string type for a service object of type '<RequestedJointStates-response>"
  "joint_state_publisher/RequestedJointStatesResponse")
(defmethod md5sum ((type (eql '<RequestedJointStates-response>)))
  "Returns md5sum for a message object of type '<RequestedJointStates-response>"
  "42807e19c10a36f2522bfa472fc172f4")
(defmethod message-definition ((type (eql '<RequestedJointStates-response>)))
  "Returns full string definition for message of type '<RequestedJointStates-response>"
  (format nil "~%uint32[] found~%~%"))
(defmethod serialization-length ((msg <RequestedJointStates-response>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'found) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
))
(defmethod ros-message-to-list ((msg <RequestedJointStates-response>))
  "Converts a ROS message object to a list"
  (list '<RequestedJointStates-response>
    (cons ':found (found-val msg))
))
(defmethod service-request-type ((msg (eql 'RequestedJointStates)))
  '<RequestedJointStates-request>)
(defmethod service-response-type ((msg (eql 'RequestedJointStates)))
  '<RequestedJointStates-response>)
(defmethod ros-datatype ((msg (eql 'RequestedJointStates)))
  "Returns string type for a service object of type '<RequestedJointStates>"
  "joint_state_publisher/RequestedJointStates")
