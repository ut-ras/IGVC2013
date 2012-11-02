; Auto-generated. Do not edit!


(in-package sogp_node-srv)


;//! \htmlinclude Predict-request.msg.html

(defclass <Predict-request> (ros-message)
  ((predictor
    :accessor predictor-val
    :initarg :predictor
    :initform (make-instance 'sogp_node-msg:<Vector>)))
)
(defmethod serialize ((msg <Predict-request>) ostream)
  "Serializes a message object of type '<Predict-request>"
  (serialize (slot-value msg 'predictor) ostream)
)
(defmethod deserialize ((msg <Predict-request>) istream)
  "Deserializes a message object of type '<Predict-request>"
  (deserialize (slot-value msg 'predictor) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<Predict-request>)))
  "Returns string type for a service object of type '<Predict-request>"
  "sogp_node/PredictRequest")
(defmethod md5sum ((type (eql '<Predict-request>)))
  "Returns md5sum for a message object of type '<Predict-request>"
  #xfba85ef2d1dc3a06205281f14b943df8)
(defmethod message-definition ((type (eql '<Predict-request>)))
  "Returns full string definition for message of type '<Predict-request>"
  (format nil "Vector predictor~%~%"))
(defmethod serialization-length ((msg <Predict-request>))
  (+ 0
     (serialization-length (slot-value msg 'predictor))
))
(defmethod ros-message-to-list ((msg <Predict-request>))
  "Converts a ROS message object to a list"
  (list '<Predict-request>
    (cons ':predictor (ros-message-to-list (predictor-val msg)))
))
;//! \htmlinclude Predict-response.msg.html

(defclass <Predict-response> (ros-message)
  ((prediction
    :accessor prediction-val
    :initarg :prediction
    :initform (make-instance 'sogp_node-msg:<Vector>))
   (error_msg
    :accessor error_msg-val
    :initarg :error_msg
    :initform ""))
)
(defmethod serialize ((msg <Predict-response>) ostream)
  "Serializes a message object of type '<Predict-response>"
  (serialize (slot-value msg 'prediction) ostream)
  (let ((__ros_str_len (length (slot-value msg 'error_msg))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'error_msg))
)
(defmethod deserialize ((msg <Predict-response>) istream)
  "Deserializes a message object of type '<Predict-response>"
  (deserialize (slot-value msg 'prediction) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'error_msg) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'error_msg) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<Predict-response>)))
  "Returns string type for a service object of type '<Predict-response>"
  "sogp_node/PredictResponse")
(defmethod md5sum ((type (eql '<Predict-response>)))
  "Returns md5sum for a message object of type '<Predict-response>"
  #xfba85ef2d1dc3a06205281f14b943df8)
(defmethod message-definition ((type (eql '<Predict-response>)))
  "Returns full string definition for message of type '<Predict-response>"
  (format nil "Vector prediction~%string error_msg~%================================================================================~%MSG: sogp_node/Vector~%float32[] data~%~%~%"))
(defmethod serialization-length ((msg <Predict-response>))
  (+ 0
     (serialization-length (slot-value msg 'prediction))
     4 (length (slot-value msg 'error_msg))
))
(defmethod ros-message-to-list ((msg <Predict-response>))
  "Converts a ROS message object to a list"
  (list '<Predict-response>
    (cons ':prediction (ros-message-to-list (prediction-val msg)))
    (cons ':error_msg (ros-message-to-list (error_msg-val msg)))
))
(defmethod service-request-type ((msg (eql 'Predict)))
  '<Predict-request>)
(defmethod service-response-type ((msg (eql 'Predict)))
  '<Predict-response>)
(defmethod ros-datatype ((msg (eql 'Predict)))
  "Returns string type for a service object of type '<Predict>"
  "sogp_node/Predict")
