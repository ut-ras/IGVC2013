; Auto-generated. Do not edit!


(in-package sogp_node-msg)


;//! \htmlinclude TrainDatapoint.msg.html

(defclass <TrainDatapoint> (ros-message)
  ((predictor
    :accessor predictor-val
    :initarg :predictor
    :initform (make-instance 'sogp_node-msg:<Vector>))
   (target
    :accessor target-val
    :initarg :target
    :initform (make-instance 'sogp_node-msg:<Vector>)))
)
(defmethod serialize ((msg <TrainDatapoint>) ostream)
  "Serializes a message object of type '<TrainDatapoint>"
  (serialize (slot-value msg 'predictor) ostream)
  (serialize (slot-value msg 'target) ostream)
)
(defmethod deserialize ((msg <TrainDatapoint>) istream)
  "Deserializes a message object of type '<TrainDatapoint>"
  (deserialize (slot-value msg 'predictor) istream)
  (deserialize (slot-value msg 'target) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<TrainDatapoint>)))
  "Returns string type for a message object of type '<TrainDatapoint>"
  "sogp_node/TrainDatapoint")
(defmethod md5sum ((type (eql '<TrainDatapoint>)))
  "Returns md5sum for a message object of type '<TrainDatapoint>"
  #x23cafbf003e8c14d0247eb42376c3366)
(defmethod message-definition ((type (eql '<TrainDatapoint>)))
  "Returns full string definition for message of type '<TrainDatapoint>"
  (format nil "Vector predictor~%Vector target~%================================================================================~%MSG: sogp_node/Vector~%float32[] data~%~%~%"))
(defmethod serialization-length ((msg <TrainDatapoint>))
  (+ 0
     (serialization-length (slot-value msg 'predictor))
     (serialization-length (slot-value msg 'target))
))
(defmethod ros-message-to-list ((msg <TrainDatapoint>))
  "Converts a ROS message object to a list"
  (list '<TrainDatapoint>
    (cons ':predictor (ros-message-to-list (predictor-val msg)))
    (cons ':target (ros-message-to-list (target-val msg)))
))
