; Auto-generated. Do not edit!


(in-package sogp_node-msg)


;//! \htmlinclude TrainPair.msg.html

(defclass <TrainPair> (ros-message)
  ((predictor
    :accessor predictor-val
    :initarg :predictor
    :initform (make-instance 'sogp_node-msg:<Datapoint>))
   (target
    :accessor target-val
    :initarg :target
    :initform (make-instance 'sogp_node-msg:<Datapoint>)))
)
(defmethod serialize ((msg <TrainPair>) ostream)
  "Serializes a message object of type '<TrainPair>"
  (serialize (slot-value msg 'predictor) ostream)
  (serialize (slot-value msg 'target) ostream)
)
(defmethod deserialize ((msg <TrainPair>) istream)
  "Deserializes a message object of type '<TrainPair>"
  (deserialize (slot-value msg 'predictor) istream)
  (deserialize (slot-value msg 'target) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<TrainPair>)))
  "Returns string type for a message object of type '<TrainPair>"
  "sogp_node/TrainPair")
(defmethod md5sum ((type (eql '<TrainPair>)))
  "Returns md5sum for a message object of type '<TrainPair>"
  #x23cafbf003e8c14d0247eb42376c3366)
(defmethod message-definition ((type (eql '<TrainPair>)))
  "Returns full string definition for message of type '<TrainPair>"
  (format nil "Datapoint predictor~%Datapoint target~%================================================================================~%MSG: sogp_node/Datapoint~%float32[] data~%~%~%"))
(defmethod serialization-length ((msg <TrainPair>))
  (+ 0
     (serialization-length (slot-value msg 'predictor))
     (serialization-length (slot-value msg 'target))
))
(defmethod ros-message-to-list ((msg <TrainPair>))
  "Converts a ROS message object to a list"
  (list '<TrainPair>
    (cons ':predictor (ros-message-to-list (predictor-val msg)))
    (cons ':target (ros-message-to-list (target-val msg)))
))
