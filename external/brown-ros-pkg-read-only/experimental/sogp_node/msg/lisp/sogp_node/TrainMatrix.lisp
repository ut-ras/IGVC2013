; Auto-generated. Do not edit!


(in-package sogp_node-msg)


;//! \htmlinclude TrainMatrix.msg.html

(defclass <TrainMatrix> (ros-message)
  ((predictor
    :accessor predictor-val
    :initarg :predictor
    :initform (make-instance 'sogp_node-msg:<Matrix>))
   (target
    :accessor target-val
    :initarg :target
    :initform (make-instance 'sogp_node-msg:<Matrix>)))
)
(defmethod serialize ((msg <TrainMatrix>) ostream)
  "Serializes a message object of type '<TrainMatrix>"
  (serialize (slot-value msg 'predictor) ostream)
  (serialize (slot-value msg 'target) ostream)
)
(defmethod deserialize ((msg <TrainMatrix>) istream)
  "Deserializes a message object of type '<TrainMatrix>"
  (deserialize (slot-value msg 'predictor) istream)
  (deserialize (slot-value msg 'target) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<TrainMatrix>)))
  "Returns string type for a message object of type '<TrainMatrix>"
  "sogp_node/TrainMatrix")
(defmethod md5sum ((type (eql '<TrainMatrix>)))
  "Returns md5sum for a message object of type '<TrainMatrix>"
  #x9184cbeaeab8137e5760f8151a90613f)
(defmethod message-definition ((type (eql '<TrainMatrix>)))
  "Returns full string definition for message of type '<TrainMatrix>"
  (format nil "Matrix predictor~%Matrix target~%================================================================================~%MSG: sogp_node/Matrix~%Vector[] matrix_rows~%~%================================================================================~%MSG: sogp_node/Vector~%float32[] data~%~%~%"))
(defmethod serialization-length ((msg <TrainMatrix>))
  (+ 0
     (serialization-length (slot-value msg 'predictor))
     (serialization-length (slot-value msg 'target))
))
(defmethod ros-message-to-list ((msg <TrainMatrix>))
  "Converts a ROS message object to a list"
  (list '<TrainMatrix>
    (cons ':predictor (ros-message-to-list (predictor-val msg)))
    (cons ':target (ros-message-to-list (target-val msg)))
))
