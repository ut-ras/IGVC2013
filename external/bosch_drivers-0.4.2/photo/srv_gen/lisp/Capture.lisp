; Auto-generated. Do not edit!


(cl:in-package photo-srv)


;//! \htmlinclude Capture-request.msg.html

(cl:defclass <Capture-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Capture-request (<Capture-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Capture-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Capture-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name photo-srv:<Capture-request> is deprecated: use photo-srv:Capture-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Capture-request>) ostream)
  "Serializes a message object of type '<Capture-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Capture-request>) istream)
  "Deserializes a message object of type '<Capture-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Capture-request>)))
  "Returns string type for a service object of type '<Capture-request>"
  "photo/CaptureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Capture-request)))
  "Returns string type for a service object of type 'Capture-request"
  "photo/CaptureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Capture-request>)))
  "Returns md5sum for a message object of type '<Capture-request>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Capture-request)))
  "Returns md5sum for a message object of type 'Capture-request"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Capture-request>)))
  "Returns full string definition for message of type '<Capture-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Capture-request)))
  "Returns full string definition for message of type 'Capture-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Capture-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Capture-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Capture-request
))
;//! \htmlinclude Capture-response.msg.html

(cl:defclass <Capture-response> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass Capture-response (<Capture-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Capture-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Capture-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name photo-srv:<Capture-response> is deprecated: use photo-srv:Capture-response instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <Capture-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader photo-srv:image-val is deprecated.  Use photo-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Capture-response>) ostream)
  "Serializes a message object of type '<Capture-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Capture-response>) istream)
  "Deserializes a message object of type '<Capture-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Capture-response>)))
  "Returns string type for a service object of type '<Capture-response>"
  "photo/CaptureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Capture-response)))
  "Returns string type for a service object of type 'Capture-response"
  "photo/CaptureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Capture-response>)))
  "Returns md5sum for a message object of type '<Capture-response>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Capture-response)))
  "Returns md5sum for a message object of type 'Capture-response"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Capture-response>)))
  "Returns full string definition for message of type '<Capture-response>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Capture-response)))
  "Returns full string definition for message of type 'Capture-response"
  (cl:format cl:nil "sensor_msgs/Image image~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Capture-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Capture-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Capture-response
    (cl:cons ':image (image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Capture)))
  'Capture-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Capture)))
  'Capture-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Capture)))
  "Returns string type for a service object of type '<Capture>"
  "photo/Capture")