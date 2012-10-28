
(in-package :asdf)

(defsystem "ocean_server_imu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
 :roslib-msg
)
  :components ((:file "_package")
    (:file "RawData" :depends-on ("_package"))
    (:file "_package_RawData" :depends-on ("_package"))
    ))
